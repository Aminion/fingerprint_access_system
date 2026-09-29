use core::sync::atomic::Ordering;

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use crate::fingerprint_irq_task::{FINGERPRINT_IRQ_STATUS, RESYNC_SIGNAL, SENSOR_ACTIVE};
use crate::fingerprint_sensor::{ConfirmationCode, FingerError, LedColor, LedEffect, LedMode};
use crate::FingerprintSensor;

pub static FINGERPRINT_CHANNEL: Channel<CriticalSectionRawMutex, SensorCommand, 1> = Channel::new();

#[derive(Clone, Copy)]
pub enum SensorCommand {
    ValidateAccess(&'static Signal<CriticalSectionRawMutex, bool>),
    EnrollNewUser(&'static Signal<CriticalSectionRawMutex, ()>),
}

static EFFECT_IN_PROGRESS: LedEffect = LedEffect {
    mode: LedMode::AlwaysOn,
    speed: 0x20,
    color: LedColor::Purple,
    cycles: 0x0,
};
static EFFECT_SUCCESS: LedEffect = LedEffect {
    mode: LedMode::Breathing,
    speed: 0x20,
    color: LedColor::Blue,
    cycles: 0x3,
};
static EFFECT_FAIL: LedEffect = LedEffect {
    mode: LedMode::Flashing,
    speed: 0x20,
    color: LedColor::Red,
    cycles: 0x3,
};

/// Scan attempts allowed per presence event before giving up. Bounded so a
/// finger parked on the sensor can't wedge unlock_task, which stays blocked on
/// the result signal for the whole window.
const MAX_VALIDATE_ATTEMPTS: u8 = 5;
/// Pause between attempts: gives the user time to reposition and keeps a failed
/// scan from spinning the UART flat out on battery.
const VALIDATE_RETRY_DELAY_MS: u64 = 300;

/// RAII guard that marks a sensor command window.
/// Sets SENSOR_ACTIVE on creation and clears it + triggers an IRQ resync on drop,
/// so the IRQ task ignores WAKE pin glitches while any sensor command is in flight.
struct SensorGuard;

impl SensorGuard {
    fn new() -> Self {
        SENSOR_ACTIVE.store(true, Ordering::Relaxed);
        SensorGuard
    }
}

impl Drop for SensorGuard {
    fn drop(&mut self) {
        SENSOR_ACTIVE.store(false, Ordering::Relaxed);
        RESYNC_SIGNAL.signal(());
    }
}

#[embassy_executor::task]
pub async fn fingerprint_manager_task(mut sensor: FingerprintSensor) {
    async fn finger_on_sensor(expected_state: bool) {
        let mut receiver = FINGERPRINT_IRQ_STATUS.receiver().expect("FINGERPRINT_IRQ_STATUS: all receiver slots taken");
        loop {
            if receiver.get().await == expected_state {
                return;
            }
            receiver.changed().await;
        }
    }

    loop {
        let cmd = FINGERPRINT_CHANNEL.receive().await;

        {
            let _g = SensorGuard::new();
            sensor.enable().await.ok();
        }

        match cmd {
            SensorCommand::ValidateAccess(signal) => {
                // AlwaysOn with cycles: 0 — set once, stays lit across retries.
                { let _g = SensorGuard::new(); sensor.led(&EFFECT_IN_PROGRESS).await.ok(); }

                let mut matched = false;
                for attempt in 1..=MAX_VALIDATE_ATTEMPTS {
                    let result: Result<_, FingerError> = async {
                        let _g = SensorGuard::new();
                        sensor.generate_image().await?;
                        sensor.image_to_template(1).await?;
                        sensor.search_database(1, 0, 200).await?;
                        Ok(())
                    }
                    .await;

                    match result {
                        Ok(()) => {
                            matched = true;
                            break;
                        }
                        // The sensor is the presence oracle: GenImg answers
                        // NoFinger once the finger is lifted. The WAKE pin can't
                        // serve here — SENSOR_ACTIVE deliberately suppresses its
                        // HIGH edges for the whole command window.
                        Err(FingerError::Sensor(ConfirmationCode::NoFinger)) => break,
                        // Still on the sensor but unusable (NoMatch, ImageMessy,
                        // FeatureFail) or a transient bus error — scan again.
                        Err(_) => {
                            if attempt < MAX_VALIDATE_ATTEMPTS {
                                Timer::after_millis(VALIDATE_RETRY_DELAY_MS).await;
                            }
                        }
                    }
                }
                signal.signal(matched);

                // Send the effect inside a command window, then wait it out
                // *outside* one. Holding the guard across the animation would
                // blind the IRQ task to a lift-and-press during those seconds:
                // the lift is suppressed as a glitch, the press produces no
                // edge (state is already true), and the resync that follows
                // sees no change — so unlock_task, waiting on a false->true
                // transition, would never re-arm and the press is lost.
                let effect = if matched { &EFFECT_SUCCESS } else { &EFFECT_FAIL };
                { let _g = SensorGuard::new(); sensor.led(effect).await.ok(); }
                Timer::after_millis(effect.wait_ms()).await;
            }

            SensorCommand::EnrollNewUser(signal) => loop {
                let result: Result<_, FingerError> = async {
                    for i in 1..=2u8 {
                        // Command window — WAKE glitches suppressed.
                        { let _g = SensorGuard::new(); sensor.led(&EFFECT_IN_PROGRESS).await?; }
                        // IRQ active here: finger_on_sensor() can observe real pin state.
                        finger_on_sensor(true).await;
                        // Command window for scan — the critical glitch zone.
                        { let _g = SensorGuard::new(); sensor.generate_image().await?; sensor.image_to_template(i).await?; }
                        // Command window for LED feedback.
                        { let _g = SensorGuard::new(); sensor.led(&EFFECT_SUCCESS).await?; }
                        // IRQ active here: wait for finger removal.
                        finger_on_sensor(false).await;
                    }
                    { let _g = SensorGuard::new(); sensor.create_model().await?; }
                    { let _g = SensorGuard::new(); sensor.store_template(1, 1).await?; }
                    Ok(())
                }
                .await;

                if result.is_ok() {
                    { let _g = SensorGuard::new(); sensor.led_await(&EFFECT_SUCCESS).await.ok(); }
                    finger_on_sensor(false).await;
                    signal.signal(());
                    break;
                }
                // error: retry without signaling — caller stays blocked until success
            },
        }

        sensor.disable();
        RESYNC_SIGNAL.signal(());
    }
}

#[embassy_executor::task]
pub async fn add_new_finger_task(mut pin: ExtiInput<'static, Async>) {
    static DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    loop {
        pin.wait_for_low().await;

        FINGERPRINT_CHANNEL
            .send(SensorCommand::EnrollNewUser(&DONE))
            .await;
        DONE.wait().await;
    }
}
