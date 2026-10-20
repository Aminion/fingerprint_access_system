use core::sync::atomic::Ordering;

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use crate::fingerprint_irq_task::{FINGERPRINT_IRQ_STATUS, RESYNC_SIGNAL, SENSOR_ACTIVE};
use crate::fingerprint_sensor::{
    ConfirmationCode, FingerError, GenImg, Img2Tz, LedColor, LedEffect, LedMode, RegModel, Search, Store,
    LIBRARY_CAPACITY,
};
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

trait CommandWindow {
    /// Runs `f` inside a sensor command window. The window closes when `f`
    /// completes, fails, or the future is cancelled.
    async fn window<T>(&mut self, f: impl AsyncFnOnce(&mut Self) -> T) -> T;
}

impl CommandWindow for FingerprintSensor {
    async fn window<T>(&mut self, f: impl AsyncFnOnce(&mut Self) -> T) -> T {
        let _g = SensorGuard::new();
        f(self).await
    }
}

async fn finger_on_sensor(expected_state: bool) {
    let mut receiver = FINGERPRINT_IRQ_STATUS.receiver().expect("FINGERPRINT_IRQ_STATUS: all receiver slots taken");
    loop {
        if receiver.get().await == expected_state {
            return;
        }
        receiver.changed().await;
    }
}

#[embassy_executor::task]
pub async fn fingerprint_manager_task(mut sensor: FingerprintSensor) {
    loop {
        let cmd = FINGERPRINT_CHANNEL.receive().await;
        sensor.window(async |s| s.enable().await.ok()).await;

        match cmd {
            SensorCommand::ValidateAccess(signal) => validate_access(&mut sensor, signal).await,
            SensorCommand::EnrollNewUser(signal) => {
                // Retry without signaling — the caller stays blocked until success.
                while enroll(&mut sensor).await.is_err() {}
                sensor.window(async |s| s.led_await(EFFECT_SUCCESS).await.ok()).await;
                finger_on_sensor(false).await;
                signal.signal(());
            }
        }

        sensor.disable();
        RESYNC_SIGNAL.signal(());
    }
}

async fn validate_access(sensor: &mut FingerprintSensor, signal: &Signal<CriticalSectionRawMutex, bool>) {
    // AlwaysOn with cycles: 0 — set once, stays lit across retries.
    sensor.window(async |s| s.run(EFFECT_IN_PROGRESS).await.ok()).await;

    let matched = scan_and_match(sensor).await;
    signal.signal(matched);

    // Send the effect inside a command window, then wait it out
    // *outside* one. Holding the guard across the animation would
    // blind the IRQ task to a lift-and-press during those seconds:
    // the lift is suppressed as a glitch, the press produces no
    // edge (state is already true), and the resync that follows
    // sees no change — so unlock_task, waiting on a false->true
    // transition, would never re-arm and the press is lost.
    let effect = if matched { EFFECT_SUCCESS } else { EFFECT_FAIL };
    sensor.window(async |s| s.run(effect).await.ok()).await;
    Timer::after_millis(effect.wait_ms()).await;
}

/// Scans until a match, the finger is lifted, or attempts run out.
async fn scan_and_match(sensor: &mut FingerprintSensor) -> bool {
    for attempt in 1..=MAX_VALIDATE_ATTEMPTS {
        let result = sensor
            .window(async |s| -> Result<_, FingerError> {
                s.run(GenImg).await?;
                s.run(Img2Tz(1)).await?;
                s.run(Search { buffer: 1, start: 0, count: LIBRARY_CAPACITY }).await
            })
            .await;

        match result {
            Ok(m) => {
                defmt::info!("fingerprint match: slot {} score {}", m.slot, m.score);
                return true;
            }
            // The sensor is the presence oracle: GenImg answers
            // NoFinger once the finger is lifted. The WAKE pin can't
            // serve here — SENSOR_ACTIVE deliberately suppresses its
            // HIGH edges for the whole command window.
            Err(FingerError::Sensor(ConfirmationCode::NoFinger)) => return false,
            // Still on the sensor but unusable (NoMatch, ImageMessy,
            // FeatureFail) or a transient bus error — scan again.
            Err(_) if attempt < MAX_VALIDATE_ATTEMPTS => Timer::after_millis(VALIDATE_RETRY_DELAY_MS).await,
            Err(_) => {}
        }
    }
    false
}

async fn enroll(sensor: &mut FingerprintSensor) -> Result<(), FingerError> {
    // Pick the slot before scanning so a full library fails
    // without making the user press twice for nothing.
    let slot = sensor.window(async |s| s.find_free_slot().await).await?;
    for i in 1..=2u8 {
        // Command window — WAKE glitches suppressed.
        sensor.window(async |s| s.run(EFFECT_IN_PROGRESS).await).await?;
        // IRQ active here: finger_on_sensor() can observe real pin state.
        finger_on_sensor(true).await;
        // Command window for scan — the critical glitch zone.
        sensor
            .window(async |s| {
                s.run(GenImg).await?;
                s.run(Img2Tz(i)).await
            })
            .await?;
        // Command window for LED feedback.
        sensor.window(async |s| s.run(EFFECT_SUCCESS).await).await?;
        // IRQ active here: wait for finger removal.
        finger_on_sensor(false).await;
    }
    sensor
        .window(async |s| {
            s.run(RegModel).await?;
            s.run(Store { buffer: 1, slot }).await
        })
        .await
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
