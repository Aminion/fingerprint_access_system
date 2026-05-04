use defmt::println;
use embassy_stm32 as _;
use embassy_stm32 as _;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::PA8;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Timer;

use crate::fingerprint_irq_task::FINGERPRINT_IRQ_STATUS;
use crate::fingerprint_sensor::{FingerError, LedColor, LedEffect, LedMode};
use crate::FingerprintSensor;

pub static FINGERPRINT_CHANNEL: Channel<CriticalSectionRawMutex, SensorCommand, 1> = Channel::new();

#[derive(Clone, Copy)] // Commands should be small and easy to copy

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

#[embassy_executor::task]
pub async fn fingerprint_manager_task(mut sensor: FingerprintSensor) {
    async fn finger_on_sensor(expected_state: bool) {
        let mut receiver = FINGERPRINT_IRQ_STATUS.receiver().unwrap();
        Timer::after_millis(1000).await;
        loop {
            if receiver.get().await == expected_state {
                return;
            }
            receiver.changed().await;
        }
    }
    loop {
        let cmd = FINGERPRINT_CHANNEL.receive().await;
        println!("requested");
        let _ = sensor.enable().await;
        println!("enabled");
        match cmd {
            SensorCommand::ValidateAccess(signal) => {
                let result: Result<_, FingerError> = async {
                    sensor.led(&EFFECT_IN_PROGRESS).await?;
                    sensor.generate_image().await?;
                    sensor.image_to_template(1).await?;
                    sensor.search_database(1, 0, 200).await?;
                    Ok(())
                }
                .await;
                signal.signal(result.is_ok());
                if result.is_ok() {
                    sensor.led_await(&EFFECT_SUCCESS).await.ok();
                } else {
                    sensor.led_await(&EFFECT_FAIL).await.ok();
                }
            }
            SensorCommand::EnrollNewUser(signal) => loop {
                let result: Result<_, FingerError> = async {
                    for i in 1..=2u8 {
                        sensor.led(&EFFECT_IN_PROGRESS).await?;
                        finger_on_sensor(true).await;
                        sensor.generate_image().await?;
                        sensor.image_to_template(i).await?;
                        sensor.led(&EFFECT_SUCCESS).await?;
                        finger_on_sensor(false).await;
                    }
                    sensor.create_model().await?;
                    sensor.store_template(1, 1).await?;

                    Ok(())
                }
                .await;
                signal.signal(());
                if result.is_ok() {
                    sensor.led_await(&EFFECT_SUCCESS).await.ok();
                    finger_on_sensor(false).await;
                    break;
                }
            },
        }
        sensor.disable();
    }
}

#[embassy_executor::task]
pub async fn add_new_finger_task(mut pin: ExtiInput<'static, PA8>) {
    static DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    loop {
        pin.wait_for_low().await;

        FINGERPRINT_CHANNEL
            .send(SensorCommand::EnrollNewUser(&DONE))
            .await;
        DONE.wait().await;
    }
}
