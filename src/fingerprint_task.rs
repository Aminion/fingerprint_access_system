use embassy_stm32 as _;
use embassy_stm32 as _;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::PB8;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Timer;

use crate::fingerprint_irq_task::FINGERPRINT_IRQ_STATUS;
use crate::fingerprint_sensor::{FingerError, LedColor, LedMode};
use crate::MySensor;

pub static FINGERPRINT_CHANNEL: Channel<CriticalSectionRawMutex, CommandEnvelope, 1> =
    Channel::new();

#[derive(Clone, Copy)] // Commands should be small and easy to copy

pub enum SensorCommand {
    ValidateAccess,
    EnrollNewUser,
}

pub struct CommandEnvelope {
    pub cmd: SensorCommand,
    pub ending_signal: &'static Signal<CriticalSectionRawMutex, ()>,
}

#[embassy_executor::task]
pub async fn fingerprint_manager_task(mut sensor: MySensor) {
    async fn led(sensor: &mut MySensor, color: LedColor) -> Result<(), FingerError> {
        sensor
            .control_led(LedMode::AlwaysOn, 0x20, color, 0x3)
            .await
    }
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
        match cmd.cmd {
            SensorCommand::ValidateAccess => {
                let result: Result<_, FingerError> = async {
                    led(&mut sensor, LedColor::Purple).await?;
                    sensor.generate_image().await?;
                    sensor.image_to_template(1).await?;
                    sensor.search_database(1, 0, 200).await?;
                    Ok(())
                }
                .await;
                if result.is_ok() {
                    sensor
                        .control_led(
                            LedMode::Breathing,
                            0xFF, // Speed: 0x20 is a nice, slow human-like breath. 0xFF is "turbo" speed.
                            LedColor::Blue,
                            0x3, // Cycles: 0xFF usually means "Infinite"
                        )
                        .await
                        .ok();
                } else {
                    sensor
                        .control_led(
                            LedMode::Breathing,
                            0x20, // Speed: 0x20 is a nice, slow human-like breath. 0xFF is "turbo" speed.
                            LedColor::Red,
                            0x3, // Cycles: 0xFF usually means "Infinite"
                        )
                        .await
                        .ok();
                }
            }
            SensorCommand::EnrollNewUser => loop {
                let result: Result<_, FingerError> = async {
                    for i in 1..=2u8 {
                        led(&mut sensor, LedColor::Purple).await?;
                        finger_on_sensor(true).await;
                        sensor.generate_image().await?;
                        sensor.image_to_template(i).await?;
                        led(&mut sensor, LedColor::Blue).await?;
                        finger_on_sensor(false).await;
                    }
                    sensor.create_model().await?;
                    sensor.store_template(1, 1).await?;

                    Ok(())
                }
                .await;
                if result.is_ok() {
                    sensor
                        .control_led(
                            LedMode::Flashing,
                            0x20, // Speed: 0x20 is a nice, slow human-like breath. 0xFF is "turbo" speed.
                            LedColor::Blue,
                            0x3, // Cycles: 0xFF usually means "Infinite"
                        )
                        .await
                        .ok();
                    finger_on_sensor(false).await;
                    break;
                }
            },
        }
        cmd.ending_signal.signal(());
    }
}

#[embassy_executor::task]
pub async fn add_new_finger_task(mut pin: ExtiInput<'static, PB8>) {
    static DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    loop {
        pin.wait_for_low().await;

        FINGERPRINT_CHANNEL
            .send(CommandEnvelope {
                cmd: SensorCommand::EnrollNewUser,
                ending_signal: &DONE,
            })
            .await;
        DONE.wait().await;
    }
}
