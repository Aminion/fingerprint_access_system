#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]

use core::u64;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32 as _;
use embassy_stm32 as _;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, OutputType, Pull};
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2, PB2, PB8, TIM1, USART1};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{InterruptHandler, Uart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::Timer;

use crate::fingerprint_sensor::{FingerError, LedColor, LedMode};
mod fingerprint_sensor;

static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, CommandEnvelope, 1> = Channel::new();

use {defmt_rtt as _, panic_probe as _};
pub type MySensor = fingerprint_sensor::FingerprintSensor<'static, USART1, DMA1_CH1, DMA1_CH2>;
static FINGERPRINT_IRQ_STATUS: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();

const SENSOR_ADRESS: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFF];
const SENSOR_PASSWORD: [u8; 4] = [0x00, 0x00, 0x00, 0x00];

pub struct CommandEnvelope {
    pub cmd: SensorCommand,
    pub ending_signal: &'static Signal<CriticalSectionRawMutex, ()>,
}

#[derive(Clone, Copy)] // Commands should be small and easy to copy

pub enum SensorCommand {
    ValidateAccess,
    EnrollNewUser,
}

#[embassy_executor::task]
async fn fingerprint_irq_task(mut pin: ExtiInput<'static, PB2>) {
    let sender = FINGERPRINT_IRQ_STATUS.sender();

    loop {
        pin.wait_for_any_edge().await;
        let f_sample = pin.is_low();
        Timer::after_millis(100).await;
        let s_sample = pin.is_low();
        if f_sample == s_sample {
            sender.send(s_sample);
        }
    }
}

#[embassy_executor::task]
async fn unlock_task() {
    static UNLOCK_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    let mut receiver = FINGERPRINT_IRQ_STATUS.receiver().unwrap();
    loop {
        let _ = receiver.changed_and(|v| *v).await;
        let _ = COMMAND_CHANNEL.try_send(CommandEnvelope {
            cmd: SensorCommand::ValidateAccess,
            ending_signal: &UNLOCK_DONE,
        });
        UNLOCK_DONE.wait().await;
    }
}

#[embassy_executor::task]
async fn add_new_finger_task(mut pin: ExtiInput<'static, PB8>) {
    static INIT_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    loop {
        pin.wait_for_low().await;

        COMMAND_CHANNEL
            .send(CommandEnvelope {
                cmd: SensorCommand::EnrollNewUser,
                ending_signal: &INIT_DONE,
            })
            .await;
        INIT_DONE.wait().await;
    }
}

#[embassy_executor::task]
async fn fingerprint_manager_task(mut sensor: MySensor) {
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
        let cmd = COMMAND_CHANNEL.receive().await;
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
            _ => (),
        }
        cmd.ending_signal.signal(());
    }
}

async fn unlock(pwm: &mut SimplePwm<'static, TIM1>) {
    const CICK_MS: u64 = 50;
    const HOLD_MS: u64 = 3000;
    const HOLD_DUTY_DIVIDER: u16 = 3;
    const CHANNEL: embassy_stm32::timer::Channel = embassy_stm32::timer::Channel::Ch1;

    let max_duty = pwm.get_max_duty();
    pwm.enable(CHANNEL);
    pwm.set_duty(CHANNEL, max_duty);
    Timer::after_millis(CICK_MS).await;
    pwm.set_duty(CHANNEL, max_duty / HOLD_DUTY_DIVIDER);
    Timer::after_millis(HOLD_MS).await;
    pwm.set_duty(CHANNEL, 0);
    pwm.disable(CHANNEL);
}

bind_interrupts!(struct Irqs {
    USART1 => InterruptHandler<embassy_stm32::peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    FINGERPRINT_IRQ_STATUS.sender().send(false);
    let p = embassy_stm32::init(Default::default());
    let button_pin = Input::new(p.PB2, Pull::Up);
    let button_exti = ExtiInput::new(button_pin, p.EXTI2);
    let button_pin = Input::new(p.PB8, Pull::Up);
    let button2_exti = ExtiInput::new(button_pin, p.EXTI8);

    let pwm = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new_ch1(p.PA8, OutputType::PushPull)),
        None,
        None,
        None,
        hz(10000),
        Default::default(),
    );

    let mut uart_config = embassy_stm32::usart::Config::default();
    uart_config.baudrate = 57_600;
    let uart = Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        uart_config,
    )
    .unwrap();

    let mut sensor =
        fingerprint_sensor::FingerprintSensor::new(uart, SENSOR_ADRESS, SENSOR_PASSWORD);
    let _ = sensor.verify_password().await;

    spawner.spawn(unlock_task()).unwrap();
    spawner.spawn(add_new_finger_task(button2_exti)).unwrap();
    spawner.spawn(fingerprint_irq_task(button_exti)).unwrap();
    spawner.spawn(fingerprint_manager_task(sensor)).unwrap();

    loop {
        Timer::after_secs(u32::MAX as u64).await;
    }
}
