#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]

use core::u64;

use crate::fingerprint_sensor::{FingerError, LedColor, LedMode};
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32 as _;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, OutputType, Pull};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{InterruptHandler, Uart};
use embassy_sync::signal::Signal;
use embassy_time::{Instant, Timer};

use {defmt_rtt as _, panic_probe as _};

use embassy_stm32 as _;
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2, PB2, PB8, TIM1, USART1}; // Make sure to import this!

mod fingerprint_sensor;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, CommandEnvelope, 1> = Channel::new();
use embassy_sync::watch::Watch;

static IRQ: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();

pub struct CommandEnvelope {
    pub cmd: SensorCommand,
    pub reply_signal: &'static Signal<CriticalSectionRawMutex, ()>,
}

#[derive(Clone, Copy)] // Commands should be small and easy to copy

pub enum SensorCommand {
    ValidateAccess,
    EnrollNewUser(u16), // You can pass data like a Target ID
    Cancel,
}

#[embassy_executor::task]
async fn button_listener3_task(mut pin: ExtiInput<'static, PB2>) {
    let sender = IRQ.sender();

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
async fn button_listener_task() {
    static INIT_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    let mut receiver = IRQ.receiver().unwrap();
    loop {
        let _ = receiver.changed_and(|v| *v).await;
        let result = COMMAND_CHANNEL.try_send(CommandEnvelope {
            cmd: SensorCommand::ValidateAccess,
            reply_signal: &INIT_DONE,
        });
        if result.is_ok() {
            INIT_DONE.wait().await;
        }
    }
}

#[embassy_executor::task]
async fn button_listener2_task(mut pin: ExtiInput<'static, PB8>) {
    static INIT_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    loop {
        pin.wait_for_low().await;
        COMMAND_CHANNEL
            .send(CommandEnvelope {
                cmd: SensorCommand::EnrollNewUser(0),
                reply_signal: &INIT_DONE,
            })
            .await;
        INIT_DONE.wait().await;
    }
}

pub type MySensor = fingerprint_sensor::FingerprintSensor<'static, USART1, DMA1_CH1, DMA1_CH2>;

#[embassy_executor::task]
async fn fingerprint_manager_task(mut sensor: MySensor) {
    async fn led(sensor: &mut MySensor, color: LedColor) -> Result<(), FingerError> {
        sensor
            .control_led(LedMode::AlwaysOn, 0x20, color, 0x3)
            .await
    }
    async fn finger_on_sensor(expected_state: bool) {
        let mut receiver = IRQ.receiver().unwrap();
        Timer::after_millis(1000).await;
        loop {
            if receiver.get().await == expected_state {
                return;
            }
            receiver.changed().await;
        }
    }
    loop {
        // Wait for any button to send a command
        let cmd = COMMAND_CHANNEL.receive().await;
        match cmd.cmd {
            SensorCommand::ValidateAccess => {
                info!("Validating access...");
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
            SensorCommand::EnrollNewUser(target_id) => loop {
                info!("Enrolling new user...");
                let result: Result<_, FingerError> = async {
                    for i in 1..=2u8 {
                        info!("{}", i);
                        led(&mut sensor, LedColor::Purple).await?;
                        finger_on_sensor(true).await;
                        info!("DOWN");
                        sensor.generate_image().await?;
                        sensor.image_to_template(i).await?;
                        led(&mut sensor, LedColor::Blue).await?;
                        finger_on_sensor(false).await;
                        info!("UP");
                    }
                    sensor.create_model().await?;
                    sensor.store_template(1, 1).await?;

                    Ok(())
                }
                .await;
                if result.is_ok() {
                    info!("Finger enrolled successfully!");
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
        COMMAND_CHANNEL.clear();
        cmd.reply_signal.signal(());
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
    IRQ.sender().send(false);
    let p = embassy_stm32::init(Default::default());

    // Setup button input on PB2 (D7)
    let button_pin = Input::new(p.PB2, Pull::Up);
    let button_exti = ExtiInput::new(button_pin, p.EXTI2);

    let button_pin = Input::new(p.PB8, Pull::Up);
    let button2_exti = ExtiInput::new(button_pin, p.EXTI8);

    // 2. Setup PWM for the Solenoid (PA8 / D9)
    let pwm = SimplePwm::new(
        p.TIM1,                                             // Timer 1
        Some(PwmPin::new_ch1(p.PA8, OutputType::PushPull)), // PA8 is D9
        None,
        None,
        None,
        hz(10000),
        Default::default(),
    );

    spawner.spawn(button_listener_task()).unwrap();
    spawner.spawn(button_listener2_task(button2_exti)).unwrap();
    spawner.spawn(button_listener3_task(button_exti)).unwrap();

    let mut uart_config = embassy_stm32::usart::Config::default();
    uart_config.baudrate = 57_600;
    // 1. Setup UART (Do NOT split yet)
    let uart = Uart::new(
        p.USART1,
        p.PA10, // RX Pin (Connects to Sensor Yellow/TX)
        p.PA9,  // TX Pin (Connects to Sensor White/RX)
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        uart_config,
    )
    .unwrap();

    let mut s = fingerprint_sensor::FingerprintSensor::new(
        uart,
        [0xFF, 0xFF, 0xFF, 0xFF],
        [0x00, 0x00, 0x00, 0x00],
    );

    info!("Password verification");
    let _ = s.verify_password().await;

    spawner.spawn(fingerprint_manager_task(s)).unwrap();
    loop {
        Timer::after_millis(1000000).await;
    }
}
