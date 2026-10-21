#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_stm32 as _;
use embassy_stm32::adc::Adc;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed, OutputType};
use embassy_stm32::time::{khz, Hertz};
use embassy_stm32::timer::simple_pwm::{PwmPin, PwmPinConfig, SimplePwm};
use embassy_stm32::usart::Uart;
use {defmt_rtt as _, panic_probe as _};

use crate::fingerprint_sensor::Irqs;
mod battery_monitoring_task;
mod beeper_task;
mod fingerprint_sensor;
mod fingerprint_task;
mod request;
mod unlock_task;

use battery_monitoring_task::battery_monitor_task;
use beeper_task::beeper_task ;
use fingerprint_task::fingerprint_manager_task;
use unlock_task::unlock_task;

pub type FingerprintSensor = fingerprint_sensor::FingerprintSensor<'static>;

const SENSOR_ADDRESS: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFF];
const SENSOR_PASSWORD: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
const SENSOR_BAUDRATE: u32 = 57600;
const SOLENOID_FREQUENCY: Hertz = khz(10);

// A disabled timer channel stops driving its pin, which would leave the
// driver transistor's base floating between beeps/unlocks; hold it low.
const PWM_PIN_CONFIG: PwmPinConfig = PwmPinConfig {
    output_type: OutputType::PushPull,
    speed: Speed::Low,
    pull: Pull::Down,
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let button_pin1 = Output::new(p.PA3, Level::High, Speed::Low);

    let mut fingerprint_uart_config = embassy_stm32::usart::Config::default();
    fingerprint_uart_config.baudrate = SENSOR_BAUDRATE;

    let fingerprint_uart = Uart::new(
        p.USART1,
        p.PB7,
        p.PB6,
        p.DMA1_CH1,
        p.DMA1_CH2,
        Irqs,
        fingerprint_uart_config,
    )
    .unwrap();

    let sensor = FingerprintSensor::new(
        button_pin1,
        SENSOR_ADDRESS,
        SENSOR_PASSWORD,
        fingerprint_uart,
    );

    // WAKE is driven both ways by the sensor; an internal pull-up fights its
    // weak low and leaves it at ~0.8 V, right at the input threshold.
    let fingerprint_wake_pin = ExtiInput::new(p.PA2, p.EXTI2, Pull::None, Irqs);
    spawner.spawn(fingerprint_manager_task(sensor, fingerprint_wake_pin).unwrap());
    let solenoid_pin = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new_with_config(p.PA8, PWM_PIN_CONFIG)), // PA8 is Channel 1
        None,
        None,
        None,
        SOLENOID_FREQUENCY,
        Default::default(),
    );

    spawner.spawn(unlock_task(solenoid_pin).unwrap());

    let adc = Adc::new(p.ADC1);
    let beeper_pwm = SimplePwm::new(
        p.TIM14,
        Some(PwmPin::new_with_config(p.PA7, PWM_PIN_CONFIG)),
        None,
        None,
        None,
        khz(2),
        Default::default(),
    );

    spawner.spawn(beeper_task(beeper_pwm).unwrap());

    let shutdown_pin = Output::new(p.PA5, Level::Low, Speed::Low);
    spawner.spawn(battery_monitor_task(adc, p.PA4, shutdown_pin).unwrap());

    // Park forever without occupying a timer-queue slot.
    core::future::pending::<()>().await;
}
