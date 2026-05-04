#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]

use core::u64;
use defmt::println;
use embassy_executor::Spawner;
use embassy_stm32 as _;
use embassy_stm32 as _;
use embassy_stm32::adc::Adc;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2, PB6, PB7, USART1};
use embassy_stm32::time::{khz, Hertz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{InterruptHandler, Uart};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use crate::fingerprint_irq_task::FINGERPRINT_IRQ_STATUS;
use crate::fingerprint_sensor::Irqs;
use crate::fingerprint_task::add_new_finger_task;
use crate::unlock_task::unlock;
mod battery_monitoring_task;
mod beeper_task;
mod fingerprint_irq_task;
mod fingerprint_sensor;
mod fingerprint_task;
mod unlock_task;

use battery_monitoring_task::battery_monitor_task;
use beeper_task::beeper_task;
use fingerprint_irq_task::fingerprint_irq_task;
use fingerprint_task::fingerprint_manager_task;
use unlock_task::unlock_task;

pub type FingerprintSensor =
    fingerprint_sensor::FingerprintSensor<'static, USART1, DMA1_CH1, DMA1_CH2>;

const SENSOR_ADDRESS: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFF];
const SENSOR_PASSWORD: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
const SENSOR_BAUDRATE: u32 = 57600;
const SOLENOID_FREQUENCY: Hertz = khz(10);

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
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        fingerprint_uart_config,
    )
    .unwrap();

    let sensor = FingerprintSensor::new::<PB6, PB7>(
        button_pin1,
        SENSOR_ADDRESS,
        SENSOR_PASSWORD,
        fingerprint_uart,
    )
    .await;

    FINGERPRINT_IRQ_STATUS.sender().send(false);
    let button_pin = Input::new(p.PA2, Pull::Up);
    let fingerprint_irq_pin = ExtiInput::new(button_pin, p.EXTI2);
    spawner.spawn(fingerprint_manager_task(sensor)).unwrap();
    spawner
        .spawn(fingerprint_irq_task(fingerprint_irq_pin))
        .unwrap();
    let solenoid_pin = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new_ch1(p.PA8, OutputType::PushPull)), // PA8 is Channel 1
        None,
        None,
        None,
        SOLENOID_FREQUENCY,
        Default::default(),
    );

    spawner.spawn(unlock_task(solenoid_pin)).unwrap();
    loop {
        Timer::after_secs(u32::MAX as u64).await;
    }

    //let button_pin = Input::new(p.PB8, Pull::Up);
    //let add_finger_pin = ExtiInput::new(button_pin, p.EXTI8);

    let mut delay = embassy_time::Delay;
    let adc = Adc::new(p.ADC1, &mut delay);
    let ref_enable_pin = Output::new(p.PB3, Level::Low, Speed::Low);
    let beeper_pin = Output::new(p.PB9, Level::Low, Speed::Low);

    //spawner.spawn(add_new_finger_task(add_finger_pin)).unwrap();

    spawner.spawn(beeper_task(beeper_pin)).unwrap();
    spawner
        .spawn(battery_monitor_task(adc, p.PA5, ref_enable_pin, p.PA4))
        .unwrap();
}
