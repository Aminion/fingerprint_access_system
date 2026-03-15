#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]

use core::u64;
use embassy_executor::Spawner;
use embassy_stm32 as _;
use embassy_stm32 as _;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::low_level::Pin;
use embassy_stm32::gpio::{Input, OutputType, Pull};
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2, USART1};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{InterruptHandler, Uart};
use embassy_time::Timer;

use crate::fingerprint_irq_task::FINGERPRINT_IRQ_STATUS;
use crate::fingerprint_task::add_new_finger_task;
use crate::unlock_task::unlock;
mod fingerprint_irq_task;
mod fingerprint_sensor;
mod fingerprint_task;
mod unlock_task;

use fingerprint_irq_task::fingerprint_irq_task;
use fingerprint_task::fingerprint_manager_task;
use unlock_task::unlock_task;

use {defmt_rtt as _, panic_probe as _};
pub type MySensor = fingerprint_sensor::FingerprintSensor<'static, USART1, DMA1_CH1, DMA1_CH2>;

const SENSOR_ADDRESS: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFF];
const SENSOR_PASSWORD: [u8; 4] = [0x00, 0x00, 0x00, 0x00];

bind_interrupts!(struct Irqs {
    USART1 => InterruptHandler<embassy_stm32::peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    Timer::after_millis(200).await;
    FINGERPRINT_IRQ_STATUS.sender().send(false);
    let button_pin = Input::new(p.PB2, Pull::Up);
    let fingerprint_irq_pin = ExtiInput::new(button_pin, p.EXTI2);
    //let button_pin = Input::new(p.PB8, Pull::Up);
    //let add_finger_pin = ExtiInput::new(button_pin, p.EXTI8);
    let pwm = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new_ch1(p.PA8, OutputType::PushPull)), // PA8 is Channel 1
        None,
        None,
        None,
        hz(1000),
        Default::default(),
    );
    //unlock(&mut ).await;
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
        fingerprint_sensor::FingerprintSensor::new(uart, SENSOR_ADDRESS, SENSOR_PASSWORD);
    let _ = sensor.verify_password().await;

    spawner
        .spawn(fingerprint_irq_task(fingerprint_irq_pin))
        .unwrap();
    spawner.spawn(fingerprint_manager_task(sensor)).unwrap();
    //spawner.spawn(add_new_finger_task(add_finger_pin)).unwrap();
    spawner.spawn(unlock_task(pwm)).unwrap();

    loop {
        Timer::after_secs(u32::MAX as u64).await;
    }
}
