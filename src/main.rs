#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_stm32 as _;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::CountingMode;
use embassy_stm32::usart::{InterruptHandler, Uart, UartRx};
use embassy_stm32::{bind_interrupts, Config};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32 as _;
use embassy_stm32::peripherals::{DMA1_CH2, TIM1, USART2}; // Make sure to import this!

mod fingerprint_sensor;

#[embassy_executor::task]
async fn serial_listener_task(
    mut rx: UartRx<'static, USART2, DMA1_CH2>,
    mut pwm: SimplePwm<'static, TIM1>,
) {
    let mut cmdBuff = [0u8; 16];
    let mut curr = 0;

    loop {
        let mut buf = [0u8; 1];
        match rx.read(&mut buf).await {
            Ok(_) => {
                let received = buf[0];
                match received {
                    b'\n' | b'\r' if curr > 0 => {
                        match core::str::from_utf8(&cmdBuff[..curr]) {
                            Ok(command) => {
                                let mut lines = command.split_whitespace();
                                match lines.next() {
                                    Some("run") => {
                                        unlock(&mut pwm).await;
                                    }
                                    _ => {}
                                }
                            }
                            Err(_) => {
                                info!("UTF-8 Error");
                            }
                        }
                        curr = 0;
                    }
                    _ if curr < cmdBuff.len() => {
                        cmdBuff[curr] = received;
                        curr += 1;
                    }
                    _ => {
                        warn!("Buffer Overflow");
                        curr = 0;
                    }
                }
            }
            Err(e) => error!("UART Read Error: {:?}", e),
        }
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
    let p = embassy_stm32::init(Default::default());

    // 2. Setup PWM for the Solenoid (PA8 / D9)
    let mut pwm = SimplePwm::new(
        p.TIM1,                                             // Timer 1
        Some(PwmPin::new_ch1(p.PA8, OutputType::PushPull)), // PA8 is D9
        None,
        None,
        None,
        hz(10000),
        Default::default(),
    );

    let mut uart_config = embassy_stm32::usart::Config::default();
    uart_config.baudrate = 57_600;
    // 1. Setup UART (Do NOT split yet)
    let mut uart = Uart::new(
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

    loop {
        info!("Password verification");
        let r = s.verify_password().await;
        info!("Password verification result: {:?}", r);
        Timer::after_millis(1000).await;
    }
}
