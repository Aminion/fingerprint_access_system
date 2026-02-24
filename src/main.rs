#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::Config;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());
    let mut pwm = SimplePwm::new(
        p.TIM3,
        Some(PwmPin::new_ch1(
            p.PC6,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        None,
        None,
        None,
        hz(10000),
        Default::default(),
    );

    let max_duty = pwm.get_max_duty();
    pwm.enable(embassy_stm32::timer::Channel::Ch1);

    loop {
        for i in 0..=100 {
            let duty = i * (max_duty / 100);
            pwm.set_duty(embassy_stm32::timer::Channel::Ch1, duty);
            Timer::after_millis(2).await;
        }

        for i in (0..=100).rev() {
            let duty = i * (max_duty / 100);
            pwm.set_duty(embassy_stm32::timer::Channel::Ch1, duty);
            Timer::after_millis(2).await;
        }
    }
}
