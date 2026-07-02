use core::u64;

use embassy_stm32 as _;
use embassy_stm32::peripherals::TIM1;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;

use crate::{
    fingerprint_irq_task::FINGERPRINT_IRQ_STATUS,
    fingerprint_task::{SensorCommand, FINGERPRINT_CHANNEL},
};

static DONE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

#[embassy_executor::task]
pub async fn unlock_task(mut pwm: SimplePwm<'static, TIM1>) {
    let mut receiver = FINGERPRINT_IRQ_STATUS.receiver().unwrap();
    loop {
        receiver.changed_and(|v| *v).await;
        FINGERPRINT_CHANNEL
            .send(SensorCommand::ValidateAccess(&DONE))
            .await;
        if DONE.wait().await {
            unlock(&mut pwm).await;
        }
    }
}

pub async fn unlock(pwm: &mut SimplePwm<'static, TIM1>) {
    const KICK_MS: u64 = 200;
    const HOLD_MS: u64 = 3000;
    const HOLD_DUTY_MULTIPLIER: f32 = 0.75;

    let max_duty = pwm.ch1().max_duty_cycle();
    pwm.ch1().enable();
    pwm.ch1().set_duty_cycle(max_duty);
    Timer::after_millis(KICK_MS).await;
    pwm.ch1().set_duty_cycle((max_duty as f32 * HOLD_DUTY_MULTIPLIER) as u32);
    Timer::after_millis(HOLD_MS).await;
    pwm.ch1().set_duty_cycle(0);
    pwm.ch1().disable();
}
