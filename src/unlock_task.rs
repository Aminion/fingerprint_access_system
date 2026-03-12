use core::u64;

use embassy_stm32 as _;
use embassy_stm32 as _;

use embassy_stm32::peripherals::TIM1;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;

use crate::{
    fingerprint_irq_task::FINGERPRINT_IRQ_STATUS,
    fingerprint_task::{CommandEnvelope, SensorCommand, FINGERPRINT_CHANNEL},
};

static DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
pub async fn unlock_task() {
    let mut receiver = FINGERPRINT_IRQ_STATUS.receiver().unwrap();
    loop {
        let _ = receiver.changed_and(|v| *v).await;
        FINGERPRINT_CHANNEL
            .send(CommandEnvelope {
                cmd: SensorCommand::ValidateAccess,
                ending_signal: &DONE,
            })
            .await;
        Timer::after_ticks(100).await;
        DONE.wait().await;
    }
}

async fn unlock(pwm: &mut SimplePwm<'static, TIM1>) {
    const KICK_MS: u64 = 50;
    const HOLD_MS: u64 = 3000;
    const HOLD_DUTY_DIVIDER: u16 = 3;
    const CHANNEL: embassy_stm32::timer::Channel = embassy_stm32::timer::Channel::Ch1;

    let max_duty = pwm.get_max_duty();
    pwm.enable(CHANNEL);
    pwm.set_duty(CHANNEL, max_duty);
    Timer::after_millis(KICK_MS).await;
    pwm.set_duty(CHANNEL, max_duty / HOLD_DUTY_DIVIDER);
    Timer::after_millis(HOLD_MS).await;
    pwm.set_duty(CHANNEL, 0);
    pwm.disable(CHANNEL);
}
