use embassy_stm32::peripherals::TIM14;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};
use embassy_time::{Duration, Timer};

pub static BEEPER_CHANNEL: Channel<CriticalSectionRawMutex, BeeperCommand, 1> = Channel::new();

#[derive(Clone, Copy)]
pub struct BeeperCommand {
    pub duration: Duration,
    pub delay: Duration,
    pub times: u8,
    pub done: Option<&'static Signal<CriticalSectionRawMutex, ()>>,
}

#[embassy_executor::task]
pub async fn beeper_task(mut pwm: SimplePwm<'static, TIM14>) {
    pwm.ch1().set_duty_cycle_percent(50);
    loop {
        let cmd = BEEPER_CHANNEL.receive().await;
        for i in 0..cmd.times {
            pwm.ch1().enable();
            Timer::after(cmd.duration).await;
            pwm.ch1().disable();
            if i + 1 < cmd.times {
                Timer::after(cmd.delay).await;
            }
        }
        if let Some(done) = cmd.done {
            done.signal(());
        }
    }
}
