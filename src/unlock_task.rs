use embassy_stm32 as _;
use embassy_stm32::peripherals::TIM1;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

/// Signaled by the fingerprint manager on a match.
pub static UNLOCK_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
pub async fn unlock_task(mut pwm: SimplePwm<'static, TIM1>) {
    loop {
        UNLOCK_SIGNAL.wait().await;
        unlock(&mut pwm).await;
    }
}

pub async fn unlock(pwm: &mut SimplePwm<'static, TIM1>) {
    const KICK: Duration = Duration::from_millis(200);
    const HOLD: Duration = Duration::from_millis(3000);

    pwm.ch1().enable();
    pwm.ch1().set_duty_cycle_fully_on();
    Timer::after(KICK).await;
    pwm.ch1().set_duty_cycle_fraction(4, 4);
    Timer::after(HOLD).await;
    pwm.ch1().set_duty_cycle_fully_off();
    pwm.ch1().disable();
}
