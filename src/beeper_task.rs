use embassy_stm32::{gpio::Output, peripherals::PB9};
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
    pub done: &'static Signal<CriticalSectionRawMutex, ()>,
}

#[embassy_executor::task]
pub async fn beeper_task(mut beeper_pin: Output<'static, PB9>) {
    loop {
        let cmd = BEEPER_CHANNEL.receive().await;
        for _ in 0..cmd.times {
            beeper_pin.set_high();
            Timer::after(cmd.duration).await;
            beeper_pin.set_low();
            if cmd.times > 0 {
                Timer::after(cmd.delay).await;
            }
        }
        cmd.done.signal(());
    }
}
