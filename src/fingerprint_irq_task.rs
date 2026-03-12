use defmt::info;
use embassy_stm32 as _;
use embassy_stm32 as _;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::PB2;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_sync::watch::Watch;
use embassy_time::Timer;

use crate::fingerprint_task::{CommandEnvelope, FINGERPRINT_CHANNEL, SensorCommand};

pub static FINGERPRINT_IRQ_STATUS: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();

#[embassy_executor::task]
pub async fn fingerprint_irq_task(mut pin: ExtiInput<'static, PB2>) {
    let sender = FINGERPRINT_IRQ_STATUS.sender();
    static DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    loop {
        pin.wait_for_any_edge().await;
        let f_sample = pin.is_low();
        Timer::after_millis(100).await;
        let s_sample = pin.is_low();
        if f_sample == s_sample {
            sender.send(s_sample);
            let _ = FINGERPRINT_CHANNEL.send(CommandEnvelope {
                cmd: SensorCommand::ValidateAccess,
                ending_signal: &DONE,
            });
        }
    }
}
