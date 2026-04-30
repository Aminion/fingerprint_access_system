use embassy_stm32 as _;
use embassy_stm32 as _;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::PA2;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Timer;


pub static FINGERPRINT_IRQ_STATUS: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();

#[embassy_executor::task]
pub async fn fingerprint_irq_task(mut pin: ExtiInput<'static, PA2>) {
    let sender = FINGERPRINT_IRQ_STATUS.sender();
    loop {
        pin.wait_for_any_edge().await;
        let f_sample = pin.is_low();
        Timer::after_millis(100).await;
        let s_sample = pin.is_low();
        if f_sample == s_sample {
            sender.send(s_sample);
        }
    }
}
