use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Timer;

pub static FINGERPRINT_IRQ_STATUS: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();

#[embassy_executor::task]
pub async fn fingerprint_irq_task(mut pin: ExtiInput<'static, Async>) {
    let sender = FINGERPRINT_IRQ_STATUS.sender();
    let mut state = false;
    loop {
        pin.wait_for_any_edge().await;
        if pin.is_low() && !state {
            defmt::println!("on");
            state = true;
            sender.send(state);
        } else {
            defmt::println!("off");
            Timer::after_millis(200).await;
            if pin.is_high() {
                defmt::println!("really off");
                state = false;
                sender.send(state);
            }
        }
    }
}
