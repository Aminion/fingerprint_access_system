use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{select, Either};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::Timer;

pub static FINGERPRINT_IRQ_STATUS: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();
/// Set true while a sensor UART command is in flight; the IRQ task ignores
/// WAKE pin HIGH transitions during this window to avoid scan-induced glitches.
pub static SENSOR_ACTIVE: AtomicBool = AtomicBool::new(false);
/// Pulsed after SENSOR_ACTIVE is cleared so the IRQ task re-reads the pin
/// and publishes the authoritative finger presence state.
pub static RESYNC_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
pub async fn fingerprint_irq_task(mut pin: ExtiInput<'static, Async>) {
    let sender = FINGERPRINT_IRQ_STATUS.sender();
    let mut state = false;
    loop {
        match select(pin.wait_for_any_edge(), RESYNC_SIGNAL.wait()).await {
            Either::First(_) => {
                if pin.is_low() && !state {
                    defmt::println!("finger on");
                    state = true;
                    sender.send(state);
                } else if pin.is_high() && !SENSOR_ACTIVE.load(Ordering::Relaxed) {
                    // Only treat a HIGH edge as finger-off when the sensor is idle.
                    // Re-check after debounce; also re-check the flag in case a new
                    // command started while we were waiting.
                    Timer::after_millis(50).await;
                    if pin.is_high() && !SENSOR_ACTIVE.load(Ordering::Relaxed) {
                        defmt::println!("finger off");
                        state = false;
                        sender.send(state);
                    }
                }
                // HIGH edge while sensor active → scan-induced glitch, ignore.
            }
            Either::Second(_) => {
                // Give the pin a moment to settle after a sensor command finishes
                // or the sensor is powered down.
                Timer::after_millis(10).await;
                let current = pin.is_low();
                defmt::println!("irq resync: finger={}", current);
                if current != state {
                    state = current;
                    sender.send(state);
                }
            }
        }
    }
}
