use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::{ADC1, PA4};
use embassy_stm32::Peri;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

use crate::beeper_task::{BeeperCommand, BEEPER_CHANNEL};

static CRITICAL_SHUTDOWN_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// NI-MH 4S
const ADC_RANGE: f64 = 4096.0;
const BAT_COUNT: f64 = 4.0;
const V_FULL: f64 = BAT_COUNT * 1.2;
const V_LOW: f64 = BAT_COUNT * 1.1;
const V_CRITICAL: f64 = BAT_COUNT * 1.0;
const V_LOW_LEVEL: u16 = (V_LOW / V_FULL * ADC_RANGE) as u16;
const V_CRITICAL_LEVEL: u16 = (V_CRITICAL / V_FULL * ADC_RANGE) as u16;

const LOW_LEVEL_SIGNAL: BeeperCommand = BeeperCommand {
    duration: embassy_time::Duration::from_millis(500),
    delay: embassy_time::Duration::from_millis(0),
    times: 1,
    done: None,
};

const CRITICAL_LEVEL_SIGNAL: BeeperCommand = BeeperCommand {
    duration: embassy_time::Duration::from_millis(500),
    delay: embassy_time::Duration::from_millis(100),
    times: 3,
    done: Some(&CRITICAL_SHUTDOWN_DONE),
};

const DELAY: Duration = Duration::from_secs(3600);

#[embassy_executor::task]
pub async fn battery_monitor_task(
    mut adc: Adc<'static, ADC1>,
    mut measure_pin: Peri<'static, PA4>,
    mut shutdown_pin: Output<'static>,
) {
    loop {
        let sample = adc.blocking_read(&mut measure_pin, SampleTime::CYCLES160_5);
   
        if sample <= V_CRITICAL_LEVEL {
            BEEPER_CHANNEL.send(CRITICAL_LEVEL_SIGNAL).await;
            CRITICAL_SHUTDOWN_DONE.wait().await;
            shutdown_pin.set_high();

        } else if sample <= V_LOW_LEVEL {
            BEEPER_CHANNEL.send(LOW_LEVEL_SIGNAL).await;
        }
         

        Timer::after(DELAY).await;
    }
}
