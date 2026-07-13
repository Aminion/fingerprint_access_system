use defmt::info;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::peripherals::{ADC1, PA4};
use embassy_stm32::Peri;
use embassy_time::{Duration, Timer};

use crate::beeper_task::{BeeperCommand, BEEPER_CHANNEL};

// NI-MH 4S
const ADC_RANGE: f64 = 4096.0;
const BAT_COUNT: f64 = 4.0;
const V_FULL: f64 = BAT_COUNT * 1.35;
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
    done: None,
};

const DELAY: Duration = Duration::from_secs(3600);

#[embassy_executor::task]
pub async fn battery_monitor_task(mut adc: Adc<'static, ADC1>, mut measure_pin: Peri<'static, PA4>) {
    loop {
        let sample = adc.blocking_read(&mut measure_pin, SampleTime::CYCLES160_5);

        if sample <= V_CRITICAL_LEVEL {
            BEEPER_CHANNEL.send(CRITICAL_LEVEL_SIGNAL).await;
        } else if sample <= V_LOW_LEVEL {
            BEEPER_CHANNEL.send(LOW_LEVEL_SIGNAL).await;
        }

        Timer::after(DELAY).await;
    }
}
