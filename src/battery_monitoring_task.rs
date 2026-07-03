use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::{ADC1, PA4, PA5};
use embassy_stm32::Peri;
use embassy_time::{Duration, Timer};

use crate::beeper_task::{BeeperCommand, BEEPER_CHANNEL};

//NI-MH
const ADC_RANGE: f64 = 4096.0;
const ADC_MAX_V: f64 = 3.3;
const REF_V: f64 = 2.495;
const BAT_COUNT: f64 = 4.0;
const V_FULL: f64 = BAT_COUNT * 1.35;
const V_LOW: f64 = BAT_COUNT * 1.1;
const V_CRITICAL: f64 = BAT_COUNT * 1.0;
const V_LOW_LEVEL: u16 = (V_LOW / V_FULL * ADC_RANGE) as u16;
const V_CRITICAL_LEVEL: u16 = (V_CRITICAL / V_FULL * ADC_RANGE) as u16;
const REF_LEVEL: u16 = (REF_V / ADC_MAX_V * ADC_RANGE) as u16;

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

const DELAY: Duration = Duration::from_secs(3600); // 1 hour

#[embassy_executor::task]
pub async fn battery_monitor_task(
    mut adc: Adc<'static, ADC1>,
    mut measure_pin: Peri<'static, PA5>,
    mut reference_enable_pin: Output<'static>,
    mut reference_pin: Peri<'static, PA4>,
) {
    loop {
        reference_enable_pin.set_high();

        let ref_sample = adc.blocking_read(&mut reference_pin, SampleTime::CYCLES160_5);
        let bat_sample = adc.blocking_read(&mut measure_pin, SampleTime::CYCLES160_5);

        reference_enable_pin.set_low();

        // Ratiometric correction: scale battery reading by how far the reference
        // deviates from its expected level. Handles both ADC over- and under-read.
        let bat_sample_corrected = if ref_sample > 0 {
            (bat_sample as u32 * REF_LEVEL as u32 / ref_sample as u32) as u16
        } else {
            bat_sample
        };
        if bat_sample_corrected <= V_CRITICAL_LEVEL {
            BEEPER_CHANNEL.send(LOW_LEVEL_SIGNAL).await;
        } else if bat_sample_corrected <= V_LOW_LEVEL {
            BEEPER_CHANNEL.send(CRITICAL_LEVEL_SIGNAL).await;
        }
        Timer::after(DELAY).await;
    }
}
