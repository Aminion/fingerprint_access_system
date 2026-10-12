use embassy_stm32::adc::{Adc, SampleTime, VREF_CALIB_MV};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::{ADC1, PA4};
use embassy_stm32::Peri;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

use crate::beeper_task::{BeeperCommand, BEEPER_CHANNEL};

static CRITICAL_SHUTDOWN_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// NI-MH 4S
const BAT_COUNT: u32 = 4;
const V_FULL_MV: u32 = BAT_COUNT * 1200;
const V_LOW_MV: u32 = BAT_COUNT * 1100;
const V_CRITICAL_MV: u32 = BAT_COUNT * 1000;

const ADC_MAX: u32 = 4095;
// RV1 is trimmed so that a full battery (V_FULL_MV) gives this voltage on BAT_SENSE.
const SENSE_AT_FULL_MV: u32 = 3300;

// VREF+ is bonded to VDD on LQFP32, so the ADC full scale follows the 3V3 rail.
// Measure VDDA against VREFINT (factory-calibrated at VDDA = 3.0 V) instead of assuming it.
fn battery_mv(sample: u16, vrefint_sample: u16, vrefint_cal: u16) -> u32 {
    let vdda_mv = VREF_CALIB_MV * vrefint_cal as u32 / (vrefint_sample as u32).max(1);
    let sense_mv = vdda_mv * sample as u32 / ADC_MAX;
    sense_mv * V_FULL_MV / SENSE_AT_FULL_MV
}

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
    // Buffer stays on (12.5 µA typ.); embassy has no API to disable it again.
    let mut vrefint = adc.enable_vrefint();
    let vrefint_cal = vrefint.calibrated_value();

    loop {
        // CYCLES160_5 also covers VREFINT's 4 µs minimum sampling time.
        let vrefint_sample = adc.blocking_read(&mut vrefint, SampleTime::CYCLES160_5);
        let sample = adc.blocking_read(&mut measure_pin, SampleTime::CYCLES160_5);
        let bat_mv = battery_mv(sample, vrefint_sample, vrefint_cal);

        if bat_mv <= V_CRITICAL_MV {
            BEEPER_CHANNEL.send(CRITICAL_LEVEL_SIGNAL).await;
            CRITICAL_SHUTDOWN_DONE.wait().await;
            shutdown_pin.set_high();

        } else if bat_mv <= V_LOW_MV {
            BEEPER_CHANNEL.send(LOW_LEVEL_SIGNAL).await;
        }
         

        Timer::after(DELAY).await;
    }
}
