use embassy_stm32::adc::{Adc, SampleTime, VREF_CALIB_MV};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::{ADC1, PA4};
use embassy_stm32::Peri;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

use crate::beeper_task::{BeeperCommand, BEEPER_CHANNEL};
use crate::request::request;

static CRITICAL_SHUTDOWN_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// NI-MH 4S
const BAT_COUNT: u32 = 4;
const V_LOW_MV: u32 = BAT_COUNT * 1100;
const V_CRITICAL_MV: u32 = BAT_COUNT * 1000;

const ADC_MAX: u32 = 4095;
// RV1 is trimmed so that a battery at V_CAL_MV gives SENSE_AT_CAL_MV on BAT_SENSE.
const V_CAL_MV: u32 = 5500;
const SENSE_AT_CAL_MV: u32 = 3200;

// Readings per measurement. Both channels are summed over the same count, so the
// ratio of sums equals the ratio of averages and no averaging division is needed.
const SAMPLES: u32 = 16;

// VREF+ is bonded to VDD on LQFP32, so the ADC full scale follows the 3V3 rail.
// Measure VDDA against VREFINT (factory-calibrated at VDDA = 3.0 V) instead of assuming it.
// Computed in one u64 division so intermediate results are not truncated.
fn battery_mv(sample_sum: u32, vrefint_sum: u32, vrefint_cal: u16) -> u32 {
    let num = VREF_CALIB_MV as u64 * vrefint_cal as u64 * sample_sum as u64 * V_CAL_MV as u64;
    let den = (vrefint_sum as u64).max(1) * ADC_MAX as u64 * SENSE_AT_CAL_MV as u64;
    (num / den) as u32
}

const LOW_LEVEL_SIGNAL: BeeperCommand = BeeperCommand {
    duration: Duration::from_millis(500),
    delay: Duration::from_millis(0),
    times: 1,
    done: None,
};

const CRITICAL_LEVEL_SIGNAL: BeeperCommand = BeeperCommand {
    duration: Duration::from_millis(500),
    delay: Duration::from_millis(100),
    times: 3,
    done: None,
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
        // Interleaved so slow VDDA drift affects both sums equally.
        // CYCLES160_5 also covers VREFINT's 4 µs minimum sampling time.
        let mut vrefint_sum = 0u32;
        let mut sample_sum = 0u32;
        for _ in 0..SAMPLES {
            vrefint_sum += adc.blocking_read(&mut vrefint, SampleTime::CYCLES160_5) as u32;
            sample_sum += adc.blocking_read(&mut measure_pin, SampleTime::CYCLES160_5) as u32;
        }

        let bat_mv = battery_mv(sample_sum, vrefint_sum, vrefint_cal);

        if bat_mv <= V_CRITICAL_MV {
            // Cut power only after the last beep has finished.
            request(&BEEPER_CHANNEL, &CRITICAL_SHUTDOWN_DONE, |done| BeeperCommand {
                done: Some(done),
                ..CRITICAL_LEVEL_SIGNAL
            })
            .await;
            shutdown_pin.set_high();
        } else if bat_mv <= V_LOW_MV {
            BEEPER_CHANNEL.send(LOW_LEVEL_SIGNAL).await;
        }
        Timer::after(DELAY).await;
    }
}
