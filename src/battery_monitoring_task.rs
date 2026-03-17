use defmt::info;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::peripherals::{ADC1, PA4, PA5};
use embassy_time::{Duration, Timer};

//NI-MH
const ADC_RANGE: f64 = 4096.0;
const ADC_MAX_V: f64 = 3.3;
const BAT_COUNT: f64 = 4.0;
const V_FULL: f64 = BAT_COUNT * 1.35;
const V_LOW: f64 = BAT_COUNT * 1.1;
const V_CRITICAL: f64 = BAT_COUNT * 1.0;
const V_FULL_LEVEL: u16 = ADC_RANGE as u16;
const V_LOW_LEVEL: u16 = (V_LOW / V_FULL * ADC_RANGE) as u16;
const V_CRITICAL_LEVEL: u16 = (V_CRITICAL / V_FULL * ADC_RANGE) as u16;
const REF_LEVEL: u16 = ((2.493 / 3.3) * ADC_RANGE as f64) as u16;

#[embassy_executor::task]
pub async fn battery_monitor_task(
    mut adc: Adc<'static, ADC1>,
    mut measure_pin: PA5,   // Battery divider input
    mut reference_pin: PA4, // Battery divider input
) {
    // Configure ADC resolution and sample time
    adc.set_sample_time(SampleTime::Cycles160_5);

    loop {
        let r = adc.read(&mut reference_pin);
        let v_batt_raw = adc.read(&mut measure_pin);
        let div = REF_LEVEL - r;
        let v_at_pin = v_batt_raw + div;
        info!("{}",v_at_pin);
        if v_at_pin < V_CRITICAL_LEVEL {
            info!("CRITICAL BATTERY VOLTAGE")
        } else if v_at_pin < V_LOW_LEVEL {
            info!("LOW BATTERY VOLTAGE")
        } else if v_at_pin < V_FULL_LEVEL {
            info!("FULL BATTERY VOLTAGE")
        };

        Timer::after(Duration::from_secs(1)).await;
    }
}
