use defmt::info;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::{ADC1, PA4, PA5, PB3};
use embassy_time::Timer;

//NI-MH
const ADC_RANGE: f64 = 4096.0;
const ADC_MAX_V: f64 = 3.3;
const REF_V: f64 = 2.495;
const BAT_COUNT: f64 = 4.0;
const V_FULL: f64 = BAT_COUNT * 1.35;
const V_LOW: f64 = BAT_COUNT * 1.1;
const V_CRITICAL: f64 = BAT_COUNT * 1.0;
const V_FULL_LEVEL: u16 = ADC_RANGE as u16;
const V_LOW_LEVEL: u16 = (V_LOW / V_FULL * ADC_RANGE) as u16;
const V_CRITICAL_LEVEL: u16 = (V_CRITICAL / V_FULL * ADC_RANGE) as u16;
const REF_LEVEL: u16 = (REF_V / ADC_MAX_V * ADC_RANGE) as u16;

const DELAY: u64 = 3600; //1 hour

#[embassy_executor::task]
pub async fn battery_monitor_task(
    mut adc: Adc<'static, ADC1>,
    mut measure_pin: PA5,
    mut reference_enable_pin: Output<'static, PB3>,
    mut reference_pin: PA4,
) {
    adc.set_sample_time(SampleTime::Cycles160_5);
    loop {
        reference_enable_pin.set_high();
        
        let ref_sample = adc.read(&mut reference_pin);
        let bat_sample = adc.read(&mut measure_pin);
        
        reference_enable_pin.set_low();
        
        let discrepancy = REF_LEVEL - ref_sample;
        let bat_sample_corrected = bat_sample + discrepancy;
        if bat_sample_corrected < V_CRITICAL_LEVEL {
            info!("CRITICAL BATTERY VOLTAGE")
        } else if bat_sample_corrected < V_LOW_LEVEL {
            info!("LOW BATTERY VOLTAGE")
        } else if bat_sample_corrected < V_FULL_LEVEL {
            info!("FULL BATTERY VOLTAGE")
        };
        Timer::after_secs(DELAY).await;
    }
}
