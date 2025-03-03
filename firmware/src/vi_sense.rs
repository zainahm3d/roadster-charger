// todo: clean up includes
use esp_hal::analog::adc::*;
use esp_hal::peripherals::ADC1;
use esp_hal::{gpio::*, Blocking};

struct ViData {
    input_current_ma: u32,
    output_current_ma: u32,
    battery_voltage_mv: u32,
}

static mut VI_DATA: ViData = ViData {
    input_current_ma: 0,
    output_current_ma: 0,
    battery_voltage_mv: 0,
};

pub fn run(
    adc1_instance: &mut Adc<ADC1, Blocking>,
    v_sense: &mut AdcPin<GpioPin<1>, ADC1, AdcCalCurve<ADC1>>,
    i_sense: &mut AdcPin<GpioPin<0>, ADC1, AdcCalCurve<ADC1>>,
    input_i_sense: &mut AdcPin<GpioPin<4>, ADC1, AdcCalCurve<ADC1>>,
) {
    // since ADC channels have been calibrated, the HAL returns readings in mV
    let mut v_mv: u32 = 0;
    let mut i_mv: u32 = 0;
    let mut input_i_mv: u32 = 0;

    let num_samples = 50;
    for _ in 0..num_samples {
        v_mv += adc1_instance.read_oneshot(v_sense).unwrap() as u32;
        i_mv += adc1_instance.read_oneshot(i_sense).unwrap() as u32;
        input_i_mv += adc1_instance.read_oneshot(input_i_sense).unwrap() as u32;
    }

    v_mv /= num_samples;
    i_mv /= num_samples;
    input_i_mv /= num_samples;

    // safety: this is the only place we write to VI_DATA
    unsafe {
        VI_DATA.input_current_ma = input_i_mv * 2; // 0.5mV per mA
        VI_DATA.output_current_ma = i_mv; // 1 mV per mA
        VI_DATA.battery_voltage_mv = v_mv * 16;
    }
}

pub fn input_current_ma() -> u32 {
    unsafe { VI_DATA.input_current_ma }
}

pub fn output_current_ma() -> u32 {
    unsafe { VI_DATA.output_current_ma }
}

pub fn battery_voltage_mv() -> u32 {
    unsafe { VI_DATA.battery_voltage_mv }
}
