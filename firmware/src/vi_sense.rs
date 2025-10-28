// todo: clean up includes
use esp_hal::analog::adc::*;
use esp_hal::peripherals::{ADC1, GPIO0, GPIO1, GPIO4};
use esp_hal::Blocking;

type Adc1CalCurve = AdcCalCurve<ADC1<'static>>;

pub struct VISense {
    pub v_sense: AdcPin<GPIO1<'static>, ADC1<'static>, Adc1CalCurve>,
    pub i_sense: AdcPin<GPIO0<'static>, ADC1<'static>, Adc1CalCurve>,
    pub input_i_sense: AdcPin<GPIO4<'static>, ADC1<'static>, Adc1CalCurve>,
    pub adc: Adc<'static, ADC1<'static>, Blocking>,

    pub input_current_ma: u32,
    pub output_current_ma: u32,
    pub battery_voltage_mv: u32,
}

impl VISense {
    pub fn run(&mut self) {
        // since ADC channels have been calibrated, the HAL returns readings in mV
        let mut v_mv: u32 = 0;
        let mut i_mv: u32 = 0;
        let mut input_i_mv: u32 = 0;

        let num_samples = 50;
        for _ in 0..num_samples {
            v_mv += nb::block!(self.adc.read_oneshot(&mut self.v_sense)).unwrap() as u32;
            i_mv += nb::block!(self.adc.read_oneshot(&mut self.i_sense)).unwrap() as u32;
            input_i_mv +=
                nb::block!(self.adc.read_oneshot(&mut self.input_i_sense)).unwrap() as u32;
        }

        v_mv /= num_samples;
        i_mv /= num_samples;
        input_i_mv /= num_samples;

        self.input_current_ma = input_i_mv * 2; // 0.5mV per mA
        self.output_current_ma = i_mv; // 1 mV per mA
        self.battery_voltage_mv = v_mv * 20;
    }

    pub fn input_current_ma(&self) -> u32 {
        self.input_current_ma
    }

    pub fn output_current_ma(&self) -> u32 {
        self.output_current_ma
    }

    pub fn battery_voltage_mv(&self) -> u32 {
        self.battery_voltage_mv
    }
}
