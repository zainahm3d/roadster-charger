#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::adc::*,
    delay::Delay,
    gpio::{self, GpioPin, InputConfig, Level, Output, OutputConfig},
    i2c,
    peripherals::ADC1,
    rmt::TxChannelCreator,
    timer::{timg::TimerGroup, PeriodicTimer},
    Blocking,
};

mod boost;
mod charger;
mod led;
mod tcpc;
mod temp_sense;
mod usb_pd;
mod vi_sense;

#[rtic::app(device = esp32c3, dispatchers=[FROM_CPU_INTR0, FROM_CPU_INTR1])]
mod app {
    use esp_hal::{
        gpio::Input,
        rmt::TxChannelConfig,
        time::{Duration, Rate},
    };

    use super::*;

    #[shared]
    struct Shared {
        i2c: i2c::master::I2c<'static, Blocking>,
    }

    #[local]
    struct Local {
        fusb_int: gpio::Input<'static>,
        periodic: PeriodicTimer<'static, Blocking>,
        boost_enable: gpio::Output<'static>,
        v_sense: AdcPin<GpioPin<1>, ADC1, AdcCalCurve<ADC1>>,
        i_sense: AdcPin<GpioPin<0>, ADC1, AdcCalCurve<ADC1>>,
        input_i_sense: AdcPin<GpioPin<4>, ADC1, AdcCalCurve<ADC1>>,
        adc1: Adc<'static, ADC1, Blocking>,
    }

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut delay = Delay::new();

        let i2c_config = i2c::master::Config::default().with_frequency(Rate::from_khz(100));
        let mut i2c = i2c::master::I2c::new(peripherals.I2C0, i2c_config)
            .unwrap()
            .with_scl(peripherals.GPIO10) // TODO: check if SCL and SDA are swapped!
            .with_sda(peripherals.GPIO8);

        let rmt = esp_hal::rmt::Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();

        let mut rgb = rmt
            .channel0
            .configure(
                peripherals.GPIO3,
                TxChannelConfig::default()
                    .with_clk_divider(2)
                    .with_idle_output_level(Level::Low)
                    .with_idle_output(true)
                    .with_carrier_modulation(false)
                    .with_carrier_high(1)
                    .with_carrier_low(1)
                    .with_carrier_level(Level::Low),
            )
            .unwrap();

        // voltage and current sense // todo: cleanup / move into another file?
        let mut adc1_config = AdcConfig::new();
        let v_sense = adc1_config.enable_pin_with_cal(peripherals.GPIO1, Attenuation::_11dB);
        let i_sense = adc1_config.enable_pin_with_cal(peripherals.GPIO0, Attenuation::_11dB);
        let input_i_sense = adc1_config.enable_pin_with_cal(peripherals.GPIO4, Attenuation::_11dB);
        let adc1 = Adc::new(peripherals.ADC1, adc1_config);

        let mut boost_enable = Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default());
        let mut fusb_int = Input::new(peripherals.GPIO7, InputConfig::default());

        boost::init(&mut i2c, &mut boost_enable);
        tcpc::init(&mut i2c, &mut delay);

        // set PD LED to cyan if we have a contract, red if failed
        if tcpc::establish_pd_contract(&mut i2c, &mut fusb_int, &mut delay) {
            rgb = led::set_pixel(rgb, 0, 0, 20, 5);
            _ = led::set_pixel(rgb, 1, 0, 20, 5);
        } else {
            rgb = led::set_pixel(rgb, 0, 20, 0, 0);
            _ = led::set_pixel(rgb, 1, 20, 0, 0);
            panic!("failed to establish PD contract");
        }

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let mut periodic = PeriodicTimer::new(timg0.timer0);
        periodic.enable_interrupt(true);
        periodic.start(Duration::from_millis(100)).unwrap();

        (
            Shared { i2c },
            Local {
                fusb_int,
                periodic,
                boost_enable,
                v_sense,
                i_sense,
                input_i_sense,
                adc1,
            },
        )
    }

    #[task(binds=GPIO, local=[fusb_int], shared=[i2c], priority=1)]
    fn gpio_handler(mut cx: gpio_handler::Context) {
        if cx.local.fusb_int.is_low() {
            cx.local.fusb_int.clear_interrupt();
            cx.shared.i2c.lock(|i2c| {
                tcpc::write_reg(i2c, tcpc::Register::AlertL, &0xFF);
            });
        }
    }

    // Update control values and read back sensors
    #[ task(binds=TG0_T0_LEVEL,
        local=[periodic, boost_enable, adc1, v_sense, i_sense, input_i_sense],
        shared=[i2c], priority=2) ]
    fn timer_handler(mut cx: timer_handler::Context) {
        let loc = cx.local;
        loc.periodic.clear_interrupt();

        vi_sense::run(loc.adc1, loc.v_sense, loc.i_sense, loc.input_i_sense);
        boost::run();

        cx.shared.i2c.lock(|i2c| {
            charger::run(i2c, loc.boost_enable);
        });
    }
}
