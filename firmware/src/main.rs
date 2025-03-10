#![no_std]
#![no_main]

use core::sync::atomic::Ordering;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::*,
    delay::Delay,
    gpio::{self, Input, InputConfig, Level, Output, OutputConfig},
    i2c,
    time::{Duration, Rate},
    timer::{timg::TimerGroup, PeriodicTimer},
    Blocking,
};

use crate::led::color;
mod boost;
mod charger;
mod led;
mod tcpc;
mod temp_sense;
mod usb_pd;
mod vi_sense;

#[rtic::app(device = esp32c3, dispatchers=[FROM_CPU_INTR0, FROM_CPU_INTR1])]
mod app {
    use crate::vi_sense::VISense;

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
        vi_sense: VISense,
        state: charger::State,
        leds: led::Led,
    }

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut delay = Delay::new();

        let i2c_config = i2c::master::Config::default().with_frequency(Rate::from_khz(100));
        let mut i2c = i2c::master::I2c::new(peripherals.I2C0, i2c_config)
            .unwrap()
            .with_scl(peripherals.GPIO8)
            .with_sda(peripherals.GPIO10);

        let rmt = esp_hal::rmt::Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
        let mut leds = led::Led {
            pixel_buffer: [led::Rgb { r: 0, g: 0, b: 0 }; 2],
            rmt_channel: Some(led::Led::configure_rmt(rmt, peripherals.GPIO3)),
            last_update_time: esp_hal::time::Instant::EPOCH,
        };

        let mut adc1_config = AdcConfig::new();
        let vi_sense = VISense {
            v_sense: adc1_config.enable_pin_with_cal(peripherals.GPIO1, Attenuation::_11dB),
            i_sense: adc1_config.enable_pin_with_cal(peripherals.GPIO0, Attenuation::_11dB),
            input_i_sense: adc1_config.enable_pin_with_cal(peripherals.GPIO4, Attenuation::_11dB),
            adc: Adc::new(peripherals.ADC1, adc1_config),
            input_current_ma: 0,
            output_current_ma: 0,
            battery_voltage_mv: 0,
        };

        let mut boost_enable = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
        let mut fusb_int = Input::new(peripherals.GPIO7, InputConfig::default());

        boost::init(&mut i2c, &mut boost_enable);
        tcpc::init(&mut i2c, &mut delay);

        if tcpc::establish_pd_contract(&mut i2c, &mut fusb_int, &mut delay) {
            leds.set_pixel(0, color::GREEN);
        } else {
            leds.set_pixel(0, color::RED);
            panic!("failed to establish PD contract");
        }
        leds.set_pixel(1, color::RED);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let mut periodic = PeriodicTimer::new(timg0.timer0);
        periodic.enable_interrupt(true);
        periodic.start(Duration::from_millis(100)).unwrap();

        let mut state = charger::State::default();
        state.pdo_mv = tcpc::PDO_VOLTAGE_MV.load(Ordering::Relaxed);
        state.pdo_ma = tcpc::PDO_CURRENT_MA.load(Ordering::Relaxed);

        (
            Shared { i2c },
            Local {
                fusb_int,
                periodic,
                boost_enable,
                vi_sense,
                state,
                leds,
            },
        )
    }

    #[task(binds=GPIO, local=[fusb_int], shared=[i2c], priority=1)]
    fn gpio_handler(mut c: gpio_handler::Context) {
        if c.local.fusb_int.is_low() {
            c.local.fusb_int.clear_interrupt();
            c.shared.i2c.lock(|i2c| {
                tcpc::write_reg(i2c, tcpc::Register::AlertL, &0xFF);
            });
        }
    }

    // Update control values and read back sensors
    #[ task(binds=TG0_T0_LEVEL,
        local=[periodic, boost_enable, vi_sense, state, leds],
        shared=[i2c], priority=2) ]
    fn timer_handler(mut c: timer_handler::Context) {
        c.local.periodic.clear_interrupt();
        c.local.vi_sense.run();
        c.shared.i2c.lock(|i2c| {
            charger::run(
                i2c,
                c.local.boost_enable,
                c.local.state,
                c.local.vi_sense,
                c.local.leds,
            );
        });
    }
}
