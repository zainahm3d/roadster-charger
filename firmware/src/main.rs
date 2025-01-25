#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Attenuation::*, *},
    clock::{ClockControl, CpuClock},
    delay::Delay,
    gpio::{self, GpioPin, Io},
    i2c::I2C,
    peripherals::{Peripherals, ADC1, I2C0, TIMG0},
    prelude::*,
    rmt::TxChannelCreator,
    system::SystemControl,
    timer::{
        timg::{Timer, TimerGroup, TimerX},
        PeriodicTimer,
    },
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
    use super::*;

    #[shared]
    struct Shared {
        i2c: esp_hal::i2c::I2C<'static, I2C0, Blocking>,
    }

    #[local]
    struct Local {
        fusb_int: gpio::AnyInput<'static>,
        periodic: PeriodicTimer<'static, Timer<TimerX<TIMG0>, Blocking>>,
        boost_enable: gpio::AnyOutput<'static>,
        v_sense: AdcPin<GpioPin<1>, ADC1, AdcCalCurve<ADC1>>,
        i_sense: AdcPin<GpioPin<0>, ADC1, AdcCalCurve<ADC1>>,
        input_i_sense: AdcPin<GpioPin<4>, ADC1, AdcCalCurve<ADC1>>,
        adc1: Adc<'static, ADC1>,
    }

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
        let io = Io::new_no_bind_interrupt(peripherals.GPIO, peripherals.IO_MUX);
        let mut delay = Delay::new(&clocks);

        let mut i2c = I2C::new(
            peripherals.I2C0,
            io.pins.gpio10,
            io.pins.gpio8,
            100u32.kHz(), // TODO: why doesn't 400khz work?
            &clocks,
        );

        let rmt = esp_hal::rmt::Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

        let mut rgb = rmt
            .channel0
            .configure(
                io.pins.gpio3,
                esp_hal::rmt::TxChannelConfig {
                    clk_divider: 2,
                    idle_output_level: false,
                    idle_output: true,
                    carrier_modulation: false,
                    carrier_high: 1,
                    carrier_low: 1,
                    carrier_level: false,
                },
            )
            .unwrap();

        // voltage and current sense // todo: cleanup / move into another file?
        let mut adc1_config = AdcConfig::new();
        let v_sense = adc1_config.enable_pin_with_cal(io.pins.gpio1, Attenuation11dB);
        let i_sense = adc1_config.enable_pin_with_cal(io.pins.gpio0, Attenuation11dB);
        let input_i_sense = adc1_config.enable_pin_with_cal(io.pins.gpio4, Attenuation11dB);
        let adc1 = Adc::new(peripherals.ADC1, adc1_config);

        let mut boost_enable = gpio::AnyOutput::new(io.pins.gpio21, gpio::Level::Low);
        let mut fusb_int = gpio::AnyInput::new(io.pins.gpio7, gpio::Pull::None);

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

        let TIMG0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let mut periodic = PeriodicTimer::new(TIMG0.timer0);
        periodic.enable_interrupt(true);
        periodic.start(100.millis()).unwrap();

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
