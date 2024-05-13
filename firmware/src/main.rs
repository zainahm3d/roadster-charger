#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::adc::Attenuation::*,
    analog::adc::*,
    clock::{ClockControl, CpuClock},
    delay::Delay,
    gpio::{Analog, Floating, GpioPin, Input, IO},
    i2c::I2C,
    interrupt::Priority,
    peripherals::Peripherals,
    peripherals::ADC1,
    prelude::*,
    rmt::TxChannelCreator,
};

use core::cell::RefCell;
use critical_section::Mutex;

mod boost;
mod led;
mod tcpc;
mod usb_pd;
mod vi_sense;

// global reference for int pin so we can clear interrupt
static FUSB_INTERRUPT_PIN: Mutex<RefCell<Option<GpioPin<Input<Floating>, 7>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    let mut io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut delay = Delay::new(&clocks);
    io.set_interrupt_handler(gpio_interrupt_handler);

    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio10,
        io.pins.gpio8,
        100u32.kHz(), // TODO: why doesn't 400khz work?
        &clocks,
        None,
    );

    let rmt = esp_hal::rmt::Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks, None).unwrap();

    let mut rgb = rmt
        .channel0
        .configure(
            io.pins.gpio3.into_push_pull_output(),
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
    let mut v_sense = adc1_config.enable_pin_with_cal::<GpioPin<Analog, 1>, AdcCalCurve<ADC1>>(
        io.pins.gpio1.into_analog(),
        Attenuation11dB,
    );
    let mut i_sense = adc1_config.enable_pin_with_cal::<GpioPin<Analog, 0>, AdcCalCurve<ADC1>>(
        io.pins.gpio0.into_analog(),
        Attenuation11dB,
    );
    let mut input_i_sense = adc1_config
        .enable_pin_with_cal::<GpioPin<Analog, 4>, AdcCalCurve<ADC1>>(
            io.pins.gpio4.into_analog(),
            Attenuation11dB,
        );
    let mut adc1 = ADC::<ADC1>::new(peripherals.ADC1, adc1_config);

    let mut boost_enable = io.pins.gpio21.into_push_pull_output();
    let mut fusb_int = io.pins.gpio7.into_floating_input();

    boost::init(&mut i2c, &mut boost_enable);
    tcpc::init(&mut i2c, &mut delay);

    // set PD LED to yellow while attempting to negotiate, red if failed, cyan if we have a contract
    rgb = led::set_pixel(rgb, 0, 20, 20, 0);
    rgb = led::set_pixel(rgb, 1, 20, 20, 0);

    if tcpc::establish_pd_contract(&mut i2c, &mut fusb_int) {
        rgb = led::set_pixel(rgb, 0, 0, 20, 5);
        _ = led::set_pixel(rgb, 1, 0, 20, 5);
    } else {
        rgb = led::set_pixel(rgb, 0, 20, 0, 0);
        _ = led::set_pixel(rgb, 1, 20, 0, 0);
        loop {} // no pd no charging
    }

    // move fusb interrupt pin to global scope
    critical_section::with(|cs| FUSB_INTERRUPT_PIN.borrow_ref_mut(cs).replace(fusb_int));

    loop {
        vi_sense::run(&mut adc1, &mut v_sense, &mut i_sense, &mut input_i_sense);
        tcpc::run(&mut i2c);
    }
}

#[handler(priority = "Priority::Priority3")]
fn gpio_interrupt_handler() {
    tcpc::INTERRUPT_PENDING.store(true, core::sync::atomic::Ordering::Relaxed);

    critical_section::with(|cs| {
        FUSB_INTERRUPT_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
