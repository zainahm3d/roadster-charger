// Inteface to i2c DAC and boost converter

use bitfield::{bitfield, BitRange};
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;

const ADDRESS: u8 = 0x48;

pub fn init<T: I2c, P: OutputPin>(i2c: &mut T, enable_pin: &mut P) {
    enable_pin.set_low().unwrap();

    // Set internal reference to 2.50V with no division
    let mut gain = Gain(0x00);
    gain.set_buf_gain(true);
    gain.set_ref_div(true);
    i2c.write(
        ADDRESS,
        &[
            Register::Gain as u8,
            gain.bit_range(15, 8),
            gain.bit_range(7, 0),
        ],
    )
    .unwrap();

    set_duty(i2c, enable_pin, 0);
}

// 16384 * 1.75 / 2.5
// 1.65V at DAC output equals 44V at boost output, add 100mV for drop across
// 1K resistor in between DAC and boost converter.
pub const DAC_MAX_OUTPUT: u16 = 11_470;
pub fn set_duty<T: I2c, P: OutputPin>(i2c: &mut T, enable_pin: &mut P, duty: u16) {
    // If we're turning the output off, make sure it turns off by flipping the pin
    if duty == 0 {
        enable_pin.set_low().unwrap();
    }

    let mut data = Data(0x00);
    data.set_data(duty);
    i2c.write(
        ADDRESS,
        &[
            Register::Data as u8,
            data.bit_range(15, 8),
            data.bit_range(7, 0),
        ],
    )
    .unwrap();

    if duty == 0 {
        enable_pin.set_low().unwrap();
    } else {
        enable_pin.set_high().unwrap();
    }
}

#[repr(u8)]
#[allow(dead_code)]
enum Register {
    Gain = 0x04,
    Data = 0x08,
}

bitfield! {
    pub struct Gain(u16);
    impl Debug;
    _, set_ref_div: 8;
    _, set_buf_gain: 0;
}

bitfield! {
    pub struct Data(u16);
    impl Debug;
    _, set_data: 15, 2;
}
