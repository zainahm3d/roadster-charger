// Inteface to i2c DAC and boost converter

use bitfield::{bitfield, BitRange};
use esp32c3_hal::i2c::I2C;
use esp32c3_hal::peripherals::I2C0;
use esp32c3_hal::prelude::*;

const ADDRESS: u8 = 0x48;

pub fn init(i2c: &mut I2C<'_, I2C0>) {
    // Set internal reference to 1.25V
    let mut gain = Gain(0x00);
    gain.set_buf_gain(false);
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

    set_voltage_mv(i2c, 0);
}

// Set the boost converter output voltage
pub fn set_voltage_mv(i2c: &mut I2C<'_, I2C0>, voltage_mv: u32) {
    let mut data = Data(0x00);

    if voltage_mv == 0 {
        // todo!("flip off boost converter enable pin");
        data.set_data(0);
    } else {
        // todo!("flip on boost enable pin");
        data.set_data((voltage_mv as f32 / 38.4) as u16); // 48.0 / 1.25 = 38.4
    }

    i2c.write(
        ADDRESS,
        &[
            Register::Data as u8,
            data.bit_range(15, 8),
            data.bit_range(7, 0),
        ],
    )
    .unwrap();
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
