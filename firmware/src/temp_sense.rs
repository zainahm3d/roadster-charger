// Simple LM75B digital temperature sensor interface

use esp32c3_hal::i2c::I2C;
use esp32c3_hal::peripherals::I2C0;
use esp32c3_hal::prelude::*;
use zerocopy::AsBytes;

const ADDRESS: u8 = 0x4F;

// Sensor gives us an LSB of 0.5C, throw it away
// and return 1C precision. -25C to +125C
pub fn get_board_temp(i2c: &mut I2C<'_, I2C0>) -> i16 {
    let mut data: i16 = 0;
    i2c.read(ADDRESS, data.as_bytes_mut()).unwrap();
    data = data.to_be();
    // Rust gives us an arithmetic shift since data is an i16.
    // Data is a twos complement 9 bit number, left aligned
    (data >> 7) / 2
}
