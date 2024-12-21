// Simple LM75B digital temperature sensor interface

use esp_hal::i2c::I2C;
use esp_hal::peripherals::I2C0;
use esp_hal::Blocking;
use zerocopy::IntoBytes;

const ADDRESS: u8 = 0x4F;

// Sensor gives us an LSB of 0.5C, throw it away
// and return 1C precision. -25C to +125C
pub fn board_temp_c(i2c: &mut I2C<'_, I2C0, Blocking>) -> i16 {
    let mut data: i16 = 0;
    i2c.read(ADDRESS, data.as_mut_bytes()).unwrap();
    data = data.to_be();
    // Rust gives us an arithmetic shift since data is an i16.
    // Data is a twos complement 9 bit number, left aligned
    (data >> 7) / 2
}
