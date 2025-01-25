// Simple LM75B digital temperature sensor interface
use embedded_hal::i2c::I2c;
use zerocopy::IntoBytes;

const ADDRESS: u8 = 0x4F;

// Sensor gives us an LSB of 0.5C, throw it away
// and return 1C precision. -25C to +125C
pub fn board_temp_c<T: I2c>(i2c: &mut T) -> i16 {
    let mut data: i16 = 0;
    i2c.read(ADDRESS, data.as_mut_bytes()).unwrap();
    data = data.to_be();
    // Rust gives us an arithmetic shift since data is an i16.
    // Data is a twos complement 9 bit number, left aligned
    (data >> 7) / 2
}
