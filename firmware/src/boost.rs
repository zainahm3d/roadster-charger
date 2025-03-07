// Inteface to i2c DAC and boost converter

use bitfield::{bitfield, BitRange};
use core::sync::atomic::{AtomicU16, Ordering};
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use esp_println::println;

const ADDRESS: u8 = 0x48;
static TARGET_MV: AtomicU16 = AtomicU16::new(0);

use crate::{tcpc, vi_sense};

pub fn init<T: I2c, P: OutputPin>(i2c: &mut T, enable_pin: &mut P) {
    enable_pin.set_low().unwrap();

    // Set internal reference to 2.50V
    let mut gain = Gain(0x00);
    gain.set_buf_gain(false); // Gain of 1
    gain.set_ref_div(false); // Divide ref by 1
    i2c.write(
        ADDRESS,
        &[
            Register::Gain as u8,
            gain.bit_range(15, 8),
            gain.bit_range(7, 0),
        ],
    )
    .unwrap();

    set_voltage_mv(i2c, enable_pin, 0);
}

// Set the boost converter output voltage
// TODO: no floats
pub fn set_voltage_mv<T: I2c, P: OutputPin>(i2c: &mut T, enable_pin: &mut P, mut voltage_mv: u16) {
    voltage_mv = voltage_mv.clamp(0, 42_000);
    TARGET_MV.store(voltage_mv, Ordering::Relaxed);

    // If we're turning the output off, make sure it turns off by flipping the pin
    if voltage_mv == 0 {
        enable_pin.set_low().unwrap();
    }

    // 2^14 * (1.20 / 1.25) (DAC full scale output is 1.25V)
    const FULL_SCALE: u16 = 15729; // 1.20V @ DAC, 48V @ boost
    let mut data = Data(0x00);
    data.set_data(((voltage_mv as f32 / 48000.0) * FULL_SCALE as f32) as u16);

    i2c.write(
        ADDRESS,
        &[
            Register::Data as u8,
            data.bit_range(15, 8),
            data.bit_range(7, 0),
        ],
    )
    .unwrap();

    if voltage_mv == 0 {
        enable_pin.set_low().unwrap();
    } else {
        enable_pin.set_high().unwrap();
    }
}

pub fn run() {
    let target_mv = TARGET_MV.load(Ordering::Relaxed) as u32;
    let actual_mv = vi_sense::battery_voltage_mv();

    // The boost converter will present the input voltage minus a couple hundred mv at
    // the output if it is not enabled. This is due to the body diode of one of the
    // synchronous MOSFETs. During this case, we want to error out if we are drawing
    // current through the body diode for thermal reasons. This state should not ever be
    // active when charging a 36V nominal battery w/ a 20V PD source. Unfortunately,
    // there is no hardware solution to save us from this scenario other than the fact
    // that any 36V nominal battery should never be below the input voltage of this charger.
    let pdo_mv = tcpc::PDO_VOLTAGE_MV.load(Ordering::Relaxed);
    if target_mv < pdo_mv && vi_sense::output_current_ma() >= 100 {
        println!("Boost inactive, disconnect load!");
        // TODO: set LEDs flashing red
    } else if u32::abs_diff(target_mv, actual_mv) >= 1_000 && target_mv > pdo_mv {
        // We expect output voltage to equal VBUS voltage minus ~1V even when the boost is disabled
        // when no battery is connected.
        println!(
            "Boost voltage strayed from target! Target: {}mv, Actual: {}mv",
            target_mv, actual_mv
        )
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
