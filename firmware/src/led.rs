// Remote Control Peripheral (RMT) based WS2812b RGB LED driver
use bitfield::Bit;
use esp_hal::delay::Delay;
use esp_hal::gpio::Level;
use esp_hal::rmt::{Channel, PulseCode, TxChannel};
use esp_hal::time::{Duration, Instant};
use esp_hal::Blocking;

#[derive(Debug, Clone, Copy)]
pub struct Rgb {
    r: u32,
    g: u32,
    b: u32,
}

static mut PIXEL_BUFFER: [Rgb; 2] = [Rgb { r: 0, g: 0, b: 0 }, Rgb { r: 0, g: 0, b: 0 }];

// RMT HAL driver consumes RMT instance provided to it and gives it back, we
// do the same here (til I come up with something better)
pub fn set_pixel(
    channel: Channel<Blocking, 0>,
    pixel: u8,
    r: u8,
    g: u8,
    b: u8,
) -> Channel<Blocking, 0> {
    unsafe {
        PIXEL_BUFFER[pixel as usize].r = r as u32;
        PIXEL_BUFFER[pixel as usize].g = g as u32;
        PIXEL_BUFFER[pixel as usize].b = b as u32;
    }
    update(channel)
}

fn update(channel: Channel<Blocking, 0>) -> Channel<Blocking, 0> {
    // We need to wait >70us between updates to trigger a new "frame"
    let min_update_interval = Duration::from_micros(70);
    static mut LAST_UPDATE_TIME: Instant = Instant::EPOCH;

    let ns_per_tick = 25;

    let t0 = PulseCode::new(
        Level::High,
        400 / ns_per_tick,
        Level::Low,
        800 / ns_per_tick,
    );

    let t1 = PulseCode::new(
        Level::High,
        800 / ns_per_tick,
        Level::Low,
        450 / ns_per_tick,
    );

    let mut pulse_train: [u32; 48] = [t0; 48];
    unsafe {
        for pixel in 0..PIXEL_BUFFER.len() {
            // 24 pulses + 1 stop signal pulse
            let mut pixel_bits: u32 = 0;
            pixel_bits |= PIXEL_BUFFER[pixel].g << 16;
            pixel_bits |= PIXEL_BUFFER[pixel].r << 8;
            pixel_bits |= PIXEL_BUFFER[pixel].b;

            for (i, pulse) in pulse_train[pixel * 24..(pixel + 1) * 24]
                .iter_mut()
                .rev()
                .enumerate()
            {
                if pixel_bits.bit(i) {
                    *pulse = t1;
                } else {
                    *pulse = t0;
                }
            }
        }
    }

    // Modify the final pulse to have a length2 of 0 to signal end of transaction to RMT.
    *pulse_train.last_mut().unwrap() &= 0x8000_FFFF;
    Delay::new().delay(min_update_interval);

    unsafe { while Instant::now() - LAST_UPDATE_TIME < min_update_interval {} }

    let channel = channel.transmit(&pulse_train).unwrap().wait().unwrap();
    unsafe { LAST_UPDATE_TIME = Instant::now() }
    channel
}
