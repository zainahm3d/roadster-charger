// Remote Control Peripheral (RMT) based WS2812b RGB LED driver
use bitfield::Bit;
use esp32c3_hal::rmt::{Channel0, PulseCode, TxChannel};
use esp32c3_hal::systimer::SystemTimer;

#[derive(Debug, Clone, Copy)]
pub struct Rgb {
    r: u32,
    g: u32,
    b: u32,
}

static mut PIXEL_BUFFER: [Rgb; 2] = [Rgb { r: 0, g: 0, b: 0 }, Rgb { r: 0, g: 0, b: 0 }];

// RMT HAL driver consumes RMT instance provided to it and gives it back, we
// do the same here (til I come up with something better)
pub fn set_pixel(channel: Channel0<0>, pixel: u8, r: u8, g: u8, b: u8) -> Channel0<0> {
    unsafe {
        PIXEL_BUFFER[pixel as usize].r = r as u32;
        PIXEL_BUFFER[pixel as usize].g = g as u32;
        PIXEL_BUFFER[pixel as usize].b = b as u32;
    }
    update(channel)
}

fn update(channel: Channel0<0>) -> Channel0<0> {
    // We need to wait >50us between updates to trigger a new "frame"
    const MIN_UPDATE_INTERVAL: u64 = 70 * SystemTimer::TICKS_PER_SECOND / 1_000_000;
    static mut LAST_UPDATE_TIME: u64 = 0;

    // todo: use fugit for this
    let ns_per_tick = 25;
    let t0 = PulseCode {
        level1: true,
        length1: 400 / ns_per_tick,
        level2: false,
        length2: 800 / ns_per_tick,
    };
    let t1 = PulseCode {
        level1: true,
        length1: 800 / ns_per_tick,
        level2: false,
        length2: 450 / ns_per_tick,
    };

    let mut pulse_train: [PulseCode; 48] = [t0; 48];
    unsafe {
        for pixel in 0..PIXEL_BUFFER.len() {
            // 24 pulses + 1 stop signal pulse
            // initialize all to end so that we don't have to manually add "end" later
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

    pulse_train.last_mut().unwrap().length2 = 0; // signal end of transaction

    unsafe {
        while SystemTimer::now() - LAST_UPDATE_TIME < MIN_UPDATE_INTERVAL {}
    }

    let channel = channel.transmit(&pulse_train).wait().unwrap();
    unsafe { LAST_UPDATE_TIME = SystemTimer::now() }
    channel
}
