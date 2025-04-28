// Remote Control Peripheral (RMT) based WS2812b RGB LED driver
use bitfield::Bit;
use esp_hal::delay::Delay;
use esp_hal::gpio::{GpioPin, Level};
use esp_hal::rmt::{Channel, PulseCode, Rmt, TxChannel, TxChannelConfig, TxChannelCreator};
use esp_hal::time::{Duration, Instant};
use esp_hal::Blocking;

#[allow(unused)]
pub mod color {
    use super::Rgb;
    pub const RED: Rgb = Rgb { r: 255, g: 0, b: 0 };
    pub const YELLOW: Rgb = Rgb {
        r: 255,
        g: 255,
        b: 0,
    };
    pub const GREEN: Rgb = Rgb { r: 0, g: 255, b: 0 };
    pub const PURPLE: Rgb = Rgb {
        r: 255,
        g: 0,
        b: 255,
    };
    pub const OFF: Rgb = Rgb { r: 0, g: 0, b: 0 };
}

#[derive(Debug, Clone, Copy)]
pub struct Rgb {
    pub r: u32,
    pub g: u32,
    pub b: u32,
}

pub struct Led {
    pub pixel_buffer: [Rgb; 2],
    pub rmt_channel: Option<Channel<Blocking, 0>>,
    pub last_update_time: Instant,
}

impl Led {
    pub fn set_pixel(&mut self, pixel: usize, color: Rgb) {
        self.pixel_buffer[pixel].r = color.r;
        self.pixel_buffer[pixel].g = color.g;
        self.pixel_buffer[pixel].b = color.b;
        self.update();
    }

    fn update(&mut self) {
        // We need to wait >70us between updates to trigger a new "frame"
        let min_update_interval = Duration::from_micros(70);

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
        for pixel in 0..self.pixel_buffer.len() {
            // 24 pulses + 1 stop signal pulse
            let mut pixel_bits: u32 = 0;
            pixel_bits |= self.pixel_buffer[pixel].g << 16;
            pixel_bits |= self.pixel_buffer[pixel].r << 8;
            pixel_bits |= self.pixel_buffer[pixel].b;

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

        // Modify the final pulse to have a length2 of 0 to signal end of transaction to RMT.
        *pulse_train.last_mut().unwrap() &= 0x8000_FFFF;
        Delay::new().delay(min_update_interval);

        while Instant::now() - self.last_update_time < min_update_interval {}

        let channel = self.rmt_channel.take().unwrap();
        self.rmt_channel = Some(channel.transmit(&pulse_train).unwrap().wait().unwrap());
        self.last_update_time = Instant::now();
    }

    pub fn configure_rmt(rmt: Rmt<Blocking>, pin: GpioPin<3>) -> Channel<Blocking, 0> {
        rmt.channel0
            .configure(
                pin,
                TxChannelConfig::default()
                    .with_clk_divider(2)
                    .with_idle_output_level(Level::Low)
                    .with_idle_output(true)
                    .with_carrier_modulation(false)
                    .with_carrier_high(1)
                    .with_carrier_low(1)
                    .with_carrier_level(Level::Low),
            )
            .unwrap()
    }
}
