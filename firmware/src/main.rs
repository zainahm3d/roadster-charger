#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl, i2c::I2C, peripherals::Peripherals, prelude::*, timer::TimerGroup, Delay,
    Rtc, IO,
};
use zerocopy::*;

mod fusb307b;
mod usb_pd;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
    }

    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}

#[repr(C, packed)]
#[derive(FromBytes, FromZeroes)]
struct ShtData {
    temperature: u16,
    temperature_checksum: u8,
    humidity: u16,
    humidity_checksum: u8,
}

#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut delay = Delay::new(&clocks);

    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio10,
        io.pins.gpio8,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    println!("booted!");

    // Enable the boost converter
    let mut ps_enable = io.pins.gpio21;
    ps_enable.set_to_push_pull_output();
    ps_enable.set_output_high(true);
    println!("boost enabled");

    // Set up DAC
    let dac_addr: u8 = 0x48;

    // Internal reference is 2.5V, need to cut in half
    // BUFF-GAIN bit set to 0, REF-DIV bit set to 1
    i2c.write(dac_addr, &[0x04, 0x01, 0x00]).unwrap();

    // // Set DAC output to max voltage (1.25V)
    // // set UPPER 14 bits of DAC-DATA (0xfffc) (left aligned for speed)
    // i2c.write(dac_addr, &[0x08, 0xFF, 0xFC]).unwrap();

    let mut dac_value: u16 = 0;
    loop {
        // Slowly ramp dac output up to full scale
        if dac_value >= (i32::pow(2, 14)) as u16 {
            dac_value = 0;
        }

        let high_byte: u8 = (((dac_value << 2) & 0xFF00) >> 8) as u8;
        let low_byte: u8 = ((dac_value << 2) & 0x00FF) as u8;

        i2c.write(dac_addr, &[0x08, high_byte, low_byte]).unwrap();

        let dac_voltage: f32 = dac_value as f32 / u32::pow(2, 14) as f32 * 1.25;
        println!("DAC voltage: {:?}", dac_voltage);
        // println!("Bin: 0b{:b}", high_byte << 7 | low_byte);

        dac_value += 10;

        delay.delay_ms(1u32);
    }
}
