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
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    println!("booted!");

    let fusb_address: u8 = 0x50;

    // Reset the chip
    i2c.write(fusb_address, &[fusb307b::Register::RESET as u8, 0x01])
        .unwrap();

    delay.delay_ms(100u32);

    // Read cable orientation
    let mut ccstat: [u8; 1] = [0x00];
    i2c.write_read(
        fusb_address,
        &[fusb307b::Register::CCSTAT as u8],
        &mut ccstat,
    )
    .unwrap();
    let cc1_stat = ccstat[0] & 0b11;
    let cc2_stat = (ccstat[0] & 0b1100) >> 2;

    println!("CC1: {:#b}\tCC2: {:#b}", cc1_stat, cc2_stat);

    if cc1_stat != 0 {
        println!("CC1 connected!");
    } else if cc2_stat != 0 {
        println!("CC2 connected!");
    } else {
        println!("No CC connection!");
    }

    loop {}
}
