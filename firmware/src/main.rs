#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc, IO, Delay, i2c::I2C};
use zerocopy::*;


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

    // i2c.write(sensor_address, &read_cmd).unwrap();
    // led.set_output_high(true);
    // delay.delay_ms(15 as u32);

    // See if devices are alive
    let fusb_address: u8 = 0x50;
    let lm75_address: u8 = 0x4F;
    let dac_address: u8 = 0x48;
    let phony_address: u8 = 0x49;

    let device_address = [
        fusb_address,
        lm75_address,
        dac_address,
        // phony_address, // panics with AckCheckFailed
    ];

    let nothing: [u8; 1] = [0x00];

    for address in device_address {
        println!("Attempting address: 0x{:02x}", address);
        i2c.write(address, &nothing).unwrap();
        delay.delay_ms(100u32);
    }

    loop {
    }
}
