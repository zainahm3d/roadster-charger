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

    // SHTC3
    let sensor_address: u8 = 0x70;
    let read_cmd: [u8; 2] = [0x78, 0x66];

    let mut led = io.pins.gpio7.into_push_pull_output();
    led.set_output_high(true);

    println!("booted!");



    loop {
        // tell sensor to read
        i2c.write(sensor_address, &read_cmd).unwrap();
        led.set_output_high(true);
        delay.delay_ms(15 as u32);

        // read out data from sensor
        let mut databuf: [u8; 6] = [0x00; 6];
        i2c.read(sensor_address, &mut databuf).unwrap();
        let data = ShtData::read_from(&databuf).unwrap();

        let temp: f32 = -45.0 + (175.0 * (data.temperature.swap_bytes() as f32 / 65536.0));
        let temp_f: f32 = (temp * 1.8) + 32.0;
        let rh: f32 = 100.0 * (data.humidity.swap_bytes() as f32 / 65536.0);

        println!("Temp: {:.2}C / {:.2}F, rh: {:.2}%", temp, temp_f, rh);
        led.set_output_high(false);
        delay.delay_ms(85 as u32);
    }
}
