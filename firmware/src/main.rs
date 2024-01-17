#![no_std]
#![no_main]
extern crate alloc;

use esp32c3_hal::{
    clock::{ClockControl, CpuClock},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay, Rtc, IO,
};
use esp_backtrace as _;
use esp_println::println;

mod boost;
mod tcpc;
mod usb_pd;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
    }

    unsafe {
        let heap_start = &core::ptr::addr_of!(_heap_start) as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}

#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
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

    let mut boost_enable = io.pins.gpio21.into_push_pull_output();
    boost::init(&mut i2c, &mut boost_enable);
    tcpc::init(&mut i2c, &mut delay);
    tcpc::establish_pd_contract(&mut i2c);

    loop {}
}
