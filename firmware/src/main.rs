#![no_std]
#![no_main]
extern crate alloc;

use esp32c3_hal::{
    clock::{ClockControl, CpuClock},
    gpio::{Floating, GpioPin, Input},
    i2c::I2C,
    interrupt,
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    timer::TimerGroup,
    Delay, Rtc, IO,
};
use esp_backtrace as _;
use esp_println::println;

use core::cell::RefCell;
use critical_section::Mutex;

mod boost;
mod tcpc;
mod usb_pd;

// global reference for int pin so we can clear interrupt
static FUSB_INTERRUPT_PIN: Mutex<RefCell<Option<GpioPin<Input<Floating>, 7>>>> =
    Mutex::new(RefCell::new(None));

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
        100u32.kHz(), // TODO: why doesn't 400khz work?
        &mut system.peripheral_clock_control,
        &clocks,
    );


    let mut boost_enable = io.pins.gpio21.into_push_pull_output();
    let mut fusb_int = io.pins.gpio7.into_floating_input();

    interrupt::enable(Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();

    boost::init(&mut i2c, &mut boost_enable);
    tcpc::init(&mut i2c, &mut delay);
    tcpc::establish_pd_contract(&mut i2c, &mut fusb_int);

    // move fusb interrupt pin to global scope
    critical_section::with(|cs| FUSB_INTERRUPT_PIN.borrow_ref_mut(cs).replace(fusb_int));

    loop {
        tcpc::run(&mut i2c);
        // boost::run(&mut i2c);
    }
}

#[interrupt]
fn GPIO() {
    tcpc::INTERRUPT_PENDING.store(true, core::sync::atomic::Ordering::Relaxed);

    critical_section::with(|cs| {
        FUSB_INTERRUPT_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
