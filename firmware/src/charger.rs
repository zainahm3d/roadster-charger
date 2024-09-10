use esp_hal::gpio::*;
use esp_hal::i2c::I2C;
use esp_hal::peripherals::I2C0;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::Blocking;
use esp_println::println;

use crate::boost;
use crate::tcpc;
use crate::temp_sense;
use crate::vi_sense;

const CHARGING_TARGET_MV: u32 = 41_500;
const CHARGING_CURRENT_MA: u32 = 750;

// 75mA per parallel string
const CHARGING_CUTOFF_CV_MA: u32 = 150;
const MIN_BATTERY_VOLTAGE_MV: u32 = 28_000;

#[derive(Debug)]
enum State {
    NoBattery,
    ChargingCC,
    ChargingCV,
    ChargingComplete,
}

#[derive(Debug)]
enum ChargerMode {
    ConstantCurrent,
    ConstantVoltage,
    Disabled,
}

// This struct is currently doubling as a telemetry struct
#[derive(Debug)]
struct ChargeController {
    system_state: State,
    charger_mode: ChargerMode,

    board_temp_c: i16,

    tick: u32,

    target_ma: u32,
    target_mv: u32,

    input_mv: u32,
    input_ma: u32,

    output_mv: u32,
    output_ma: u32,
}

static mut CONTROLLER: ChargeController = ChargeController {
    system_state: State::NoBattery,
    charger_mode: ChargerMode::Disabled,

    board_temp_c: 0,

    tick: 0,

    target_ma: 0,
    target_mv: 0,

    input_mv: 0,
    input_ma: 0,
    output_mv: 0,
    output_ma: 0,
};

// todo: LED control
pub fn run(i2c: &mut I2C<'_, I2C0, Blocking>, enable_pin: &mut AnyOutput) {
    static mut LAST_UPDATE_TIME: u64 = 0;

    // Safety: none of the statics here are touched by any interrupts,
    // and run() is only called from the main loop + not reentrant.
    unsafe {
        let now = SystemTimer::now();
        if now - LAST_UPDATE_TIME >= SystemTimer::ticks_per_second() / 10 {
            LAST_UPDATE_TIME = now;

            match CONTROLLER.system_state {
                State::NoBattery => {
                    println!("charger: no battery");
                    let battery_voltage = vi_sense::battery_voltage_mv();
                    if battery_voltage > MIN_BATTERY_VOLTAGE_MV
                        && battery_voltage < CHARGING_TARGET_MV
                    {
                        // battery is connected and not under/over voltage
                        // todo: decide charge current based on PD source caps
                        CONTROLLER.system_state = State::ChargingCC;
                    }
                }

                State::ChargingCC => {
                    if vi_sense::battery_voltage_mv() >= CHARGING_TARGET_MV {
                        CONTROLLER.system_state = State::ChargingCV;
                    } else {
                        println!("charger: charging cc");
                        CONTROLLER.target_ma = CHARGING_CURRENT_MA;
                        CONTROLLER.charger_mode = ChargerMode::ConstantCurrent;
                    }
                }

                State::ChargingCV => {
                    println!("charger: charging cv");
                    if vi_sense::output_current_ma() <= CHARGING_CUTOFF_CV_MA {
                        CONTROLLER.system_state = State::ChargingComplete;
                    } else {
                        CONTROLLER.charger_mode = ChargerMode::ConstantVoltage;
                    }
                }

                State::ChargingComplete => {
                    println!("charger: charging complete");
                    if vi_sense::battery_voltage_mv() < MIN_BATTERY_VOLTAGE_MV {
                        CONTROLLER.system_state = State::NoBattery;
                    } else {
                        CONTROLLER.charger_mode = ChargerMode::Disabled;
                    }
                }
            }
            update_output(i2c, enable_pin);
        }
    }
}

fn update_output(i2c: &mut I2C<'_, I2C0, Blocking>, enable_pin: &mut AnyOutput) {
    unsafe {
        CONTROLLER.board_temp_c = temp_sense::board_temp_c(i2c);
        CONTROLLER.input_mv = tcpc::vbus_mv(i2c);
        CONTROLLER.input_ma = vi_sense::input_current_ma();

        CONTROLLER.output_ma = vi_sense::output_current_ma();
        CONTROLLER.output_mv = vi_sense::battery_voltage_mv();

        CONTROLLER.tick = CONTROLLER.tick.wrapping_add(1);

        match CONTROLLER.charger_mode {
            // integrating controller
            ChargerMode::ConstantCurrent => {
                // positive error_ma means we need more current
                let error_ma: i32 = CONTROLLER.target_ma as i32 - CONTROLLER.output_ma as i32;
                if error_ma > 100 {
                    CONTROLLER.target_mv += 100;
                } else if error_ma < -100 {
                    CONTROLLER.target_mv -= 100;
                } else if error_ma > 1 {
                    CONTROLLER.target_mv += 1;
                } else if error_ma < 1 {
                    CONTROLLER.target_mv -= 1;
                }
                CONTROLLER.target_mv = CONTROLLER
                    .target_mv
                    .clamp(MIN_BATTERY_VOLTAGE_MV, CHARGING_TARGET_MV);
                boost::set_voltage_mv(i2c, enable_pin, CONTROLLER.target_mv as u16);
            }

            ChargerMode::ConstantVoltage => {
                // treat the output_mv measurement as source of truth, target is not as accurate.
                // positive error_mv means we need more voltage
                let error_mv: i32 = CHARGING_TARGET_MV as i32 - CONTROLLER.output_mv as i32;
                if error_mv >= 100 {
                    CONTROLLER.target_mv += 100;
                } else if error_mv <= -100 {
                    CONTROLLER.target_mv -= 100;
                } else if error_mv > 1 {
                    CONTROLLER.target_mv += 1;
                } else if error_mv < 1 {
                    CONTROLLER.target_mv -= 1;
                }
                // allow 1V of slop in boost controller
                CONTROLLER.target_mv = CONTROLLER
                    .target_mv
                    .clamp(MIN_BATTERY_VOLTAGE_MV, CHARGING_TARGET_MV + 1_000);
                boost::set_voltage_mv(i2c, enable_pin, CONTROLLER.target_mv as u16);
            }

            ChargerMode::Disabled => {
                CONTROLLER.target_ma = 0;
                CONTROLLER.target_mv = 0;
                boost::set_voltage_mv(i2c, enable_pin, 0);
            }
        }
        println!("{:?}", CONTROLLER)
    }
}
