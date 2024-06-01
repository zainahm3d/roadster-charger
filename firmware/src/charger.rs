use esp_hal::gpio::*;
use esp_hal::i2c::I2C;
use esp_hal::peripherals::I2C0;
use esp_hal::systimer::SystemTimer;
use esp_hal::Blocking;
use esp_println::println;

use crate::boost;
use crate::tcpc;
use crate::vi_sense;
use crate::temp_sense;

const UPDATE_INTERVAL: u64 = SystemTimer::TICKS_PER_SECOND / 10;
const CHARGING_TARGET_MV: u32 = 41_500;
const CHARGING_CURRENT_MA: u32 = 500;

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

    p_gain: i32,
    p_error: i32,
    p_term: i32,

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

    p_gain: 10, // 1mV / 10mA
    p_error: 0,
    p_term: 0,

    input_mv: 0,
    input_ma: 0,
    output_mv: 0,
    output_ma: 0,
};

// todo: LED control
pub fn run(i2c: &mut I2C<'_, I2C0, Blocking>, enable_pin: &mut GpioPin<Output<PushPull>, 21>) {
    static mut LAST_UPDATE_TIME: u64 = 0;

    // Safety: none of the statics here are touched by any interrupts,
    // and run() is only called from the main loop + not reentrant.
    unsafe {
        let now = SystemTimer::now();
        if now - LAST_UPDATE_TIME >= UPDATE_INTERVAL {
            LAST_UPDATE_TIME = now;

            match CONTROLLER.system_state {
                State::NoBattery => {
                    println!("charger: no battery");
                    let battery_voltage = vi_sense::battery_voltage_mv();
                    if battery_voltage > 28_000 && battery_voltage < CHARGING_TARGET_MV {
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
                    if vi_sense::output_current_ma() <= 150 {
                        CONTROLLER.system_state = State::ChargingComplete;
                    } else {
                        // todo: configurable charge voltage target
                        CONTROLLER.target_mv = CHARGING_TARGET_MV;
                        CONTROLLER.charger_mode = ChargerMode::ConstantVoltage;
                    }
                }

                State::ChargingComplete => {
                    println!("charger: charging complete");
                    if vi_sense::battery_voltage_mv() < 10_000 {
                        CONTROLLER.system_state = State::NoBattery;
                    } else {
                        CONTROLLER.charger_mode = ChargerMode::Disabled;
                    }
                }
            }
            update_output(i2c, enable_pin); // todo: make this safe
        }
    }
}

fn update_output(
    i2c: &mut I2C<'_, I2C0, Blocking>,
    enable_pin: &mut GpioPin<Output<PushPull>, 21>,
) {
    // todo: better access to controller so we can remove this huge unsafe block
    unsafe {
        CONTROLLER.board_temp_c = temp_sense::board_temp_c(i2c);
        CONTROLLER.input_mv = tcpc::vbus_mv(i2c);
        CONTROLLER.input_ma = vi_sense::input_current_ma();

        CONTROLLER.output_ma = vi_sense::output_current_ma();
        CONTROLLER.output_mv = vi_sense::battery_voltage_mv();

        CONTROLLER.tick = CONTROLLER.tick.wrapping_add(1);

        match CONTROLLER.charger_mode {
            // todo: there's too much typecasting here
            ChargerMode::ConstantCurrent => {
                CONTROLLER.p_error = CONTROLLER.target_ma as i32 - CONTROLLER.output_ma as i32;
                CONTROLLER.p_term = CONTROLLER.p_error * CONTROLLER.p_gain;

                // Cap the P term so that it can't drive target voltage below 0v or above charging target
                if (CONTROLLER.p_term + CONTROLLER.target_mv as i32) < 0 {
                    CONTROLLER.p_term = -(CONTROLLER.target_mv as i32);
                } else if CHARGING_TARGET_MV as i32 - CONTROLLER.p_term < 0 {
                    // CONTROLLER.p_term = CONTROLLER.target_mv as i32 - CHARGING_TARGET_MV as i32; // wrong
                }

                // The above checks guarentee that this value is between [0, CHARGING_TARGET_MV]
                let new_target = CONTROLLER.target_mv as i32 + CONTROLLER.p_term;
                assert!(new_target >= 0, "controller target less than 0mV");
                assert!(
                    new_target <= CHARGING_TARGET_MV as i32,
                    "controller target greater than charge target voltage"
                );
                CONTROLLER.target_mv = new_target as u32;
                boost::set_voltage_mv(i2c, enable_pin, CONTROLLER.target_mv as u16);
            }

            ChargerMode::ConstantVoltage => {
                // todo: we likely need to also look at the measured voltage at the battery
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
