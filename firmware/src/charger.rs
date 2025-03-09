use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use esp_println::dbg;

use crate::boost;
use crate::tcpc;
use crate::temp_sense;
use crate::vi_sense;

// Battery specs for 10S2P INR18650-35E pack
const CV_TARGET_MV: u32 = 41_500;
// 75mA per parallel string
const CHARGING_CUTOFF_MA: u32 = 150;
const MIN_BATTERY_VOLTAGE_MV: u32 = 28_000;
const MIN_CURRENT_MA: u32 = 50;

#[derive(Debug)]
enum Mode {
    ConstantCurrent,
    ConstantVoltage,
    Disabled,
}

// This struct is currently doubling as a telemetry struct
#[derive(Debug)]
pub struct State {
    tick: u32,
    mode: Mode,

    board_temp_c: i16,

    target_ma: u32,
    duty: u16,

    input_mv: u32,
    input_ma: u32,

    output_mv: u32,
    output_ma: u32,

    pub pdo_mv: u32,
    pub pdo_ma: u32,
}

impl Default for State {
    fn default() -> Self {
        State {
            mode: Mode::Disabled,
            board_temp_c: 0,
            tick: 0,
            target_ma: 2_000,
            duty: 0,
            input_mv: 0,
            input_ma: 0,
            output_mv: 0,
            output_ma: 0,
            pdo_mv: 0,
            pdo_ma: 0,
        }
    }
}

// Run charge controller. CC and CV controllers are slew rate limited P controllers.
// In practice, they act as integrating controllers at large P errors, and P controllers
// at small P errors.
pub fn run<T: I2c, P: OutputPin>(i2c: &mut T, enable_pin: &mut P, s: &mut State) {
    // Slew rate in DAC ticks / update period
    const MAX_BOOST_DIFF: i32 = 250;

    s.tick += 1;
    s.board_temp_c = temp_sense::board_temp_c(i2c);
    s.input_mv = tcpc::vbus_mv(i2c);
    s.input_ma = vi_sense::input_current_ma();
    s.output_mv = vi_sense::battery_voltage_mv();
    s.output_ma = vi_sense::output_current_ma();
    dbg!(&s); // Print out the results of the last update

    match s.mode {
        Mode::ConstantCurrent => {
            let p_error = s.target_ma as i32 - s.output_ma as i32;

            // Slew rate limit only in positive direction
            let duty_shift = p_error.clamp(i32::MIN, MAX_BOOST_DIFF);

            // Prevent windup past boost limits
            s.duty = (s.duty as i32 + duty_shift).clamp(0, boost::DAC_MAX_OUTPUT as i32) as u16;

            if s.output_mv >= CV_TARGET_MV {
                s.mode = Mode::ConstantVoltage;
            } else if s.output_ma < MIN_CURRENT_MA {
                s.mode = Mode::Disabled; // Battery is gone!
            } else {
                boost::set_duty(i2c, enable_pin, s.duty);
            }
        }

        Mode::ConstantVoltage => {
            let p_error = CV_TARGET_MV as i32 - s.output_mv as i32;

            // Slew rate limit only in positive direction
            let duty_shift = p_error.clamp(i32::MIN, MAX_BOOST_DIFF);

            // Clamp to boost converter limits
            s.duty = (s.duty as i32 + duty_shift).clamp(0, boost::DAC_MAX_OUTPUT as i32) as u16;

            if s.output_ma <= CHARGING_CUTOFF_MA {
                s.mode = Mode::Disabled;
                // Set LED to green
            } else {
                boost::set_duty(i2c, enable_pin, s.duty);
            }
        }

        Mode::Disabled => {
            boost::set_duty(i2c, enable_pin, 0);

            // If boost is off and we see a voltage then a battery has been plugged in.
            // Only switch to CC if battery exists and is > 0.5V below fully charged target.
            if s.output_mv > MIN_BATTERY_VOLTAGE_MV && s.output_mv < CV_TARGET_MV - 500 {
                s.mode = Mode::ConstantCurrent;
                // Set LED to yellow
            } // Else set LED to off
        }
    }
}
