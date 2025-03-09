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
    tick_disabled: u32,

    board_temp_c: i16,

    target_ma: u32,
    duty: u16,

    input_mv: u32,
    input_ma: u32,

    output_mv: u32,
    output_ma: u32,
    max_output_ma: u32,

    pub pdo_mv: u32,
    pub pdo_ma: u32,
}

impl State {
    // Calculate max charge current based on charger PDO and current temperature.
    fn update_max_charge_current(&mut self) {
        // Only output 85% of brick's advertised capabilities to account for
        // power dissapated in cable and our own 92ish % efficiency.
        let pdo_mw = self.pdo_ma * self.pdo_mv / 1000;
        let max_output_mw = (pdo_mw as f32 * 0.85) as u32;

        // Based on PDO and current output voltage, never charge at > 2A
        esp_println::dbg!(max_output_mw / self.output_mv * 1000);
        self.max_output_ma = (max_output_mw / self.output_mv * 1000).clamp(0, 2_000);
    }
}

impl Default for State {
    fn default() -> Self {
        State {
            tick: 0,
            mode: Mode::Disabled,
            tick_disabled: 0,
            board_temp_c: 0,
            target_ma: 0,
            duty: 0,
            input_mv: 0,
            input_ma: 0,
            output_mv: 0,
            output_ma: 0,
            max_output_ma: 0,
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
            s.update_max_charge_current();
            s.target_ma = s.target_ma.clamp(0, s.max_output_ma);
            let p_error = s.target_ma as i32 - s.output_ma as i32;

            // Slew rate limit only in positive direction
            let duty_shift = p_error.clamp(i32::MIN, MAX_BOOST_DIFF);

            // Prevent windup past boost limits
            s.duty = (s.duty as i32 + duty_shift).clamp(0, boost::DAC_MAX_OUTPUT as i32) as u16;

            if s.output_mv >= CV_TARGET_MV {
                s.mode = Mode::ConstantVoltage;
            } else if s.output_ma < MIN_CURRENT_MA {
                s.mode = Mode::Disabled; // Battery is gone!
                s.tick_disabled = s.tick;
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
                s.tick_disabled = s.tick;
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
                // Make sure we've been in the disabled state for at least 3 seconds before
                // enabling the charger. This allows time for output caps to discharge
                // once a battery has been disconnected.
                if s.tick - s.tick_disabled > 30 {
                    s.mode = Mode::ConstantCurrent;
                }
            }
        }
    }
}
