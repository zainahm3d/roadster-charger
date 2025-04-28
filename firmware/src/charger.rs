use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use esp_println::dbg;

use crate::boost;
use crate::led::{color, Led};
use crate::tcpc;
use crate::temp_sense;
use crate::vi_sense::VISense;

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
    OverTemp,
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

    pub pdo_mv: u32,
    pub pdo_ma: u32,
}

impl State {
    // Calculate max charge current based on charger PDO and current temperature.
    fn update_target_ma(&mut self) {
        if true {
            // Only output 90% of brick's advertised capabilities to account for
            // power dissapated in cable and our own 92ish % efficiency.
            let pdo_mw = self.pdo_ma as f32 * self.pdo_mv as f32 / 1000.0;
            let max_output_mw = pdo_mw * 0.9;

            // Based on PDO and current output voltage, never charge at > 2A
            self.target_ma =
                ((max_output_mw / self.output_mv as f32 * 1000.0).clamp(0.0, 1_500.0)) as u32;
        } else {
            self.target_ma = 0;
        }
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
            pdo_mv: 0,
            pdo_ma: 0,
        }
    }
}

// Run charge controller. CC and CV controllers are slew rate limited P controllers.
// In practice, they act as integrating controllers at large P errors, and P controllers
// at small P errors.
pub fn run<T: I2c, P: OutputPin>(
    i2c: &mut T,
    enable_pin: &mut P,
    s: &mut State,
    vi_sense: &VISense,
    leds: &mut Led,
) {
    // Slew rate in DAC ticks / update period
    const MAX_BOOST_DIFF: i32 = 25;

    s.tick += 1;
    s.board_temp_c = temp_sense::board_temp_c(i2c);
    s.input_mv = tcpc::vbus_mv(i2c);
    s.input_ma = vi_sense.input_current_ma();
    s.output_mv = vi_sense.battery_voltage_mv();
    s.output_ma = vi_sense.output_current_ma();
    dbg!(&s); // Print out the results of the last update

    if s.board_temp_c >= 90 {
        s.mode = Mode::OverTemp;
    }

    match s.mode {
        Mode::ConstantCurrent => {
            s.update_target_ma();
            let p_error = s.target_ma as i32 - s.output_ma as i32;

            // Slew rate limit only in positive direction
            let duty_shift = p_error.clamp(i32::MIN, MAX_BOOST_DIFF);

            // Prevent windup past boost limits
            s.duty = (s.duty as i32 + duty_shift).clamp(0, boost::DAC_MAX_OUTPUT as i32) as u16;

            if s.output_mv >= CV_TARGET_MV {
                s.target_ma = 0;
                s.mode = Mode::ConstantVoltage;
            } else if s.output_ma < MIN_CURRENT_MA {
                // Battery is gone!
                s.target_ma = 0;
                s.mode = Mode::Disabled;
                s.tick_disabled = s.tick;
                leds.set_pixel(1, color::RED);
            } else {
                boost::set_duty(i2c, enable_pin, s.duty);
            }
        }

        Mode::ConstantVoltage => {
            let p_error = CV_TARGET_MV as i32 - s.output_mv as i32;

            let duty_shift = if p_error.abs() < 100 {
                // Limit slew rate severely within 100mV of target
                p_error.clamp(-1, 1)
            } else {
                // If error is large, allow large negative movement
                p_error.clamp(i32::MIN, MAX_BOOST_DIFF)
            };

            // Clamp to boost converter limits
            s.duty = (s.duty as i32 + duty_shift).clamp(0, boost::DAC_MAX_OUTPUT as i32) as u16;

            if s.output_ma <= CHARGING_CUTOFF_MA {
                // Charging is done
                s.mode = Mode::Disabled;
                s.tick_disabled = s.tick;
                leds.set_pixel(1, color::GREEN);
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
                    leds.set_pixel(1, color::YELLOW);
                }
            }
        }

        Mode::OverTemp => {
            boost::set_duty(i2c, enable_pin, 0);
            if s.tick % 10 == 0 {
                leds.set_pixel(1, color::OFF);
            } else if s.tick % 5 == 0 {
                leds.set_pixel(1, color::RED);
            }
        }
    }
}
