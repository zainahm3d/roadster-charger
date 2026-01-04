use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use shared::{pi::Pi, state::Mode, state::State};
use zerocopy::IntoBytes;

use crate::boost::{self, DAC_MAX_OUTPUT};
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
pub const TICKS_PER_SECOND: u32 = 100;
const MAX_OUTPUT_MV: f32 = 43_000.0;

pub fn init(state: &mut State) {
    state.i_ctrl = Pi::new(shared::pi::Config {
        kp: 0.0,
        ki: 0.01,
        i_max: DAC_MAX_OUTPUT as f32,
        output_max: DAC_MAX_OUTPUT as f32,
        p_ff: 0.0,
    });

    state.v_ctrl = Pi::new(shared::pi::Config {
        kp: (DAC_MAX_OUTPUT as f32) / MAX_OUTPUT_MV,
        ki: 0.1,
        i_max: DAC_MAX_OUTPUT as f32,
        output_max: DAC_MAX_OUTPUT as f32,
        p_ff: (DAC_MAX_OUTPUT as f32) / MAX_OUTPUT_MV,
    });
}

pub fn run<T: I2c, P: OutputPin>(
    i2c: &mut T,
    enable_pin: &mut P,
    s: &mut State,
    vi_sense: &VISense,
    leds: &mut Led,
) {
    s.tick += 1;
    s.board_temp_c = temp_sense::board_temp_c(i2c);
    s.input_mv = tcpc::vbus_mv(i2c);
    s.input_ma = vi_sense.input_current_ma();
    s.output_mv = vi_sense.battery_voltage_mv();
    s.output_ma = vi_sense.output_current_ma();

    esp_println::Printer::write_bytes(s.as_bytes());

    if s.board_temp_c >= 90 {
        s.mode = Mode::OverTemp;
    }

    if s.output_mv >= CV_TARGET_MV + 500 {
        disable(s, leds, color::WHITE);
    }

    match s.mode {
        // Battery has been detected, slowly ramp up until we flow current
        Mode::Ramping => {
            if s.output_mv < MIN_BATTERY_VOLTAGE_MV {
                disable(s, leds, color::RED); // Battery is gone
            } else if s.output_ma < MIN_CURRENT_MA {
                if s.duty >= DAC_MAX_OUTPUT {
                    disable(s, leds, color::RED);
                } else {
                    s.duty += 5u16;
                    boost::set_duty(i2c, enable_pin, s.duty);
                }
            } else {
                s.mode = Mode::ConstantCurrent;
                s.i_ctrl.i = s.duty as f32; // Don't wait for i term
                s.i_ctrl.output = s.duty as f32;
                leds.set_pixel(1, color::YELLOW);
            }
        }

        Mode::ConstantCurrent => {
            update_cc_target(s);

            if s.output_mv >= CV_TARGET_MV {
                s.target_ma = 0;
                s.mode = Mode::ConstantVoltage;
                s.i_ctrl.reset();
                leds.set_pixel(1, color::BLUE);
            } else if s.output_ma < MIN_CURRENT_MA {
                disable(s, leds, color::RED); // Battery is gone
            } else if s.output_ma.abs_diff(s.target_ma) >= 10 {
                s.duty = s.i_ctrl.update(s.output_ma as f32, s.target_ma as f32) as u16;
                boost::set_duty(i2c, enable_pin, s.duty);
            }
        }

        Mode::ConstantVoltage => {
            s.duty = s.v_ctrl.update(s.output_mv as f32, CV_TARGET_MV as f32) as u16;

            if s.output_ma <= CHARGING_CUTOFF_MA {
                // Charging is done
                disable(s, leds, color::GREEN);
            } else {
                boost::set_duty(i2c, enable_pin, s.duty);
            }
        }

        Mode::Disabled => {
            boost::set_duty(i2c, enable_pin, 0);

            s.i_ctrl.reset();
            s.v_ctrl.reset();

            // If boost is off and we see a voltage then a battery has been plugged in.
            // Only switch to CC if battery exists and is > 0.5V below fully charged target.
            if s.output_mv > MIN_BATTERY_VOLTAGE_MV && s.output_mv < CV_TARGET_MV - 500 {
                // Make sure we've been in the disabled state for at least 3 seconds before
                // enabling the charger. This allows time for output caps to discharge
                // once a battery has been disconnected.
                if s.tick.wrapping_sub(s.tick_disabled) > 3 * TICKS_PER_SECOND {
                    s.mode = Mode::Ramping;
                    leds.set_pixel(1, color::YELLOW);
                }
            }
        }

        Mode::OverTemp => {
            boost::set_duty(i2c, enable_pin, 0);
            if s.tick.is_multiple_of(TICKS_PER_SECOND) {
                leds.set_pixel(1, color::OFF);
            } else if s.tick.is_multiple_of(TICKS_PER_SECOND / 2) {
                leds.set_pixel(1, color::RED);
            }
        }
    }
}

fn disable(s: &mut State, leds: &mut Led, color: crate::led::Rgb) {
    s.target_ma = 0;
    s.mode = Mode::Disabled;
    s.tick_disabled = s.tick;
    s.duty = 0;
    leds.set_pixel(1, color);
}

// Calculate max charge current based on charger PDO and current temperature.
fn update_cc_target(state: &mut State) {
    if state.output_mv > MIN_BATTERY_VOLTAGE_MV {
        // Only output 90% of brick's advertised capabilities to account for
        // power dissipated in cable and our own 92ish % efficiency.
        let pdo_mw = state.pdo_ma as f32 * state.pdo_mv as f32 / 1000.0;
        let max_output_mw = pdo_mw * 0.9;

        // Based on PDO and current output voltage, never charge at > 1.5A
        state.target_ma =
            ((max_output_mw / state.output_mv as f32 * 1000.0).clamp(0.0, 1_500.0)) as u32;
    } else {
        state.target_ma = 0;
    }
}
