#![no_std]

use zerocopy::{Immutable, IntoBytes, KnownLayout, TryFromBytes};

pub mod state {
    use super::*;
    use crate::pi::Pi;

    #[repr(u32)]
    #[derive(Debug, Clone, Copy, IntoBytes, TryFromBytes, Immutable, Default)]
    pub enum Mode {
        #[default]
        Disabled,
        Ramping,
        ConstantCurrent,
        ConstantVoltage,
        OverTemp,
    }

    #[derive(Debug, Clone, Copy, IntoBytes, TryFromBytes, Immutable, KnownLayout, Default)]
    pub struct State {
        pub tick: u32,
        pub mode: Mode,
        pub tick_disabled: u32,

        pub board_temp_c: i16,

        pub target_ma: u32,
        pub duty: u16,

        pub input_mv: u32,
        pub input_ma: u32,

        pub output_mv: u32,
        pub output_ma: u32,

        pub pdo_mv: u32,
        pub pdo_ma: u32,

        pub i_ctrl: Pi,
        pub v_ctrl: Pi,
    }
}

pub mod pi {
    use super::*;

    #[derive(Debug, Clone, Copy, IntoBytes, TryFromBytes, Immutable, Default)]
    pub struct Pi {
        pub config: Config,

        pub target: f32,

        pub p: f32,
        pub i: f32,
        pub ff: f32,

        pub output: f32,
    }

    #[derive(Debug, Clone, Copy, IntoBytes, TryFromBytes, Immutable, Default)]
    pub struct Config {
        pub kp: f32,
        pub ki: f32,
        pub p_ff: f32,
        pub i_max: f32,
        pub output_max: f32,
    }

    impl Pi {
        pub fn new(config: Config) -> Self {
            Self {
                config,
                ..Default::default()
            }
        }

        pub fn update(&mut self, measured: f32, target: f32) -> f32 {
            self.target = target;

            let error = self.target - measured;
            let cfg = &self.config;

            self.p = (error * cfg.kp).clamp(0.0, cfg.output_max);

            let i = self.i + (error * cfg.ki);
            self.i = i.clamp(-cfg.i_max, cfg.i_max);

            self.ff = self.target * cfg.p_ff;

            self.output = (self.p + self.i + self.ff).clamp(0.0, cfg.output_max);
            self.output
        }

        pub fn reset(&mut self) {
            self.target = 0.0;

            self.p = 0.0;
            self.i = 0.0;

            self.output = 0.0;
        }
    }
}
