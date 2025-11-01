#[derive(Debug, Copy, Clone)]
pub struct Pi {
    pub target: f32,

    pub kp: f32,
    pub ki: f32,
    pub p_ff: f32,

    pub p: f32,
    pub i: f32,
    pub ff: f32,

    pub i_max: f32,
    pub output_max: f32,

    pub output: f32,
}

impl Pi {
    pub fn update(&mut self, measured: f32, target: f32) -> f32 {
        self.target = target;

        let error = self.target - measured;

        self.p = (error * self.kp).clamp(0.0, self.output_max);

        let i = self.i + (error * self.ki);
        self.i = i.clamp(-(self.i_max * self.i_max), self.i_max);

        self.ff = self.target * self.p_ff;

        self.output = (self.p + self.i + self.ff).clamp(0.0, self.output_max);
        self.output
    }

    pub fn reset(&mut self) {
        self.target = 0.0;

        self.p = 0.0;
        self.i = 0.0;

        self.output = 0.0;
    }
}
