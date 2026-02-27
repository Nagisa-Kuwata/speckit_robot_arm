// Core simulator logic (abstract from rendering)

pub struct SimulatorState {
    pub time: f64,
    pub running: bool,
}

impl SimulatorState {
    pub fn new() -> Self {
        Self {
            time: 0.0,
            running: false,
        }
    }

    pub fn update(&mut self, dt: f64) {
        if self.running {
            self.time += dt;
            // Physics update would go here
        }
    }
}
