// Dynamics module: Force, torque, gravity, friction calculation
pub struct DynamicsParameters {
    pub gravity: f64,
    pub friction_coefficients: Vec<f64>,
}

impl Default for DynamicsParameters {
    fn default() -> Self {
        Self {
            gravity: 9.81,
            friction_coefficients: vec![0.1; 7],
        }
    }
}
