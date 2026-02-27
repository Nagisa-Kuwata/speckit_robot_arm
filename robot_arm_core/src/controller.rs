use crate::kinematics::RobotModel;
use crate::dynamics::RobotDynamics;

pub struct ComputedTorqueController {
    pub kp: f64,
    pub kv: f64,
}

impl ComputedTorqueController {
    pub fn new(kp: f64, kv: f64) -> Self {
        Self { kp, kv }
    }

    pub fn compute_torque(
        &self,
        model: &RobotModel,
        dynamics: &RobotDynamics,
        q: &[f64], 
        q_dot: &[f64],
        q_des: &[f64],
        q_dot_des: &[f64],
        q_ddot_des: &[f64],
    ) -> Vec<f64> {
        let n = q.len();
        
        let mut q_ddot_cmd = Vec::with_capacity(n);
        
        for i in 0..n {
            let error_pos = q_des[i] - q[i];
            let error_vel = q_dot_des[i] - q_dot[i];
            
            // commanded accel = q_ddot_des + Kv * e_dot + Kp * e
            let acc = q_ddot_des[i] + self.kv * error_vel + self.kp * error_pos;
            q_ddot_cmd.push(acc);
        }
        
        // This computes M(q)*q_ddot_cmd + h(q, q_dot) = tau
        dynamics.inverse_dynamics(model, q, q_dot, &q_ddot_cmd)
    }
}
