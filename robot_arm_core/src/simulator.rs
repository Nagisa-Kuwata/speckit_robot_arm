use crate::kinematics::RobotModel;
use crate::controller::ComputedTorqueController;
use crate::dynamics::RobotDynamics;
use crate::integrator::{Integrator, RungeKutta4};
use crate::trajectory::Trajectory;

pub struct SimulatorState {
    pub time: f64,
    pub running: bool,
    pub q: Vec<f64>,
    pub q_dot: Vec<f64>,
    pub model: RobotModel,
    pub dynamics: RobotDynamics,
    pub controller: ComputedTorqueController,
    pub current_trajectory: Option<(Trajectory, f64)>, // (Trajectory, start_time)
}

impl SimulatorState {
    pub fn new(model: RobotModel) -> Self {
        let n = model.dh_params.len();
        Self {
            time: 0.0,
            running: false,
            q: vec![0.0; n],
            q_dot: vec![0.0; n],
            model,
            dynamics: RobotDynamics::new(n),
            controller: ComputedTorqueController::new(100.0, 20.0), // Tuned gains
            current_trajectory: None,
        }
    }

    pub fn set_target(&mut self, target_joints: &[f64], duration: f64) {
        // Create trajectory from current state to target
        // Duration is explicitly passed or calculated.
        // My Trajectory::new takes "avg_velocity"
        // Let's assume duration is passed for now, and I will modify Trajectory to accept duration in a separate constructor if needed,
        // or just calculate "avg_velocity" to match duration.
        // avg_vel = dist / duration.
        
        let max_dist = self.q.iter().zip(target_joints.iter())
            .map(|(a, b)| (b - a).abs())
            .fold(0.0f64, |acc, x| acc.max(x));
            
        let avg_vel = if duration > 0.001 { max_dist / duration } else { 1.0 };
        
        // Ensure minimum velocity to avoid infinite duration if dist is 0
        let avg_vel = avg_vel.max(0.001);

        let traj = Trajectory::new(&self.q, target_joints, avg_vel);
        self.current_trajectory = Some((traj, self.time));
        self.running = true;
    }

    pub fn update(&mut self, dt: f64) {
        // Use local integrator instance to avoid borrow checker issues with self
        let integrator = RungeKutta4;

        if !self.running {
            return;
        }

        let n = self.q.len();
        // Pack state
        let mut y = Vec::with_capacity(2 * n);
        y.extend_from_slice(&self.q);
        y.extend_from_slice(&self.q_dot);

        // Integrate
        // We need to pass a context or closure that has access to self.dynamics, self.model, self.controller
        // but NOT self.q/self.q_dot (passed via y).
        // Since we are inside &mut self method, we can't easily capture &self in closure if we call method on self.
        // But integrator is local. Closure captures &self.
        // Does closure capture &self or &mut self? &self (immutable borrow).
        // `update` holds &mut self.
        // We can re-borrow &self as immutable for the closure?
        // Rust allow: &mut self -> &*self (reborrow).
        // The closure `f` in `integrate` is `Fn`.
        
        let new_y = integrator.integrate(
            |t, y_curr| self.dynamics_derivative(t, y_curr),
            self.time,
            &y,
            dt
        );
        
        // Unpack state
        self.q = new_y[0..n].to_vec();
        self.q_dot = new_y[n..2*n].to_vec();
        self.time += dt;
        
        // Stop if trajectory finished?
        if let Some((traj, start_time)) = &self.current_trajectory {
            if self.time - start_time > traj.duration + 0.5 {
                // trajectories finished
                // self.running = false; // Optional: keep running regulator
            }
        }
    }
    
    fn dynamics_derivative(&self, t: f64, y: &[f64]) -> Vec<f64> {
        let n = self.q.len();
        let q = &y[0..n];
        let q_dot = &y[n..2*n];
        
        // Trajectory sampling
        let (q_des, q_dot_des, q_ddot_des) = if let Some((traj, start_time)) = &self.current_trajectory {
            let traj_time = t - start_time;
            if let Some(sample) = traj.sample(traj_time) {
                sample
            } else {
                 // End of trajectory: hold last position
                 // For now, regulator to current q is bad if moving.
                 // Ideally regulator to final target.
                 // Let's reuse last valid sample or target.
                 // Assuming trajectory handles clamping or returning end state?
                 // My Trajectory::sample returns None if t > duration.
                 // Ideally we should hold the final point.
                 // Let's ask trajectory for end point.
                 // Quick fix: if t > duration, use duration.
                 traj.sample(traj.duration).unwrap_or((q.to_vec(), vec![0.0; n], vec![0.0; n]))
            }
        } else {
            // Regulator to current state (zero velocity) if no trajectory?
            // This effectively brakes.
             (q.to_vec(), vec![0.0; n], vec![0.0; n]) 
        };
        
        // Control
        let tau = self.controller.compute_torque(
            &self.model, &self.dynamics, q, q_dot, &q_des, &q_dot_des, &q_ddot_des
        );
        
        // Forward Dynamics
        let q_ddot = self.dynamics.forward_dynamics(
            &self.model, q, q_dot, &tau
        );
        
        let mut dy = Vec::with_capacity(2 * n);
        dy.extend_from_slice(q_dot);
        dy.extend_from_slice(&q_ddot);
        dy
    }
}
