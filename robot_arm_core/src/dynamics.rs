use nalgebra::{Vector3, Matrix3};
use crate::kinematics::RobotModel;

#[derive(Clone, Debug)]
pub struct LinkDynamics {
    pub mass: f64,
    pub com: Vector3<f64>, // Center of mass in link frame
    pub inertia: Matrix3<f64>, // Inertia tensor at COM
    pub friction_viscous: f64,
    pub friction_static: f64,
}

impl Default for LinkDynamics {
    fn default() -> Self {
        Self {
            mass: 1.0,
            com: Vector3::new(0.0, 0.0, 0.1), // Offset slightly to avoid singular inertia if 0
            inertia: Matrix3::identity() * 0.01,
            friction_viscous: 0.1,
            friction_static: 0.1,
        }
    }
}

pub struct RobotDynamics {
    pub links: Vec<LinkDynamics>,
    pub gravity: Vector3<f64>,
}

impl RobotDynamics {
    pub fn new(num_links: usize) -> Self {
        Self {
            links: vec![LinkDynamics::default(); num_links],
            gravity: Vector3::new(0.0, 0.0, -9.81),
        }
    }

    pub fn inverse_dynamics(
        &self,
        model: &RobotModel,
        q: &[f64],
        q_dot: &[f64],
        q_ddot: &[f64],
    ) -> Vec<f64> {
        let n = self.links.len(); // Should match model.dh_params.len()
        let mut tau = vec![0.0; n];

        // Kinematics storage (velocities, accelerations in link frame)
        let mut omegas = vec![Vector3::zeros(); n + 1]; // omega_i
        let mut omega_dots = vec![Vector3::zeros(); n + 1]; // dot{omega}_i
        let mut v_dots = vec![Vector3::zeros(); n + 1]; // dot{v}_i (linear acceleration of frame origin)
        // Base initialization (link 0 is base, index 0 in loop is link 1)
        // Base is static, but has gravity acceleration upwards (or standard gravity downwards)
        // RNEA method usually initializes v_dot_0 = -g to simulate gravity
        v_dots[0] = -self.gravity; // Gravity vector in base frame. e.g. (0,0,-9.81) -> -(-9.81) = +9.81 upwards

        // Forward Recursion
        // Frame i is attached to link i. Joint i connects link i-1 to link i.
        // We iterate i from 0 to n-1 (representing joint 1 to n)
        // Using index i+1 for storage corresponding to link i+1 (1-based)
        
        // Transforms 
        // T_{i-1, i} is computed from DH params.
        // R_{i-1, i} is rotation.
        // P_{i-1, i} is translation.
        
        let z_axis = Vector3::z(); // Rotation axis for accumulated revolute joints in DH frame logic is Z

        for i in 0..n {
            let theta = q[i];
            let d_theta = q_dot[i];
            let dd_theta = q_ddot[i];
            
            // Transform from {i-1} to {i}
            let transform = model.dh_params[i].to_transform(theta);
            let rotation = transform.fixed_view::<3, 3>(0, 0).into_owned();
            let position = transform.column(3).xyz(); // P_{i-1, i} relative to {i-1} expressed in {i-1}
            let rotation_t = rotation.transpose(); // R_i^{i-1} (from i-1 to i) needs Inverse for vector projection

            // Correct RNEA uses R_i^{i-1} to project vectors from {i-1} to {i}
            // omega_i = R^T * omega_{i-1} + z * d_theta
            
            omegas[i+1] = rotation_t * omegas[i] + z_axis * d_theta;
            
            // omega_dot_i = R^T * omega_dot_{i-1} + (R^T * omega_{i-1}) x (z * d_theta) + z * dd_theta
            let omega_prev_in_i = rotation_t * omegas[i];
            omega_dots[i+1] = rotation_t * omega_dots[i] 
                            + omega_prev_in_i.cross(&(z_axis * d_theta)) 
                            + z_axis * dd_theta;
            
            // v_dot_i = R^T * (v_dot_{i-1} + omega_dot_{i-1} x P + omega_{i-1} x (omega_{i-1} x P))
            // Cross product order: a x b
            let omega_cross_p = omegas[i].cross(&position);
            let term = omega_dots[i].cross(&position) + omegas[i].cross(&omega_cross_p);
            v_dots[i+1] = rotation_t * (v_dots[i] + term);
        }

        // Backward Recursion
        // Compute forces and moments at COM, then propagate back
        
        let mut f_next = Vector3::zeros(); // Force from link i+1 on i
        let mut n_next = Vector3::zeros(); // Moment from link i+1 on i

        for i in (0..n).rev() {
            let link = &self.links[i];
            // Acceleration of COM
            // v_dot_ci = v_dot_i + omega_dot_i x r_ci + omega_i x (omega_i x r_ci)
            let r_ci = link.com; // Center of mass in frame i
            let omega = omegas[i+1];
            let omega_dot = omega_dots[i+1];
            let v_dot = v_dots[i+1];
            
            let omega_cross_r = omega.cross(&r_ci);
            let v_dot_ci = v_dot + omega_dot.cross(&r_ci) + omega.cross(&omega_cross_r);
            
            // Inertial force/torque
            let f_inertial = v_dot_ci * link.mass; // F = ma
            let n_inertial = link.inertia * omega_dot + omega.cross(&(link.inertia * omega)); // Euler eq
            
            // Force balance
            // f_i = R_{i+1}^i * f_{i+1} + F_i
            // We need rotation from {i+1} to {i}.
            // In forward pass we had R_{i-1, i}.
            // Here 'n_next' and 'f_next' are already in frame {i} if we transform them at end of loop.
            // Wait, previous iteration (i+1) computed f_i+1 (force exerted on i+1 by i).
            // This force acts on i as -f_{i+1}. 
            // My variables `f_next` store force exerted by link i+1 on link i, expressed in frame i?
            // Standard RNEA:
            // f_i = R^{i+1}_i f_{i+1} + F_i
            // n_i = N_i + R^{i+1}_i n_{i+1} + r_{ci} x F_i + P_{i+1}^i x (R^{i+1}_i f_{i+1})
            
            // Let's declare f_i, n_i as force/moment of joint i on link i.
            // f_next is force of joint i+1 on link i+1, expressed in frame i+1.
            // We need to rotate it to frame i.
            
            let f_next_in_i: Vector3<f64>;
            let n_next_in_i: Vector3<f64>;
            let p_next_in_i: Vector3<f64>;
            
            if i == n - 1 {
                f_next_in_i = Vector3::zeros(); // End effector force (external)
                n_next_in_i = Vector3::zeros();
                p_next_in_i = Vector3::zeros();
            } else {
                // Get transformation from i to i+1
                let transform = model.dh_params[i+1].to_transform(q[i+1]);
                let rotation = transform.fixed_view::<3, 3>(0, 0).into_owned(); // R_i+1^i
                let position = transform.column(3).xyz(); // P_{i, i+1} expressed in {i}
                
                f_next_in_i = rotation * f_next;
                n_next_in_i = rotation * n_next;
                p_next_in_i = position;
            }
            
            let f_i = f_inertial + f_next_in_i;
            let n_i = n_inertial + n_next_in_i + r_ci.cross(&f_inertial) + p_next_in_i.cross(&f_next_in_i);
            
            // Project to joint axis (Z) for torque
            // tau_i = n_i . z_i
            // Friction
            let friction = link.friction_viscous * q_dot[i] 
                         + link.friction_static * q_dot[i].signum();
                         
            tau[i] = n_i.z + friction;
            
            // Update f_next, n_next for next iteration (which is i-1)
            f_next = f_i;
            n_next = n_i;
        }

        tau
    }

    pub fn forward_dynamics(
        &self,
        model: &RobotModel,
        q: &[f64],
        q_dot: &[f64],
        tau: &[f64],
    ) -> Vec<f64> {
        let n = q.len();
        let zeros = vec![0.0; n];
        
        // 1. Calculate h(q, q_dot) (Corriolis + Centrifugal + Gravity) assuming q_ddot = 0
        // h = ID(q, q_dot, 0)
        let h = self.inverse_dynamics(model, q, q_dot, &zeros);
        
        // 2. Calculate g(q) (Gravity only) assuming q_dot=0, q_ddot=0
        // g = ID(q, 0, 0)
        // Needed to isolate M columns from ID(q, 0, e_i) resultant which is M_i + g
        let g_force = self.inverse_dynamics(model, q, &zeros, &zeros);
        
        let mut m_mat = nalgebra::DMatrix::zeros(n, n);
        
        for i in 0..n {
             let mut e_i = vec![0.0; n];
             e_i[i] = 1.0;
             
             // term = M * e_i + g
             let term = self.inverse_dynamics(model, q, &zeros, &e_i);
             
             // M_col_i = term - g
             for r in 0..n {
                 m_mat[(r, i)] = term[r] - g_force[r];
             }
        }
        
        // 3. Solve M * q_ddot + h = tau  =>  M * q_ddot = tau - h
        let mut rhs = nalgebra::DVector::zeros(n);
        for i in 0..n {
            rhs[i] = tau[i] - h[i];
        }
        
        // Solve linear system
        if let Some(solution) = m_mat.lu().solve(&rhs) {
             solution.as_slice().to_vec()
        } else {
             // Fallback if M is singular (should rarely happen for robot arms)
             vec![0.0; n]
        }
    }
}
