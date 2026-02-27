use nalgebra::{Vector3, Matrix4, Matrix3, OMatrix, OVector, U6, U7};
use std::f64::consts::PI;
use serde::{Serialize, Deserialize};

pub type Matrix6x7 = OMatrix<f64, U6, U7>;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DHParam {
    pub a: f64,     // Link length
    pub alpha: f64, // Link twist
    pub d: f64,     // Link offset
    pub theta_offset: f64, // Joint angle offset
}

impl DHParam {
    pub fn to_transform(&self, theta: f64) -> Matrix4<f64> {
        let theta = theta + self.theta_offset;
        let c_theta = theta.cos();
        let s_theta = theta.sin();
        let c_alpha = self.alpha.cos();
        let s_alpha = self.alpha.sin();

        Matrix4::new(
            c_theta, -s_theta * c_alpha,  s_theta * s_alpha, self.a * c_theta,
            s_theta,  c_theta * c_alpha, -c_theta * s_alpha, self.a * s_theta,
            0.0,      s_alpha,            c_alpha,           self.d,
            0.0,      0.0,                0.0,               1.0,
        )
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotModel {
    pub dh_params: Vec<DHParam>,
    pub joint_limits: Vec<(f64, f64)>, // (min, max) in radians
}

impl RobotModel {
    pub fn new_sia30d() -> Self {
        // Approximate DH parameters for SIA30D (7 DOF)
        // Structure: S-L-E-U-R-B-T
        // Total Reach approx 1485mm. 
        // We divide this among links.
        // This is a placeholder until exact kinematics are known.
        let dh_params = vec![
            // J1 (S) - Vertical
            DHParam { a: 0.0, alpha: -PI/2.0, d: 0.31, theta_offset: 0.0 },
            // J2 (L) - Horizontal
            DHParam { a: 0.0, alpha: PI/2.0, d: 0.0, theta_offset: 0.0 },
            // J3 (E) - Roll/Vertical
            DHParam { a: 0.08, alpha: -PI/2.0, d: 0.49, theta_offset: 0.0 },
            // J4 (U) - Horizontal
            DHParam { a: 0.0, alpha: PI/2.0, d: 0.0, theta_offset: 0.0 },
            // J5 (R) - Roll
            DHParam { a: 0.08, alpha: -PI/2.0, d: 0.42, theta_offset: 0.0 },
            // J6 (B) - Pitch
            DHParam { a: 0.0, alpha: PI/2.0, d: 0.0, theta_offset: 0.0 },
            // J7 (T) - Roll/Flange
            DHParam { a: 0.0, alpha: 0.0, d: 0.18, theta_offset: 0.0 },
        ];

        let joint_limits = vec![
            (-180.0f64.to_radians(), 180.0f64.to_radians()),
            (-110.0f64.to_radians(), 110.0f64.to_radians()),
            (-170.0f64.to_radians(), 170.0f64.to_radians()),
            (-130.0f64.to_radians(), 130.0f64.to_radians()), // Spec says U is -110 to 110? Check table.
            (-170.0f64.to_radians(), 170.0f64.to_radians()),
            (-110.0f64.to_radians(), 110.0f64.to_radians()),
            (-180.0f64.to_radians(), 180.0f64.to_radians()),
        ];

        Self { dh_params, joint_limits }
    }

    pub fn forward_kinematics(&self, joint_angles: &[f64]) -> Matrix4<f64> {
        let mut transform = Matrix4::identity();
        for (i, &theta) in joint_angles.iter().enumerate() {
            if i < self.dh_params.len() {
                transform = transform * self.dh_params[i].to_transform(theta);
            }
        }
        transform
    }

    pub fn inverse_kinematics_position(&self, target_pos: &Vector3<f64>, initial_guess: &[f64]) -> Option<Vec<f64>> {
        let mut q = OVector::<f64, U7>::from_column_slice(initial_guess);
        let max_iter = 100;
        let tolerance = 1e-4;
        let lambda = 0.5;

        for _ in 0..max_iter {
            let mut transform = Matrix4::identity();
            let mut origins = Vec::new();
            let mut z_axes = Vec::new();
            
            for (i, &theta) in q.iter().enumerate() {
                if i < self.dh_params.len() {
                    z_axes.push(transform.column(2).xyz());
                    origins.push(transform.column(3).xyz());
                    transform = transform * self.dh_params[i].to_transform(theta);
                }
            }
            
            let current_pos = transform.column(3).xyz();
            let error = target_pos - current_pos;

            if error.norm() < tolerance {
                return Some(q.as_slice().to_vec());
            }

            let mut j_pos = OMatrix::<f64, U6, U7>::zeros().fixed_rows::<3>(0).into_owned();
            
            for i in 0..7 {
                 let z_i = z_axes[i];
                 let p_i = origins[i];
                 let j_col = z_i.cross(&(current_pos - p_i));
                 j_pos.set_column(i, &j_col);
            }

            let jt = j_pos.transpose();
            let jj_t = j_pos * jt;
            let damping = Matrix3::identity() * (lambda * lambda);
            
            if let Some(inv) = (jj_t + damping).try_inverse() {
                let delta_q = jt * inv * error;
                q += delta_q;
                
                 for (i, val) in q.iter_mut().enumerate() {
                    let (min, max) = self.joint_limits[i];
                     *val = val.clamp(min, max);
                 }
            } else {
                break;
            }
        }
        
        Some(q.as_slice().to_vec())
    }
}
