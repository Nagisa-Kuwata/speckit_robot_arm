use crate::kinematics::RobotModel;
use nalgebra::Matrix4;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CollisionType {
    None,
    SelfCollision,
    GroundCollision,
}

pub struct CollisionDetector {
    pub ground_z: f64,
    pub min_link_distance: f64, // Threshold for self-collision
}

impl Default for CollisionDetector {
    fn default() -> Self {
        Self {
            ground_z: 0.0,
            min_link_distance: 0.05, // 5cm radius approx check
        }
    }
}

impl CollisionDetector {
    pub fn new(ground_z: f64, min_link_distance: f64) -> Self {
        Self { ground_z, min_link_distance }
    }

    pub fn check_collision(&self, model: &RobotModel, joint_angles: &[f64]) -> CollisionType {
        // 1. Calculate positions of all link origins (joints)
        let mut origins = Vec::new();
        let mut transform = Matrix4::<f64>::identity();
        
        // Base is at zero-ish (relative frame)
        origins.push(transform.column(3).xyz());

        for (i, &theta) in joint_angles.iter().enumerate() {
            if i < model.dh_params.len() {
                transform = transform * model.dh_params[i].to_transform(theta);
                origins.push(transform.column(3).xyz());
            }
        }

        // 2. Check Ground Collision (Ground is at Z=0 plane typically, but allow buffer)
        for p in &origins {
            if p.z < self.ground_z {
                 return CollisionType::GroundCollision;
            }
        }

        // 3. Check Self Collision
        // Check distance between non-adjacent link origins.
        // Adjacent links (i, i+1) are connected.
        for i in 0..origins.len() {
            for j in (i + 2)..origins.len() {
                let p1 = origins[i];
                let p2 = origins[j];
                let dist = (p1 - p2).norm();
                
                // Very simple sphere-sphere check at joint origins
                // Real implementation needs capsule-capsule check for full link body.
                // For now, origin-distance is a proxy.
                if dist < self.min_link_distance {
                     return CollisionType::SelfCollision;
                }
            }
        }

        CollisionType::None
    }
}
