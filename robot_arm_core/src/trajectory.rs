pub struct QuinticPolynomial {
    coefs: [f64; 6],
}

impl QuinticPolynomial {
    pub fn new(start: f64, end: f64, duration: f64) -> Self {
        let t = duration;
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;
        
        // Initial boundary conditions: q0 = start, v0 = 0, a0 = 0
        let a0 = start;
        let a1 = 0.0;
        let a2 = 0.0;
        
        // Final boundary conditions: qT = end, vT = 0, aT = 0
        // h = end - start
        let h = end - start;
        
        // Solving the system for v0=0, a0=0, vT=0, aT=0
        // a3 = 10h / T^3
        // a4 = -15h / T^4
        // a5 = 6h / T^5
        
        let a3 = 10.0 * h / t3;
        let a4 = -15.0 * h / t4;
        let a5 = 6.0 * h / t5;
        
        Self { coefs: [a0, a1, a2, a3, a4, a5] }
    }

    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;
        
        let pos = self.coefs[0] + self.coefs[1]*t + self.coefs[2]*t2 + 
                 self.coefs[3]*t3 + self.coefs[4]*t4 + self.coefs[5]*t5;
                 
        let vel = self.coefs[1] + 2.0*self.coefs[2]*t + 3.0*self.coefs[3]*t2 + 
                 4.0*self.coefs[4]*t3 + 5.0*self.coefs[5]*t4;
                 
        let acc = 2.0*self.coefs[2] + 6.0*self.coefs[3]*t + 
                 12.0*self.coefs[4]*t2 + 20.0*self.coefs[5]*t3;
                 
        (pos, vel, acc)
    }
}

pub struct Trajectory {
    polynomials: Vec<QuinticPolynomial>,
    pub duration: f64,
}

impl Trajectory {
    pub fn new(start_joints: &[f64], end_joints: &[f64], avg_velocity: f64) -> Self {
        // Estimate duration based on max displacement
        let max_diff = start_joints.iter().zip(end_joints.iter())
            .map(|(s, e)| (e - s).abs())
            .fold(0.0f64, |a, b| a.max(b));
            
        let duration = (max_diff / avg_velocity).max(1.0); // Minimum 1 second
        
        let polynomials = start_joints.iter().zip(end_joints.iter())
            .map(|(s, e)| QuinticPolynomial::new(*s, *e, duration))
            .collect();
            
        Self { polynomials, duration }
    }
    
    pub fn sample(&self, time: f64) -> Option<(Vec<f64>, Vec<f64>, Vec<f64>)> {
        if time > self.duration {
            return None;
        }
        
        let mut q = Vec::with_capacity(self.polynomials.len());
        let mut q_dot = Vec::with_capacity(self.polynomials.len());
        let mut q_ddot = Vec::with_capacity(self.polynomials.len());
        
        for poly in &self.polynomials {
            let (p, v, a) = poly.sample(time);
            q.push(p);
            q_dot.push(v);
            q_ddot.push(a);
        }
        
        Some((q, q_dot, q_ddot))
    }
}
