pub trait Integrator {
    fn integrate<F>(
        &self,
        f: F,
        t: f64,
        y: &[f64],
        dt: f64,
    ) -> Vec<f64>
    where
        F: Fn(f64, &[f64]) -> Vec<f64>;
}

pub struct RungeKutta4;

impl Integrator for RungeKutta4 {
    fn integrate<F>(
        &self,
        f: F,
        t: f64,
        y: &[f64],
        dt: f64,
    ) -> Vec<f64>
    where
        F: Fn(f64, &[f64]) -> Vec<f64>,
    {
        let k1 = f(t, y);
        
        let y_k1: Vec<f64> = y.iter().zip(k1.iter()).map(|(yi, k1i)| yi + k1i * dt / 2.0).collect();
        let k2 = f(t + dt / 2.0, &y_k1);
        
        let y_k2: Vec<f64> = y.iter().zip(k2.iter()).map(|(yi, k2i)| yi + k2i * dt / 2.0).collect();
        let k3 = f(t + dt / 2.0, &y_k2);
        
        let y_k3: Vec<f64> = y.iter().zip(k3.iter()).map(|(yi, k3i)| yi + k3i * dt).collect();
        let k4 = f(t + dt, &y_k3);
        
        y.iter()
            .enumerate()
            .map(|(i, yi)| yi + (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * dt / 6.0)
            .collect()
    }
}
