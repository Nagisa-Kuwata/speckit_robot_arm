use nalgebra::{OVector, Dim, DefaultAllocator};

// Using f64 as scalar type for simplicity as per requirement.
// Generic state size D.

pub trait OdeModel<D: Dim> 
where DefaultAllocator: nalgebra::allocator::Allocator<D> {
    // Computes dy/dt given state y and time t
    fn derivative(&self, state: &OVector<f64, D>, t: f64) -> OVector<f64, D>;
}

pub struct RungeKutta4;

impl RungeKutta4 {
    pub fn step<D, M>(model: &M, state: &OVector<f64, D>, t: f64, dt: f64) -> OVector<f64, D>
    where 
        D: Dim,
        M: OdeModel<D>,
        DefaultAllocator: nalgebra::allocator::Allocator<D>,
    {
        let k1 = model.derivative(state, t);
        let k2 = model.derivative(&(state + &k1 * (dt * 0.5)), t + dt * 0.5);
        let k3 = model.derivative(&(state + &k2 * (dt * 0.5)), t + dt * 0.5);
        let k4 = model.derivative(&(state + &k3 * dt), t + dt);

        state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0)
    }
}
