// Math module: Utility functions for vectors, matrices, numerical methods (RK4)

pub trait NumericalSolver {
    fn step(&mut self, dt: f64);
}
