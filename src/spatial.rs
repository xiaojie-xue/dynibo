use nalgebra::{Isometry3, SMatrix, SVector, Vector3};

pub type Frame = Isometry3<f64>;
pub type JointVector<const N: usize> = SVector<f64, N>;
pub type Jacobian<const N: usize> = SMatrix<f64, 6, N>;

/// Angular-first spatial motion vector `[angular, linear]`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Motion {
    pub angular: Vector3<f64>,
    pub linear: Vector3<f64>,
}

impl Motion {
    pub const fn new(angular: Vector3<f64>, linear: Vector3<f64>) -> Self {
        Self { angular, linear }
    }

    pub fn zeros() -> Self {
        Self::default()
    }

    pub fn to_vector(self) -> SVector<f64, 6> {
        SVector::from_iterator(self.angular.iter().chain(self.linear.iter()).copied())
    }
}

/// Torque-first spatial force vector `[torque, force]`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Wrench {
    pub torque: Vector3<f64>,
    pub force: Vector3<f64>,
}

impl Wrench {
    pub const fn new(torque: Vector3<f64>, force: Vector3<f64>) -> Self {
        Self { torque, force }
    }

    pub fn zeros() -> Self {
        Self::default()
    }
}
