use nalgebra::{Isometry3, SVector, Vector3};

/// A rigid-body transform in three-dimensional space.
pub type Frame = Isometry3<f64>;
/// Angular-first spatial twist vector `[angular, linear]`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Twist {
    /// Angular velocity or angular acceleration component.
    pub angular: Vector3<f64>,
    /// Linear velocity or linear acceleration component.
    pub linear: Vector3<f64>,
}

impl Twist {
    /// Creates a spatial twist from its angular and linear components.
    pub const fn new(angular: Vector3<f64>, linear: Vector3<f64>) -> Self {
        Self { angular, linear }
    }

    /// Returns a spatial twist whose components are all zero.
    pub fn zeros() -> Self {
        Self::default()
    }

    /// Converts this twist to an angular-first six-dimensional vector.
    pub fn to_vector(self) -> SVector<f64, 6> {
        SVector::from_iterator(self.angular.iter().chain(self.linear.iter()).copied())
    }
}

/// Torque-first spatial force vector `[torque, force]`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Wrench {
    /// Torque component of the wrench.
    pub torque: Vector3<f64>,
    /// Force component of the wrench.
    pub force: Vector3<f64>,
}

impl Wrench {
    /// Creates a spatial wrench from its torque and force components.
    pub const fn new(torque: Vector3<f64>, force: Vector3<f64>) -> Self {
        Self { torque, force }
    }

    /// Returns a wrench whose components are all zero.
    pub fn zeros() -> Self {
        Self::default()
    }
}
