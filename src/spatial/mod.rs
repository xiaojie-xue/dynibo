pub(crate) mod cross;
pub(crate) mod inertia;
pub(crate) mod transform;

use std::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

use nalgebra::{Isometry3, SVector, Vector3};

/// A rigid-body transform in three-dimensional space.
pub type Frame = Isometry3<f64>;

/// Angular-first spatial twist vector `[angular_x, angular_y, angular_z,
/// linear_x, linear_y, linear_z]`.
///
/// The coordinate frame is selected by the API that produces or consumes the
/// value; kinematics outputs use the world frame.
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

    /// Checks whether every angular and linear component is finite.
    pub fn is_finite(&self) -> bool {
        self.angular
            .iter()
            .chain(self.linear.iter())
            .all(|v| v.is_finite())
    }

    /// Converts this twist to an angular-first six-dimensional vector.
    pub fn to_vector(self) -> SVector<f64, 6> {
        SVector::from_iterator(self.angular.iter().chain(self.linear.iter()).copied())
    }

    pub(crate) fn from_vector(value: SVector<f64, 6>) -> Self {
        Self::new(
            Vector3::new(value[0], value[1], value[2]),
            Vector3::new(value[3], value[4], value[5]),
        )
    }

    #[inline]
    pub(crate) fn dot_wrench(self, force: Wrench) -> f64 {
        self.angular.dot(&force.torque) + self.linear.dot(&force.force)
    }
}

impl Add for Twist {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.angular + rhs.angular, self.linear + rhs.linear)
    }
}

impl AddAssign for Twist {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.angular += rhs.angular;
        self.linear += rhs.linear;
    }
}

impl Sub for Twist {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.angular - rhs.angular, self.linear - rhs.linear)
    }
}

impl SubAssign for Twist {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.angular -= rhs.angular;
        self.linear -= rhs.linear;
    }
}

impl Neg for Twist {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.angular, -self.linear)
    }
}

impl Mul<f64> for Twist {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.angular * rhs, self.linear * rhs)
    }
}

impl Mul<Twist> for f64 {
    type Output = Twist;

    #[inline]
    fn mul(self, rhs: Twist) -> Twist {
        Twist::new(self * rhs.angular, self * rhs.linear)
    }
}

/// Torque-first spatial force vector `[torque_x, torque_y, torque_z, force_x,
/// force_y, force_z]`.
///
/// Load APIs interpret a resisting wrench in its associated link frame at that
/// link origin; floating-base generalized-force outputs use the world frame at
/// the root origin.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Wrench {
    /// Torque component of the wrench.
    pub torque: Vector3<f64>,
    /// Force component of the wrench.
    pub force: Vector3<f64>,
}

impl Wrench {
    /// Checks whether every torque and force component is finite.
    pub fn is_finite(&self) -> bool {
        self.torque
            .iter()
            .chain(self.force.iter())
            .all(|v| v.is_finite())
    }

    /// Creates a spatial wrench from its torque and force components.
    pub const fn new(torque: Vector3<f64>, force: Vector3<f64>) -> Self {
        Self { torque, force }
    }

    /// Returns a wrench whose components are all zero.
    pub fn zeros() -> Self {
        Self::default()
    }

    /// Converts this wrench to a torque-first six-dimensional vector.
    pub fn to_vector(self) -> SVector<f64, 6> {
        SVector::from_iterator(self.torque.iter().chain(self.force.iter()).copied())
    }

    pub(crate) fn from_vector(value: SVector<f64, 6>) -> Self {
        Self::new(
            Vector3::new(value[0], value[1], value[2]),
            Vector3::new(value[3], value[4], value[5]),
        )
    }
}

impl Add for Wrench {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.torque + rhs.torque, self.force + rhs.force)
    }
}

impl AddAssign for Wrench {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.torque += rhs.torque;
        self.force += rhs.force;
    }
}

impl Sub for Wrench {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.torque - rhs.torque, self.force - rhs.force)
    }
}

impl SubAssign for Wrench {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.torque -= rhs.torque;
        self.force -= rhs.force;
    }
}

impl Neg for Wrench {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.torque, -self.force)
    }
}

impl Mul<f64> for Wrench {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.torque * rhs, self.force * rhs)
    }
}

impl Mul<Wrench> for f64 {
    type Output = Wrench;

    #[inline]
    fn mul(self, rhs: Wrench) -> Wrench {
        Wrench::new(self * rhs.torque, self * rhs.force)
    }
}
