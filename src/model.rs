use nalgebra::{Isometry3, Matrix3, Translation3, Unit, UnitQuaternion, Vector3};

use crate::{Error, Frame, Result, Wrench};

/// Joint motion supported by `RobotArm`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Prismatic,
    Fixed,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimit {
    pub lower: f64,
    pub upper: f64,
    pub velocity: f64,
}

impl Default for JointLimit {
    fn default() -> Self {
        Self {
            lower: f64::NEG_INFINITY,
            upper: f64::INFINITY,
            velocity: f64::INFINITY,
        }
    }
}

/// Inertial properties of one URDF link.
#[derive(Clone, Debug)]
pub struct RobotLink {
    name: String,
    mass: f64,
    center_of_mass: Vector3<f64>,
    inertia: Matrix3<f64>,
}

impl RobotLink {
    pub fn new(
        name: impl Into<String>,
        mass: f64,
        center_of_mass: Vector3<f64>,
        inertia: Matrix3<f64>,
    ) -> Self {
        Self {
            name: name.into(),
            mass,
            center_of_mass,
            inertia,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub const fn mass(&self) -> f64 {
        self.mass
    }

    pub fn set_mass(&mut self, mass: f64) {
        self.mass = mass;
    }

    pub const fn center_of_mass(&self) -> &Vector3<f64> {
        &self.center_of_mass
    }

    pub fn set_center_of_mass(&mut self, center_of_mass: Vector3<f64>) {
        self.center_of_mass = center_of_mass;
    }

    pub const fn inertia(&self) -> &Matrix3<f64> {
        &self.inertia
    }
}

/// Kinematic and state properties of one URDF joint.
#[derive(Clone, Debug)]
pub struct RobotJoint {
    name: String,
    kind: JointKind,
    origin: Frame,
    axis: Unit<Vector3<f64>>,
    limit: JointLimit,
    position: f64,
    velocity: f64,
    acceleration: f64,
    home_offset: f64,
}

impl RobotJoint {
    pub fn new(
        name: impl Into<String>,
        kind: JointKind,
        origin: Frame,
        axis: Vector3<f64>,
        limit: JointLimit,
    ) -> Result<Self> {
        Self::new_named(name.into(), kind, origin, axis, limit)
    }

    pub(crate) fn new_named(
        name: String,
        kind: JointKind,
        origin: Frame,
        axis: Vector3<f64>,
        limit: JointLimit,
    ) -> Result<Self> {
        let axis = Unit::try_new(axis, 1.0e-12).ok_or_else(|| Error::InvalidJointAxis {
            joint: name.clone(),
        })?;
        Ok(Self {
            name,
            kind,
            origin,
            axis,
            limit,
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
            home_offset: 0.0,
        })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub const fn kind(&self) -> JointKind {
        self.kind
    }

    pub const fn limit(&self) -> JointLimit {
        self.limit
    }

    pub const fn origin(&self) -> &Frame {
        &self.origin
    }

    pub const fn axis(&self) -> &Unit<Vector3<f64>> {
        &self.axis
    }

    pub fn frame(&self, q: f64) -> Frame {
        match self.kind {
            JointKind::Revolute => {
                self.origin
                    * Isometry3::from_parts(
                        Translation3::identity(),
                        UnitQuaternion::from_axis_angle(&self.axis, q),
                    )
            }
            JointKind::Prismatic => self.origin * Translation3::from(self.axis.as_ref() * q),
            JointKind::Fixed => self.origin,
        }
    }

    pub fn active_force(&self, load: Wrench) -> f64 {
        match self.kind {
            JointKind::Revolute => load.torque.dot(self.axis.as_ref()),
            JointKind::Prismatic => load.force.dot(self.axis.as_ref()),
            JointKind::Fixed => 0.0,
        }
    }

    pub fn is_over_limit(&self, q: f64) -> bool {
        q > self.limit.upper + 1.0e-12 || q < self.limit.lower - 1.0e-12
    }

    pub const fn position(&self) -> f64 {
        self.position
    }

    pub fn set_position(&mut self, position: f64) -> f64 {
        self.position = position.clamp(self.limit.lower, self.limit.upper);
        self.position
    }

    pub const fn velocity(&self) -> f64 {
        self.velocity
    }

    pub fn set_velocity(&mut self, velocity: f64) -> f64 {
        self.velocity = velocity.clamp(-self.limit.velocity, self.limit.velocity);
        self.velocity
    }

    pub const fn acceleration(&self) -> f64 {
        self.acceleration
    }

    pub fn set_acceleration(&mut self, acceleration: f64) -> f64 {
        self.acceleration = acceleration;
        acceleration
    }

    pub const fn home_offset(&self) -> f64 {
        self.home_offset
    }

    pub fn set_home_offset(&mut self, offset: f64) -> f64 {
        self.home_offset = offset;
        offset
    }
}
