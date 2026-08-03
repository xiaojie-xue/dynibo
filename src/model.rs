use nalgebra::{Isometry3, Matrix3, Translation3, Unit, UnitQuaternion, Vector3};

use crate::{Error, Frame, Result, Wrench};

/// Joint motion supported by `Robot`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointType {
    /// Rotation about the joint axis.
    Revolute,
    /// Translation along the joint axis.
    Prismatic,
    /// A rigid connection with no degree of freedom.
    Fixed,
}

/// Identity and immutable inertial properties of one URDF link.
#[derive(Clone, Debug, PartialEq)]
pub struct Link {
    name: String,
    mass: f64,
    center_of_mass: Vector3<f64>,
    inertia: Matrix3<f64>,
}

impl Link {
    /// Creates a link from its immutable inertial properties.
    pub(crate) fn new(
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

    /// Returns the link name loaded from the URDF.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the link mass in kilograms.
    pub const fn mass(&self) -> f64 {
        self.mass
    }

    /// Returns the center of mass expressed in the link frame, in metres.
    pub const fn center_of_mass(&self) -> &Vector3<f64> {
        &self.center_of_mass
    }

    /// Returns the rotational inertia about the center of mass in the link frame.
    pub const fn inertia(&self) -> &Matrix3<f64> {
        &self.inertia
    }
}

/// Kinematic and state properties of one URDF joint.
#[derive(Clone, Debug)]
pub struct Joint {
    name: String,
    joint_type: JointType,
    origin: Frame,
    axis: Unit<Vector3<f64>>,
    lower_limit: f64,
    upper_limit: f64,
    velocity_limit: f64,
    position: f64,
    velocity: f64,
    acceleration: f64,
    home_offset: f64,
}

impl Joint {
    /// Creates a joint after normalizing and validating its motion axis.
    ///
    /// Fixed joints ignore `axis` because they have no degree of freedom.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidJointAxis`] when a revolute or prismatic joint's
    /// `axis` is too small to normalize.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: impl Into<String>,
        joint_type: JointType,
        origin: Frame,
        axis: Vector3<f64>,
        lower_limit: f64,
        upper_limit: f64,
        velocity_limit: f64,
    ) -> Result<Self> {
        Self::new_named(
            name.into(),
            joint_type,
            origin,
            axis,
            lower_limit,
            upper_limit,
            velocity_limit,
        )
    }

    /// Creates a joint when ownership of the joint name is already available.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new_named(
        name: String,
        joint_type: JointType,
        origin: Frame,
        axis: Vector3<f64>,
        lower_limit: f64,
        upper_limit: f64,
        velocity_limit: f64,
    ) -> Result<Self> {
        let axis = match joint_type {
            JointType::Revolute | JointType::Prismatic => {
                Unit::try_new(axis, 1.0e-12).ok_or_else(|| Error::InvalidJointAxis {
                    joint: name.clone(),
                })?
            }
            JointType::Fixed => Vector3::x_axis(),
        };
        Ok(Self {
            name,
            joint_type,
            origin,
            axis,
            lower_limit,
            upper_limit,
            velocity_limit,
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
            home_offset: 0.0,
        })
    }

    /// Returns the joint name loaded from the URDF.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the joint motion type.
    pub const fn joint_type(&self) -> JointType {
        self.joint_type
    }

    /// Returns the minimum joint position, in radians or metres.
    pub const fn lower_limit(&self) -> f64 {
        self.lower_limit
    }

    /// Returns the maximum joint position, in radians or metres.
    pub const fn upper_limit(&self) -> f64 {
        self.upper_limit
    }

    /// Returns the maximum absolute velocity, in radians or metres per second.
    pub const fn velocity_limit(&self) -> f64 {
        self.velocity_limit
    }

    /// Returns the fixed transform from the parent link to the joint.
    pub const fn origin(&self) -> &Frame {
        &self.origin
    }

    /// Returns the normalized motion axis expressed in the joint frame.
    ///
    /// Fixed joints have no motion axis and return an internal placeholder.
    pub const fn axis(&self) -> &Unit<Vector3<f64>> {
        &self.axis
    }

    /// Computes the parent-to-child transform at position `q`.
    pub fn frame(&self, q: f64) -> Frame {
        match self.joint_type {
            JointType::Revolute => {
                self.origin
                    * Isometry3::from_parts(
                        Translation3::identity(),
                        UnitQuaternion::from_axis_angle(&self.axis, q),
                    )
            }
            JointType::Prismatic => self.origin * Translation3::from(self.axis.as_ref() * q),
            JointType::Fixed => self.origin,
        }
    }

    /// Projects a spatial load onto the joint's active degree of freedom.
    ///
    /// Revolute joints return torque and prismatic joints return force. Fixed
    /// joints always return zero.
    pub fn active_force(&self, load: Wrench) -> f64 {
        match self.joint_type {
            JointType::Revolute => load.torque.dot(self.axis.as_ref()),
            JointType::Prismatic => load.force.dot(self.axis.as_ref()),
            JointType::Fixed => 0.0,
        }
    }

    /// Returns whether `q` lies outside the joint position limits.
    pub fn is_over_limit(&self, q: f64) -> bool {
        q > self.upper_limit + 1.0e-12 || q < self.lower_limit - 1.0e-12
    }

    /// Returns the stored joint position.
    pub const fn position(&self) -> f64 {
        self.position
    }

    /// Stores a position clamped to the joint limits and returns that value.
    pub fn set_position(&mut self, position: f64) -> f64 {
        self.position = position.clamp(self.lower_limit, self.upper_limit);
        self.position
    }

    /// Returns the stored joint velocity.
    pub const fn velocity(&self) -> f64 {
        self.velocity
    }

    /// Stores a velocity clamped to the symmetric velocity limit and returns it.
    pub fn set_velocity(&mut self, velocity: f64) -> f64 {
        self.velocity = velocity.clamp(-self.velocity_limit, self.velocity_limit);
        self.velocity
    }

    /// Returns the stored joint acceleration.
    pub const fn acceleration(&self) -> f64 {
        self.acceleration
    }

    /// Stores a joint acceleration and returns it.
    pub fn set_acceleration(&mut self, acceleration: f64) -> f64 {
        self.acceleration = acceleration;
        acceleration
    }

    /// Returns the stored home-position offset.
    pub const fn home_offset(&self) -> f64 {
        self.home_offset
    }

    /// Stores a home-position offset and returns it.
    pub fn set_home_offset(&mut self, offset: f64) -> f64 {
        self.home_offset = offset;
        offset
    }
}
