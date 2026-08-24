use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

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

/// Kinematic and state properties of one URDF joint.
#[derive(Clone, Debug)]
pub struct Joint {
    name: String,
    kinematics: JointKinematics,
    lower_limit: f64,
    upper_limit: f64,
    velocity_limit: f64,
    home_offset: f64,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct JointKinematics {
    pub(crate) joint_type: JointType,
    pub(crate) origin: Frame,
    pub(crate) axis: Unit<Vector3<f64>>,
}

impl JointKinematics {
    #[inline]
    pub(crate) fn frame(&self, q: f64) -> Frame {
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
    pub(crate) fn new(
        name: impl Into<String>,
        joint_type: JointType,
        origin: Frame,
        axis: Vector3<f64>,
        lower_limit: f64,
        upper_limit: f64,
        velocity_limit: f64,
    ) -> Result<Self> {
        let name = name.into();
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
            kinematics: JointKinematics {
                joint_type,
                origin,
                axis,
            },
            lower_limit,
            upper_limit,
            velocity_limit,
            home_offset: 0.0,
        })
    }

    /// Returns the joint name loaded from the URDF.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the joint motion type.
    pub const fn joint_type(&self) -> JointType {
        self.kinematics.joint_type
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
        &self.kinematics.origin
    }

    /// Returns the normalized motion axis expressed in the joint frame.
    ///
    /// Fixed joints have no motion axis and return an internal placeholder.
    pub const fn axis(&self) -> &Unit<Vector3<f64>> {
        &self.kinematics.axis
    }

    /// Computes the parent-to-child transform at position `q`.
    pub fn frame(&self, q: f64) -> Frame {
        self.kinematics.frame(q)
    }

    /// Projects a spatial load onto the joint's active degree of freedom.
    ///
    /// Revolute joints return torque and prismatic joints return force. Fixed
    /// joints always return zero.
    pub fn active_force(&self, load: Wrench) -> f64 {
        match self.kinematics.joint_type {
            JointType::Revolute => load.torque.dot(self.kinematics.axis.as_ref()),
            JointType::Prismatic => load.force.dot(self.kinematics.axis.as_ref()),
            JointType::Fixed => 0.0,
        }
    }

    pub(crate) const fn kinematics(&self) -> JointKinematics {
        self.kinematics
    }

    /// Returns whether `q` lies outside the joint position limits.
    pub fn is_over_limit(&self, q: f64) -> bool {
        q > self.upper_limit + 1.0e-12 || q < self.lower_limit - 1.0e-12
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

#[cfg(test)]
#[allow(clippy::approx_constant)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, PI};

    use approx::{assert_abs_diff_eq, assert_relative_eq};
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    use super::{Joint, JointType};
    use crate::{Error, Frame, Wrench};

    fn joint(
        name: &str,
        joint_type: JointType,
        xyz: [f64; 3],
        rpy: [f64; 3],
        axis: [f64; 3],
    ) -> Joint {
        Joint::new(
            name,
            joint_type,
            Isometry3::from_parts(
                Translation3::new(xyz[0], xyz[1], xyz[2]),
                UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
            ),
            Vector3::new(axis[0], axis[1], axis[2]),
            -10.0,
            10.0,
            100.0,
        )
        .unwrap()
    }

    #[test]
    fn preserves_parameters_and_home_offset() {
        let mut joint = Joint::new(
            "joint_1",
            JointType::Revolute,
            Isometry3::from_parts(
                Translation3::new(2.0, 3.0, 4.0),
                UnitQuaternion::from_euler_angles(0.0, 2.0, 1.0),
            ),
            Vector3::z(),
            -3.14,
            3.14,
            100.0,
        )
        .unwrap();

        assert_eq!(joint.name(), "joint_1");
        assert_eq!(joint.joint_type(), JointType::Revolute);
        assert_abs_diff_eq!(joint.lower_limit(), -3.14);
        assert_abs_diff_eq!(joint.upper_limit(), 3.14);
        assert_abs_diff_eq!(joint.velocity_limit(), 100.0);
        assert_abs_diff_eq!(joint.origin().translation.vector.x, 2.0);
        assert_relative_eq!(joint.axis().as_ref(), &Vector3::z(), epsilon = 1.0e-12);
        assert!(joint.is_over_limit(4.0));
        assert!(joint.is_over_limit(-4.0));
        assert!(!joint.is_over_limit(0.0));
        assert_abs_diff_eq!(joint.set_home_offset(-0.25), -0.25);
        assert_abs_diff_eq!(joint.home_offset(), -0.25);
        assert_abs_diff_eq!(
            joint.active_force(Wrench::new(Vector3::new(1.0, 2.0, 3.0), Vector3::zeros())),
            3.0
        );
    }

    #[test]
    fn fixed_joint_ignores_its_axis_but_moving_joints_validate_theirs() {
        let fixed = Joint::new(
            "fixed",
            JointType::Fixed,
            Frame::identity(),
            Vector3::zeros(),
            0.0,
            0.0,
            0.0,
        )
        .expect("a fixed joint does not need a motion axis");
        assert_eq!(fixed.joint_type(), JointType::Fixed);

        for joint_type in [JointType::Revolute, JointType::Prismatic] {
            let error = Joint::new(
                "moving",
                joint_type,
                Frame::identity(),
                Vector3::zeros(),
                -1.0,
                1.0,
                1.0,
            )
            .unwrap_err();
            assert!(matches!(
                error,
                Error::InvalidJointAxis { ref joint } if joint == "moving"
            ));
        }
    }

    #[test]
    fn revolute_and_prismatic_joint_frames_match_urdf_semantics() {
        let revolute = joint(
            "revolute",
            JointType::Revolute,
            [0.0, 0.0, 0.226],
            [0.0, 0.0, FRAC_PI_2],
            [0.0, 0.0, 1.0],
        );
        let frame = revolute.frame(0.3 * PI);
        let expected = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.8 * PI);
        assert_relative_eq!(frame.rotation, expected, epsilon = 1.0e-12);
        assert_abs_diff_eq!(frame.translation.vector.z, 0.226);

        let prismatic = joint(
            "slide",
            JointType::Prismatic,
            [1.0, 0.0, 0.0],
            [0.0; 3],
            [0.0, 1.0, 0.0],
        );
        assert_relative_eq!(
            prismatic.frame(0.25).translation.vector,
            Vector3::new(1.0, 0.25, 0.0),
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            prismatic.active_force(Wrench::new(Vector3::zeros(), Vector3::new(2.0, 3.0, 4.0))),
            3.0
        );

        let fixed = joint(
            "fixed",
            JointType::Fixed,
            [0.1, 0.2, 0.3],
            [0.0; 3],
            [0.0; 3],
        );
        assert_relative_eq!(fixed.frame(123.0), *fixed.origin(), epsilon = 1.0e-12);
        assert_abs_diff_eq!(
            fixed.active_force(Wrench::new(Vector3::z(), Vector3::x())),
            0.0
        );
    }
}
