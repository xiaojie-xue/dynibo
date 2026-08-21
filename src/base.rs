use crate::{Error, Frame, Result, Twist};

/// Whether the URDF root link is fixed to the world or has six free degrees of freedom.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BaseMode {
    /// The root link has a fixed world pose and zero velocity and acceleration.
    #[default]
    Fixed,
    /// The root link has a world pose and an independently prescribed spatial motion.
    /// Its generalized-vector prefix is `[angular_x, angular_y, angular_z,
    /// linear_x, linear_y, linear_z]`, all expressed in the world frame.
    Floating,
}

/// Runtime state of the URDF root link.
///
/// Velocity and acceleration are angular-first classical quantities expressed
/// in the world frame at the root-link origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaseState {
    frame: Frame,
    velocity: Twist,
    acceleration: Twist,
}

impl BaseState {
    /// Creates the zero state of a fixed base at the world origin.
    pub fn fixed() -> Self {
        Self {
            frame: Frame::identity(),
            velocity: Twist::zeros(),
            acceleration: Twist::zeros(),
        }
    }

    /// Creates a stationary fixed base at a prescribed world pose.
    ///
    /// # Errors
    ///
    /// Returns an error if `frame` contains a non-finite value.
    pub fn fixed_at(frame: Frame) -> Result<Self> {
        validate_frame(&frame)?;
        Ok(Self {
            frame,
            velocity: Twist::zeros(),
            acceleration: Twist::zeros(),
        })
    }

    /// Creates a base state expressed in the world frame.
    ///
    /// # Errors
    ///
    /// Returns an error if any component is non-finite.
    pub fn new(frame: Frame, velocity: Twist, acceleration: Twist) -> Result<Self> {
        validate_frame(&frame)?;
        validate_twist(velocity, "velocity")?;
        validate_twist(acceleration, "acceleration")?;
        Ok(Self {
            frame,
            velocity,
            acceleration,
        })
    }

    /// Returns the root-link pose in the world frame.
    pub const fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Returns the root-origin classical velocity expressed in the world frame.
    pub const fn velocity(&self) -> Twist {
        self.velocity
    }

    /// Returns the root-origin classical acceleration expressed in the world frame.
    pub const fn acceleration(&self) -> Twist {
        self.acceleration
    }
}

impl Default for BaseState {
    fn default() -> Self {
        Self::fixed()
    }
}

fn validate_frame(frame: &Frame) -> Result<()> {
    if frame
        .translation
        .vector
        .iter()
        .chain(frame.rotation.coords.iter())
        .all(|value| value.is_finite())
    {
        Ok(())
    } else {
        Err(Error::InvalidBaseState {
            field: "frame",
            reason: "must contain only finite values",
        })
    }
}

fn validate_twist(value: Twist, field: &'static str) -> Result<()> {
    if value
        .angular
        .iter()
        .chain(value.linear.iter())
        .all(|value| value.is_finite())
    {
        Ok(())
    } else {
        Err(Error::InvalidBaseState {
            field,
            reason: "must contain only finite values",
        })
    }
}
