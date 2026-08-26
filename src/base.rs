use crate::{Error, Frame, Result, Twist};

/// Runtime state of a floating URDF root link.
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
    pub(crate) fn stationary_unchecked(frame: Frame) -> Self {
        Self {
            frame,
            velocity: Twist::zeros(),
            acceleration: Twist::zeros(),
        }
    }

    /// Creates a stationary floating base at a prescribed world pose.
    ///
    /// # Errors
    ///
    /// Returns an error if `frame` contains a non-finite value.
    pub fn stationary(frame: Frame) -> Result<Self> {
        validate_frame(&frame)?;
        Ok(Self::stationary_unchecked(frame))
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

pub(crate) fn validate_frame(frame: &Frame) -> Result<()> {
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
