use crate::{Error, Frame, Result, Twist};

/// Whether the URDF root link is fixed to the world or has six free degrees of freedom.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BaseMode {
    /// The root link has a fixed world pose and zero velocity and acceleration.
    #[default]
    Fixed,
    /// The root link has a world pose and an independently prescribed spatial motion.
    Floating,
}

/// Runtime state of the URDF root link.
///
/// Velocity and acceleration are angular-first classical quantities expressed
/// in the world frame at the root-link origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaseState {
    mode: BaseMode,
    frame: Frame,
    velocity: Twist,
    acceleration: Twist,
}

impl BaseState {
    pub(crate) fn new(mode: BaseMode) -> Self {
        Self {
            mode,
            frame: Frame::identity(),
            velocity: Twist::zeros(),
            acceleration: Twist::zeros(),
        }
    }

    /// Returns the immutable base mode selected when the robot was loaded.
    pub const fn mode(&self) -> BaseMode {
        self.mode
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

    pub(crate) fn set_frame(&mut self, frame: Frame) -> Result<()> {
        validate_frame(&frame)?;
        self.frame = frame;
        Ok(())
    }

    pub(crate) fn set_velocity(&mut self, velocity: Twist) -> Result<()> {
        self.require_floating_motion(velocity, "velocity")?;
        self.velocity = velocity;
        Ok(())
    }

    pub(crate) fn set_acceleration(&mut self, acceleration: Twist) -> Result<()> {
        self.require_floating_motion(acceleration, "acceleration")?;
        self.acceleration = acceleration;
        Ok(())
    }

    pub(crate) fn set_floating(
        &mut self,
        frame: Frame,
        velocity: Twist,
        acceleration: Twist,
    ) -> Result<()> {
        if self.mode != BaseMode::Floating {
            return Err(Error::FixedBaseMotion);
        }
        validate_frame(&frame)?;
        validate_twist(velocity, "velocity")?;
        validate_twist(acceleration, "acceleration")?;
        self.frame = frame;
        self.velocity = velocity;
        self.acceleration = acceleration;
        Ok(())
    }

    fn require_floating_motion(&self, value: Twist, field: &'static str) -> Result<()> {
        if self.mode != BaseMode::Floating {
            return Err(Error::FixedBaseMotion);
        }
        validate_twist(value, field)
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
