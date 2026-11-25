use crate::{Error, Frame, Result};
use nalgebra::Vector3;
use std::{path::Path, sync::Arc};

mod base_kind;
mod dynamics;
mod kinematics;
mod loads;
mod model;
mod topology;
mod workspace;

use base_kind::{BaseKind, BaseMotion, FLOATING_BASE_DOF, Fixed, Floating};
pub use kinematics::InverseKinematicsOptions;
pub use loads::{IndexedLoad, LoadBuffer};
use model::Model;
pub use model::RobotModel;
pub use topology::LinkId;
use workspace::Workspace;

/// Standard Earth gravitational acceleration along the world `-z` axis (m/s²).
pub(super) const DEFAULT_GRAVITY: Vector3<f64> = Vector3::new(0.0, 0.0, -9.80665);

/// A fixed-base robot model with reusable, instance-local calculation storage.
#[derive(Debug)]
pub struct Robot {
    model: Arc<Model>,
    workspace: Workspace,
    world_from_root: Frame,
    world_gravity: Vector3<f64>,
}

/// A floating-base robot model with reusable, instance-local calculation storage.
#[derive(Debug)]
pub struct FloatingRobot {
    model: Arc<Model>,
    workspace: Workspace,
    world_gravity: Vector3<f64>,
}

fn generalized_count<B: BaseKind>(model: &Model) -> usize {
    B::DOF + model.joint_count()
}

fn validate_gravity(gravity: &Vector3<f64>) -> Result<()> {
    if gravity.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(Error::NonFiniteInput { input: "gravity" })
    }
}

impl Robot {
    /// Loads and validates a tree robot model from a URDF file.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be parsed or its graph is invalid.
    pub fn from_urdf(path: impl AsRef<Path>) -> Result<Self> {
        Ok(RobotModel::from_urdf(path)?.robot())
    }

    /// Returns the fixed root-link pose in the world frame.
    pub const fn base_frame(&self) -> &Frame {
        &self.world_from_root
    }

    /// Replaces the fixed root-link pose in the world frame.
    pub fn set_base_frame(&mut self, frame: Frame) -> Result<()> {
        crate::base::validate_frame(&frame)?;
        self.world_from_root = frame;
        Ok(())
    }

    /// Returns the world-frame gravitational acceleration vector in m/s².
    ///
    /// Defaults to `[0.0, 0.0, -9.80665]`.
    pub const fn gravity_vector(&self) -> &Vector3<f64> {
        &self.world_gravity
    }

    /// Replaces the world-frame gravitational acceleration vector in m/s².
    ///
    /// # Errors
    ///
    /// Returns [`Error::NonFiniteInput`] if any component is non-finite.
    pub fn set_gravity(&mut self, gravity: Vector3<f64>) -> Result<()> {
        validate_gravity(&gravity)?;
        self.world_gravity = gravity;
        Ok(())
    }

    /// Shares the immutable model without allocating calculation storage.
    pub fn model(&self) -> RobotModel {
        RobotModel {
            model: Arc::clone(&self.model),
        }
    }

    /// Creates another calculation instance sharing this robot's immutable model.
    ///
    /// The returned robot allocates fresh calculation storage, so both instances
    /// can be used concurrently without locking.
    pub fn fork(&self) -> Self {
        let model = Arc::clone(&self.model);
        let workspace = Workspace::new(model.as_ref());
        Self {
            model,
            workspace,
            world_from_root: self.world_from_root,
            world_gravity: self.world_gravity,
        }
    }

    /// Returns the runtime generalized-vector size for this fixed-base robot.
    ///
    /// Fixed-base generalized vectors contain only non-fixed joint entries.
    pub fn generalized_count(&self) -> usize {
        generalized_count::<Fixed>(self.model.as_ref())
    }
}

impl FloatingRobot {
    /// Loads a tree robot model with a six-degree-of-freedom floating root.
    pub fn from_urdf(path: impl AsRef<Path>) -> Result<Self> {
        RobotModel::from_urdf(path)?.floating_robot()
    }

    /// Returns the world-frame gravitational acceleration vector in m/s².
    ///
    /// Defaults to `[0.0, 0.0, -9.80665]`.
    pub const fn gravity_vector(&self) -> &Vector3<f64> {
        &self.world_gravity
    }

    /// Replaces the world-frame gravitational acceleration vector in m/s².
    ///
    /// # Errors
    ///
    /// Returns [`Error::NonFiniteInput`] if any component is non-finite.
    pub fn set_gravity(&mut self, gravity: Vector3<f64>) -> Result<()> {
        validate_gravity(&gravity)?;
        self.world_gravity = gravity;
        Ok(())
    }

    /// Shares the immutable model without allocating calculation storage.
    pub fn model(&self) -> RobotModel {
        RobotModel {
            model: Arc::clone(&self.model),
        }
    }

    /// Creates another calculation instance sharing this robot's immutable model.
    pub fn fork(&self) -> Self {
        let model = Arc::clone(&self.model);
        let workspace = Workspace::new(model.as_ref());
        Self {
            model,
            workspace,
            world_gravity: self.world_gravity,
        }
    }

    /// Returns the runtime generalized-vector size (six base coordinates plus joints).
    pub fn generalized_count(&self) -> usize {
        generalized_count::<Floating>(self.model.as_ref())
    }
}

#[cfg(test)]
#[cfg_attr(coverage_nightly, coverage(off))]
mod tests;
