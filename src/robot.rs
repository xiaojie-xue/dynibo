use crate::{Frame, Result};
use std::{path::Path, sync::Arc};

mod dynamics;
mod kinematics;
mod loads;
mod model;
mod topology;
mod workspace;

pub use kinematics::InverseKinematicsOptions;
pub use loads::{IndexedLoad, LoadBuffer};
use model::Model;
pub use model::RobotModel;
pub use topology::LinkId;
use workspace::Workspace;

const FLOATING_BASE_DOF: usize = 6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum RootMode {
    Fixed,
    Floating,
}

/// A fixed-base robot model with reusable, instance-local calculation storage.
#[derive(Debug)]
pub struct Robot {
    model: Arc<Model>,
    workspace: Workspace,
    world_from_root: Frame,
}

/// A floating-base robot model with reusable, instance-local calculation storage.
#[derive(Debug)]
pub struct FloatingRobot {
    model: Arc<Model>,
    workspace: Workspace,
}

const fn base_dof_count(base_mode: RootMode) -> usize {
    match base_mode {
        RootMode::Fixed => 0,
        RootMode::Floating => FLOATING_BASE_DOF,
    }
}

fn generalized_count(model: &Model, base_mode: RootMode) -> usize {
    base_dof_count(base_mode) + model.joint_count()
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
        }
    }

    /// Returns the runtime generalized-vector size for this robot.
    ///
    /// Floating-base generalized vectors are ordered `[base angular, base
    /// linear, joints]`; fixed-base vectors contain only non-fixed joint entries.
    pub fn generalized_count(&self) -> usize {
        self.model.joint_count()
    }
}

impl FloatingRobot {
    /// Loads a tree robot model with a six-degree-of-freedom floating root.
    pub fn from_urdf(path: impl AsRef<Path>) -> Result<Self> {
        RobotModel::from_urdf(path)?.floating_robot()
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
        Self { model, workspace }
    }

    /// Returns the runtime generalized-vector size (six base coordinates plus joints).
    pub fn generalized_count(&self) -> usize {
        generalized_count(self.model.as_ref(), RootMode::Floating)
    }
}

#[cfg(test)]
mod tests;
