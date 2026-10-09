use std::{path::Path, sync::Arc};

use crate::{Frame, Result};

use super::{
    FloatingRobot, LoadBuffer, Model, Robot, Workspace, load_model, validate_floating_model,
};

/// Immutable, shared robot topology and inertial data without calculation storage.
///
/// Cloning this handle shares the model and its [`crate::LinkId`] scope. Use
/// [`Self::robot`] or [`Self::floating_robot`] to create independent calculation
/// instances. Handles and instances can outlive the original loader or each other.
#[derive(Clone, Debug)]
pub struct RobotModel {
    pub(super) model: Arc<Model>,
}

impl RobotModel {
    /// Loads and validates a URDF without allocating a calculation workspace.
    pub fn from_urdf(path: impl AsRef<Path>) -> Result<Self> {
        Ok(Self {
            model: load_model(path)?,
        })
    }

    /// Creates an independent fixed-base calculation instance at the world origin.
    pub fn robot(&self) -> Robot {
        Robot {
            model: Arc::clone(&self.model),
            workspace: Workspace::new(&self.model),
            world_from_root: Frame::identity(),
        }
    }

    /// Checks the positive-root-mass requirement without allocating a workspace.
    pub fn validate_floating_base(&self) -> Result<()> {
        validate_floating_model(&self.model)
    }

    /// Creates independent floating-base calculation storage.
    ///
    /// Returns a model error unless the root link has positive mass.
    pub fn floating_robot(&self) -> Result<FloatingRobot> {
        self.validate_floating_base()?;
        Ok(FloatingRobot {
            model: Arc::clone(&self.model),
            workspace: Workspace::new(&self.model),
        })
    }

    /// Creates reusable external-load storage in this model's link-ID scope.
    pub fn load_buffer(&self) -> LoadBuffer {
        LoadBuffer::new(&self.model)
    }
}
