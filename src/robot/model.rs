//! Immutable runtime model storage, shared handles, and metadata queries.
//! URDF parsing and source joint/link descriptions live in crate::model.
use super::{FloatingRobot, LinkId, LoadBuffer, Robot, RootMode, Workspace, generalized_count};
use crate::{
    Error, Frame, JointType, Result,
    model::{Joint, JointKinematics, Link, LinkDynamics, Tree, load_urdf},
};
use nalgebra::{Matrix3, Vector3};
use std::{
    path::Path,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
};

const UNOWNED_MODEL_ID: u64 = 0;
static NEXT_MODEL_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Debug)]
pub(super) struct Model {
    pub(super) model_id: u64,
    pub(super) name: String,
    pub(super) joints: Box<[Joint]>,
    pub(super) links: Box<[Link]>,
    // Compact copies keep names, limits, and other metadata out of
    // the cache lines traversed by kinematics and dynamics kernels.
    pub(super) joint_kinematics: Box<[JointKinematics]>,
    pub(super) link_dynamics: Box<[LinkDynamics]>,
    // Joint indices include fixed joints; DOF indices do not. Links have an
    // additional root at index 0. See topology for the joint/link conversion.
    // DOF index -> model joint index.
    pub(super) active_joint_indices: Box<[usize]>,
    // Model joint index -> optional DOF index.
    pub(super) joint_dof_indices: Box<[Option<usize>]>,
    // Model joint index -> parent link index.
    pub(super) parent_link_indices: Box<[usize]>,
}

impl Model {
    fn from_tree(tree: Tree) -> Self {
        let model_id = NEXT_MODEL_ID.fetch_add(1, Ordering::Relaxed);
        assert_ne!(
            model_id, UNOWNED_MODEL_ID,
            "robot model identifier overflow"
        );
        let joint_kinematics: Box<[_]> = tree.joints.iter().map(Joint::kinematics).collect();
        let link_dynamics: Box<[_]> = tree.links.iter().map(Link::dynamics).collect();
        let active_joint_indices: Box<[_]> = tree
            .joints
            .iter()
            .enumerate()
            .filter_map(|(index, joint)| (joint.joint_type() != JointType::Fixed).then_some(index))
            .collect();
        let mut joint_dof_indices = vec![None; tree.joints.len()];
        for (dof_index, &joint_index) in active_joint_indices.iter().enumerate() {
            joint_dof_indices[joint_index] = Some(dof_index);
        }
        Self {
            model_id,
            name: tree.name,
            joints: tree.joints.into_boxed_slice(),
            links: tree.links.into_boxed_slice(),
            joint_kinematics,
            link_dynamics,
            active_joint_indices,
            joint_dof_indices: joint_dof_indices.into_boxed_slice(),
            parent_link_indices: tree.parent_link_indices.into_boxed_slice(),
        }
    }

    pub(super) fn link_count(&self) -> usize {
        self.links.len()
    }

    pub(super) fn joint_count(&self) -> usize {
        self.active_joint_indices.len()
    }

    pub(super) fn model_joint_count(&self) -> usize {
        self.joints.len()
    }

    pub(super) fn active_joint(&self, dof_index: usize) -> Result<&Joint> {
        let &joint_index = self
            .active_joint_indices
            .get(dof_index)
            .ok_or(Error::InvalidJointIndex { index: dof_index })?;
        Ok(&self.joints[joint_index])
    }

    pub(super) fn link_by_id(&self, link: LinkId) -> Result<&Link> {
        let index = self.validate_link_id(link)?;
        Ok(&self.links[index])
    }

    #[inline]
    pub(super) fn joint_value(&self, values: &[f64], joint_index: usize) -> f64 {
        self.joint_dof_indices[joint_index].map_or(0.0, |dof_index| values[dof_index])
    }

    pub(super) fn validate_slice(&self, name: &'static str, slice: &[f64]) -> Result<()> {
        self.validate_slice_length(name, slice.len(), self.joint_count())?;
        if slice.iter().all(|value| value.is_finite()) {
            Ok(())
        } else {
            Err(Error::NonFiniteInput { input: name })
        }
    }

    pub(super) fn validate_output(
        &self,
        base_mode: RootMode,
        name: &'static str,
        output: &[f64],
    ) -> Result<()> {
        self.validate_slice_length(name, output.len(), generalized_count(self, base_mode))
    }

    pub(super) fn validate_joint_output(&self, name: &'static str, output: &[f64]) -> Result<()> {
        self.validate_slice_length(name, output.len(), self.joint_count())
    }

    pub(super) fn validate_slice_length(
        &self,
        name: &'static str,
        actual: usize,
        expected: usize,
    ) -> Result<()> {
        if actual == expected {
            Ok(())
        } else {
            Err(Error::WrongSliceLength {
                slice: name,
                expected,
                actual,
            })
        }
    }

    pub(super) fn validate_link_id(&self, link: LinkId) -> Result<usize> {
        if link.model_id == self.model_id && link.index < self.links.len() {
            Ok(link.index)
        } else {
            Err(Error::InvalidLinkId)
        }
    }
}

fn load_model(path: impl AsRef<Path>) -> Result<Arc<Model>> {
    Ok(Arc::new(Model::from_tree(load_urdf(path)?)))
}

fn validate_floating_model(model: &Model) -> Result<()> {
    let root = model
        .links
        .first()
        .expect("validated robot tree has one root link");
    if root.mass() <= 0.0 {
        return Err(Error::InvalidModel(format!(
            "floating-base root link {} must have positive mass",
            root.name()
        )));
    }
    Ok(())
}

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

// All three handles expose the same immutable metadata without cloning an Arc
// for every query. Base-mode-dependent methods remain on calculation instances.
macro_rules! model_queries {
    ($handle:ty) => {
        impl $handle {
            /// Returns the robot name declared in the URDF.
            pub fn name(&self) -> &str {
                &self.model.name
            }

            /// Finds a model-scoped link identifier by URDF name.
            ///
            /// # Errors
            ///
            /// Returns [`Error::UnknownLink`] if the name is absent.
            pub fn link_id(&self, name: &str) -> Result<LinkId> {
                self.model
                    .links
                    .iter()
                    .position(|link| link.name() == name)
                    .map(|index| LinkId::new(self.model.model_id, index))
                    .ok_or_else(|| Error::UnknownLink {
                        name: name.to_owned(),
                    })
            }

            /// Returns the model-scoped identifier at a link enumeration index.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidLinkId`] if `index >= self.link_count()`.
            pub fn link_id_at(&self, index: usize) -> Result<LinkId> {
                if index < self.model.link_count() {
                    Ok(LinkId::new(self.model.model_id, index))
                } else {
                    Err(Error::InvalidLinkId)
                }
            }

            /// Returns the number of links, including the root link.
            pub fn link_count(&self) -> usize {
                self.model.link_count()
            }

            /// Returns the number of non-fixed joints in the model.
            pub fn joint_count(&self) -> usize {
                self.model.joint_count()
            }

            /// Returns the name of the joint at an active-DOF index.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidJointIndex`] when `dof_index` is out of range.
            pub fn joint_name(&self, dof_index: usize) -> Result<&str> {
                Ok(self.model.active_joint(dof_index)?.name())
            }

            /// Returns the motion type of the joint at an active-DOF index.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidJointIndex`] when `dof_index` is out of range.
            pub fn joint_type(&self, dof_index: usize) -> Result<JointType> {
                Ok(self.model.active_joint(dof_index)?.joint_type())
            }

            /// Returns the lower position limit of the joint at an active-DOF index.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidJointIndex`] when `dof_index` is out of range.
            pub fn joint_lower_limit(&self, dof_index: usize) -> Result<f64> {
                Ok(self.model.active_joint(dof_index)?.lower_limit())
            }

            /// Returns the upper position limit of the joint at an active-DOF index.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidJointIndex`] when `dof_index` is out of range.
            pub fn joint_upper_limit(&self, dof_index: usize) -> Result<f64> {
                Ok(self.model.active_joint(dof_index)?.upper_limit())
            }

            /// Returns the velocity limit of the joint at an active-DOF index.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidJointIndex`] when `dof_index` is out of range.
            pub fn joint_velocity_limit(&self, dof_index: usize) -> Result<f64> {
                Ok(self.model.active_joint(dof_index)?.velocity_limit())
            }

            /// Returns the root link identifier.
            pub fn root_link_id(&self) -> LinkId {
                LinkId::new(self.model.model_id, 0)
            }

            /// Returns the name of a model-scoped link.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidLinkId`] if `link` belongs to another model.
            pub fn link_name(&self, link: LinkId) -> Result<&str> {
                Ok(self.model.link_by_id(link)?.name())
            }

            /// Returns a link's mass in kilograms.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidLinkId`] if `link` belongs to another model.
            pub fn link_mass(&self, link: LinkId) -> Result<f64> {
                Ok(self.model.link_by_id(link)?.mass())
            }

            /// Returns a link's center of mass expressed in its link frame.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidLinkId`] if `link` belongs to another model.
            pub fn link_center_of_mass(&self, link: LinkId) -> Result<Vector3<f64>> {
                Ok(*self.model.link_by_id(link)?.center_of_mass())
            }

            /// Returns a link's rotational inertia about its center of mass.
            ///
            /// # Errors
            ///
            /// Returns [`Error::InvalidLinkId`] if `link` belongs to another model.
            pub fn link_inertia(&self, link: LinkId) -> Result<Matrix3<f64>> {
                Ok(*self.model.link_by_id(link)?.inertia())
            }
        }
    };
}
model_queries!(RobotModel);
model_queries!(Robot);
model_queries!(FloatingRobot);
