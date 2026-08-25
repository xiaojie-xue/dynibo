use std::{
    path::Path,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
};

use nalgebra::{Matrix3, Vector3};

use crate::{
    BaseMode, BaseState, Error, JointType, Result,
    model::{Joint, JointKinematics, Link, LinkDynamics, Tree, load_urdf},
};

mod dynamics;
mod kinematics;
mod workspace;

pub use kinematics::InverseKinematicsOptions;
use workspace::Workspace;
pub use workspace::{IndexedLoad, LinkId};

const FLOATING_BASE_DOF: usize = 6;
const UNOWNED_MODEL_ID: u64 = 0;
static NEXT_MODEL_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Debug)]
struct Model {
    model_id: u64,
    name: String,
    joints: Box<[Joint]>,
    links: Box<[Link]>,
    // Compact copies keep names, limits, and other metadata out of
    // the cache lines traversed by kinematics and dynamics kernels.
    joint_kinematics: Box<[JointKinematics]>,
    link_dynamics: Box<[LinkDynamics]>,
    active_joint_indices: Box<[usize]>,
    joint_dof_indices: Box<[Option<usize>]>,
    parent_link_indices: Box<[usize]>,
    base_mode: BaseMode,
}

/// A robot model with reusable, instance-local calculation storage.
#[derive(Debug)]
pub struct Robot {
    model: Arc<Model>,
    workspace: Workspace,
}

impl Model {
    fn from_tree(tree: Tree, base_mode: BaseMode) -> Self {
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
            base_mode,
        }
    }

    const fn base_mode(&self) -> BaseMode {
        self.base_mode
    }

    fn link_count(&self) -> usize {
        self.links.len()
    }

    fn joint_count(&self) -> usize {
        self.active_joint_indices.len()
    }

    const fn base_dof_count(&self) -> usize {
        match self.base_mode {
            BaseMode::Fixed => 0,
            BaseMode::Floating => FLOATING_BASE_DOF,
        }
    }

    fn generalized_count(&self) -> usize {
        self.base_dof_count() + self.joint_count()
    }

    fn model_joint_count(&self) -> usize {
        self.joints.len()
    }

    fn active_joint(&self, dof_index: usize) -> Result<&Joint> {
        let &joint_index = self
            .active_joint_indices
            .get(dof_index)
            .ok_or(Error::InvalidJointIndex { index: dof_index })?;
        Ok(&self.joints[joint_index])
    }

    fn link_by_id(&self, link: LinkId) -> Result<&Link> {
        let index = self.validate_link_id(link)?;
        Ok(&self.links[index])
    }

    #[inline]
    fn joint_value(&self, values: &[f64], joint_index: usize) -> f64 {
        self.joint_dof_indices[joint_index].map_or(0.0, |dof_index| values[dof_index])
    }

    fn validate_base_state(&self, base: &BaseState) -> Result<()> {
        if self.base_mode == BaseMode::Fixed {
            if base.velocity() != crate::Twist::zeros() {
                return Err(Error::InvalidBaseState {
                    field: "velocity",
                    reason: "must be zero for a fixed base",
                });
            }
            if base.acceleration() != crate::Twist::zeros() {
                return Err(Error::InvalidBaseState {
                    field: "acceleration",
                    reason: "must be zero for a fixed base",
                });
            }
        }
        Ok(())
    }

    fn validate_slice(&self, name: &'static str, slice: &[f64]) -> Result<()> {
        self.validate_slice_length(name, slice.len(), self.joint_count())
    }

    fn validate_output(&self, name: &'static str, output: &[f64]) -> Result<()> {
        self.validate_slice_length(name, output.len(), self.generalized_count())
    }

    fn validate_joint_output(&self, name: &'static str, output: &[f64]) -> Result<()> {
        self.validate_slice_length(name, output.len(), self.joint_count())
    }

    fn validate_slice_length(
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

    fn validate_link_id(&self, link: LinkId) -> Result<usize> {
        if link.model_id == self.model_id && link.index < self.links.len() {
            Ok(link.index)
        } else {
            Err(Error::InvalidLinkId)
        }
    }
}

impl Robot {
    /// Loads and validates a tree robot model from a URDF file.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be parsed or its graph is invalid.
    pub fn from_urdf(path: impl AsRef<Path>) -> Result<Self> {
        Self::from_urdf_with_base(path, BaseMode::Fixed)
    }

    /// Loads a tree robot model and selects how its root link is connected to the world.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be parsed, its graph is invalid, or
    /// a floating-base model's root link does not have positive mass.
    pub fn from_urdf_with_base(path: impl AsRef<Path>, base_mode: BaseMode) -> Result<Self> {
        let tree = load_urdf(path)?;
        Self::from_tree(tree, base_mode)
    }

    fn from_tree(tree: Tree, base_mode: BaseMode) -> Result<Self> {
        if base_mode == BaseMode::Floating {
            let root = tree
                .links
                .first()
                .expect("validated robot tree has one root link");
            if root.mass() <= 0.0 {
                return Err(Error::InvalidModel(format!(
                    "floating-base root link {} must have positive mass",
                    root.name()
                )));
            }
        }
        let model = Arc::new(Model::from_tree(tree, base_mode));
        let workspace = Workspace::new(model.as_ref());
        Ok(Self { model, workspace })
    }

    /// Creates another calculation instance sharing this robot's immutable model.
    ///
    /// The returned robot allocates fresh calculation storage, so both instances
    /// can be used concurrently without locking.
    pub fn fork(&self) -> Self {
        let model = Arc::clone(&self.model);
        let workspace = Workspace::new(model.as_ref());
        Self { model, workspace }
    }

    /// Returns whether the root link is fixed or floating.
    pub fn base_mode(&self) -> BaseMode {
        self.model.base_mode()
    }

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

    /// Returns the runtime generalized-vector size for this robot.
    ///
    /// Floating-base generalized vectors are ordered `[base angular, base
    /// linear, joints]`; fixed-base vectors contain only non-fixed joint entries.
    pub fn generalized_count(&self) -> usize {
        self.model.generalized_count()
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

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    fn robot() -> Robot {
        Robot::from_urdf(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf"))
            .unwrap()
    }

    #[test]
    fn identifiers_queries_and_forks_preserve_model_scope() {
        let robot = robot();
        let fork = robot.fork();
        assert_eq!(
            robot
                .model
                .validate_link_id(LinkId::new(robot.model.model_id, 0))
                .unwrap(),
            0
        );
        assert!(matches!(
            robot
                .model
                .validate_link_id(LinkId::new(robot.model.model_id.wrapping_add(1), 0)),
            Err(Error::InvalidLinkId)
        ));
        assert!(matches!(
            robot
                .model
                .validate_link_id(LinkId::new(robot.model.model_id, robot.link_count())),
            Err(Error::InvalidLinkId)
        ));
        assert_eq!(robot.root_link_id(), fork.root_link_id());
        assert_eq!(robot.joint_name(0).unwrap(), "test_joint_1");
        assert_eq!(robot.joint_type(0).unwrap(), JointType::Revolute);
        assert_eq!(robot.joint_lower_limit(0).unwrap(), -0.610865238198015);
        assert_eq!(robot.joint_upper_limit(0).unwrap(), 0.610865238198015);
        assert_eq!(robot.joint_velocity_limit(0).unwrap(), 180.0);
        assert!(matches!(
            robot.joint_name(robot.joint_count()),
            Err(Error::InvalidJointIndex { .. })
        ));
    }
}
