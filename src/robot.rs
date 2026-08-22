use std::{
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{
    BaseMode, BaseState, Error, Frame, Joint, JointType, Link, Result, Twist,
    model::{JointKinematics, LinkDynamics, Tree, load_urdf},
};

mod dynamics;
mod kinematics;
mod workspace;

pub use kinematics::InverseKinematicsOptions;
pub use workspace::{IndexedLoad, LinkId, Workspace};

const FLOATING_BASE_DOF: usize = 6;
const UNOWNED_MODEL_ID: u64 = 0;
static NEXT_MODEL_ID: AtomicU64 = AtomicU64::new(1);

/// Runtime-topology tree robot with runtime-size calculation APIs.
#[derive(Clone, Debug)]
pub struct Robot {
    model_id: u64,
    name: String,
    joints: Box<[Joint]>,
    links: Box<[Link]>,
    // Compact copies keep names, limits, and mutable presentation state out of
    // the cache lines traversed by kinematics and dynamics kernels.
    joint_kinematics: Box<[JointKinematics]>,
    link_dynamics: Box<[LinkDynamics]>,
    active_joint_indices: Box<[usize]>,
    joint_dof_indices: Box<[Option<usize>]>,
    parent_link_indices: Box<[usize]>,
    base: BaseState,
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
    /// Returns an error if the file cannot be parsed or its graph is invalid.
    pub fn from_urdf_with_base(path: impl AsRef<Path>, base_mode: BaseMode) -> Result<Self> {
        let tree = load_urdf(path)?;
        Ok(Self::from_tree(tree, base_mode))
    }

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
            base: BaseState::new(base_mode),
        }
    }

    /// Returns the root-link runtime state.
    pub const fn base(&self) -> &BaseState {
        &self.base
    }

    /// Returns whether the root link is fixed or floating.
    pub const fn base_mode(&self) -> BaseMode {
        self.base.mode()
    }

    /// Sets the root-link pose in the world frame.
    ///
    /// # Errors
    ///
    /// Returns an error if the pose contains a non-finite value.
    pub fn set_base_frame(&mut self, frame: Frame) -> Result<()> {
        self.base.set_frame(frame)
    }

    /// Sets floating-base classical velocity expressed in the world frame.
    ///
    /// # Errors
    ///
    /// Returns an error for a fixed base or a non-finite value.
    pub fn set_base_velocity(&mut self, velocity: Twist) -> Result<()> {
        self.base.set_velocity(velocity)
    }

    /// Sets floating-base classical acceleration expressed in the world frame.
    ///
    /// # Errors
    ///
    /// Returns an error for a fixed base or a non-finite value.
    pub fn set_base_acceleration(&mut self, acceleration: Twist) -> Result<()> {
        self.base.set_acceleration(acceleration)
    }

    /// Atomically replaces the complete floating-base state.
    ///
    /// # Errors
    ///
    /// Returns an error for a fixed base or a non-finite value.
    pub fn set_floating_base_state(
        &mut self,
        frame: Frame,
        velocity: Twist,
        acceleration: Twist,
    ) -> Result<()> {
        self.base.set_floating(frame, velocity, acceleration)
    }

    /// Returns the robot name declared in the URDF.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns all links in topological order, starting with the root link.
    pub fn links(&self) -> &[Link] {
        &self.links
    }

    /// Returns all joints, including fixed joints, in the same topological
    /// order as their child links.
    pub fn joints(&self) -> &[Joint] {
        &self.joints
    }

    /// Returns the model's root link.
    pub fn root_link(&self) -> &Link {
        &self.links[0]
    }

    /// Finds a link by its URDF name.
    ///
    /// # Errors
    ///
    /// Returns [`Error::UnknownLink`] if the name is absent.
    pub fn link(&self, name: &str) -> Result<&Link> {
        self.links
            .iter()
            .find(|link| link.name() == name)
            .ok_or_else(|| Error::UnknownLink {
                name: name.to_owned(),
            })
    }

    /// Finds a model-scoped link identifier by URDF name.
    ///
    /// # Errors
    ///
    /// Returns [`Error::UnknownLink`] if the name is absent.
    pub fn link_id(&self, name: &str) -> Result<LinkId> {
        self.links
            .iter()
            .position(|link| link.name() == name)
            .map(|index| LinkId::new(self.model_id, index))
            .ok_or_else(|| Error::UnknownLink {
                name: name.to_owned(),
            })
    }

    /// Returns the number of links, including the root link.
    pub fn link_count(&self) -> usize {
        self.links.len()
    }

    /// Returns the number of non-fixed joints in the model.
    pub fn joint_count(&self) -> usize {
        self.active_joint_indices.len()
    }

    /// Returns the number of non-fixed joint degrees of freedom.
    ///
    /// This is an alias for [`Robot::joint_count`].
    pub fn dof_count(&self) -> usize {
        self.joint_count()
    }

    /// Returns model-joint indices for all non-fixed degrees of freedom.
    pub fn active_joint_indices(&self) -> &[usize] {
        &self.active_joint_indices
    }

    /// Maps a model-joint index to its compact active-DOF index.
    ///
    /// Returns `None` for fixed joints and out-of-range indices.
    pub fn joint_dof_index(&self, joint_index: usize) -> Option<usize> {
        self.joint_dof_indices.get(joint_index).copied().flatten()
    }

    /// Returns the number of generalized base coordinates used by calculations.
    ///
    /// For a floating base this is six, occupying the leading generalized
    /// entries in world-frame angular-then-linear order: `[omega_x, omega_y,
    /// omega_z, v_x, v_y, v_z]` (and likewise for acceleration). Non-fixed
    /// joint entries follow in URDF order.
    pub const fn base_dof_count(&self) -> usize {
        match self.base.mode() {
            BaseMode::Fixed => 0,
            BaseMode::Floating => FLOATING_BASE_DOF,
        }
    }

    /// Returns the runtime generalized-vector size for this robot.
    ///
    /// Floating-base generalized vectors are ordered `[base angular, base
    /// linear, joints]`; fixed-base vectors contain only non-fixed joint entries.
    pub fn generalized_count(&self) -> usize {
        self.base_dof_count() + self.joint_count()
    }

    fn model_joint_count(&self) -> usize {
        self.joints.len()
    }

    #[inline]
    fn joint_value(&self, values: &[f64], joint_index: usize) -> f64 {
        self.joint_dof_indices[joint_index].map_or(0.0, |dof_index| values[dof_index])
    }

    /// Allocates reusable storage for runtime-sized calculations on this model.
    pub fn workspace(&self) -> Workspace {
        Workspace::new(
            self.model_id,
            self.joint_count(),
            self.model_joint_count(),
            self.generalized_count(),
        )
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

    fn validate_workspace(&self, workspace: &Workspace) -> Result<()> {
        if workspace.model_id == self.model_id
            && workspace.joint_count == self.joint_count()
            && workspace.model_joint_count == self.model_joint_count()
            && workspace.generalized_count == self.generalized_count()
        {
            Ok(())
        } else {
            Err(Error::InvalidWorkspace)
        }
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
    fn internal_identifier_and_workspace_guards_cover_each_short_circuit() {
        let robot = robot();
        assert_eq!(
            robot
                .validate_link_id(LinkId::new(robot.model_id, 0))
                .unwrap(),
            0
        );
        assert!(matches!(
            robot.validate_link_id(LinkId::new(robot.model_id.wrapping_add(1), 0)),
            Err(Error::InvalidLinkId)
        ));
        assert!(matches!(
            robot.validate_link_id(LinkId::new(robot.model_id, robot.link_count())),
            Err(Error::InvalidLinkId)
        ));

        let valid = robot.workspace();
        assert!(robot.validate_workspace(&valid).is_ok());

        let mut wrong_model = valid.clone();
        wrong_model.model_id = robot.model_id.wrapping_add(1);
        assert!(matches!(
            robot.validate_workspace(&wrong_model),
            Err(Error::InvalidWorkspace)
        ));

        let mut wrong_joint_count = valid.clone();
        wrong_joint_count.joint_count += 1;
        assert!(matches!(
            robot.validate_workspace(&wrong_joint_count),
            Err(Error::InvalidWorkspace)
        ));

        let mut wrong_model_joint_count = valid.clone();
        wrong_model_joint_count.model_joint_count += 1;
        assert!(matches!(
            robot.validate_workspace(&wrong_model_joint_count),
            Err(Error::InvalidWorkspace)
        ));

        let mut wrong_generalized_count = valid;
        wrong_generalized_count.generalized_count += 1;
        assert!(matches!(
            robot.validate_workspace(&wrong_generalized_count),
            Err(Error::InvalidWorkspace)
        ));
    }
}
