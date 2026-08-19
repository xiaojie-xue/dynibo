use std::{
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
};

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::{
    BaseMode, BaseState, Error, Frame, Joint, JointType, Link, Result, Twist, Wrench,
    model::{JointKinematics, LinkDynamics},
    urdf::tree_model,
};

mod workspace;

pub use workspace::{IndexedLoad, LinkId, Workspace};

const GRAVITY: f64 = 9.80665;
const FLOATING_BASE_DOF: usize = 6;
const UNOWNED_MODEL_ID: u64 = 0;
static NEXT_MODEL_ID: AtomicU64 = AtomicU64::new(1);

/// Configuration for damped-least-squares inverse kinematics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InverseKinematicsOptions {
    /// Maximum number of joint updates.
    pub max_iterations: usize,
    /// Maximum accepted Euclidean position error, in metres.
    pub translation_tolerance: f64,
    /// Maximum accepted rotation-vector norm, in radians.
    pub rotation_tolerance: f64,
    /// Damping factor `lambda` in `J^T (J J^T + lambda^2 I)^-1`.
    pub damping: f64,
    /// Maximum Euclidean norm of one joint update.
    pub max_step_norm: f64,
}

impl Default for InverseKinematicsOptions {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            translation_tolerance: 1.0e-6,
            rotation_tolerance: 1.0e-6,
            damping: 1.0e-3,
            max_step_norm: 0.5,
        }
    }
}

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
    joint_parents: Box<[usize]>,
    leaf_links: Box<[usize]>,
    base: BaseState,
}

struct DynamicsScratch<'a> {
    transforms: &'a mut [Frame],
    angular_velocities: &'a mut [Vector3<f64>],
    angular_accelerations: &'a mut [Vector3<f64>],
    origin_accelerations: &'a mut [Vector3<f64>],
    link_accelerations: &'a mut [Vector3<f64>],
    link_loads: &'a mut [Wrench],
}

struct GravityScratch<'a> {
    transforms: &'a mut [Frame],
    gravity_at_link: &'a mut [Vector3<f64>],
    link_loads: &'a mut [Wrench],
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
        let robot = urdf_rs::read_file(path)?;
        let model = tree_model(&robot)?;
        let model_id = NEXT_MODEL_ID.fetch_add(1, Ordering::Relaxed);
        assert_ne!(
            model_id, UNOWNED_MODEL_ID,
            "robot model identifier overflow"
        );
        let joint_kinematics: Box<[_]> = model.joints.iter().map(Joint::kinematics).collect();
        let link_dynamics: Box<[_]> = model.links.iter().map(Link::dynamics).collect();
        let active_joint_indices: Box<[_]> = model
            .joints
            .iter()
            .enumerate()
            .filter_map(|(index, joint)| (joint.joint_type() != JointType::Fixed).then_some(index))
            .collect();
        let mut joint_dof_indices = vec![None; model.joints.len()];
        for (dof_index, &joint_index) in active_joint_indices.iter().enumerate() {
            joint_dof_indices[joint_index] = Some(dof_index);
        }
        Ok(Self {
            model_id,
            name: robot.name,
            joints: model.joints.into_boxed_slice(),
            links: model.links.into_boxed_slice(),
            joint_kinematics,
            link_dynamics,
            active_joint_indices,
            joint_dof_indices: joint_dof_indices.into_boxed_slice(),
            joint_parents: model.joint_parents.into_boxed_slice(),
            leaf_links: model.leaf_links.into_boxed_slice(),
            base: BaseState::new(base_mode),
        })
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

    /// Iterates over links that have no children.
    pub fn leaf_links(&self) -> impl ExactSizeIterator<Item = &Link> {
        self.leaf_links.iter().map(|&index| &self.links[index])
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

    /// Computes a target link frame using runtime-sized input and workspace.
    ///
    /// For the joints on the root-to-target path, the returned world pose is
    ///
    /// $$
    /// {}^W T_{\mathrm{target}}(q) = {}^W T_{\mathrm{base}}
    /// \prod_{i \in \mathrm{path}} {}^{i-1}T_i(q_i).
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid input length, link ID, or workspace.
    pub fn forward_kinematics(
        &self,
        q: &[f64],
        target: LinkId,
        workspace: &mut Workspace,
    ) -> Result<Frame> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        let target_index = self.validate_link_id(target)?;
        let depth = self.prepare_ancestor_path(target_index, &mut workspace.ancestor_path);
        Ok(*self.base.frame() * self.target_frame_kernel(q, &workspace.ancestor_path[..depth])?)
    }

    /// Writes a runtime-sized `6 x G` geometric Jacobian in column-major order.
    ///
    /// Each column stores `[angular_x, angular_y, angular_z, linear_x,
    /// linear_y, linear_z]`.
    /// Here `G` is [`Robot::generalized_count`]. For a floating base the first
    /// six columns map world-expressed base velocity in angular-then-linear
    /// order and the remaining columns map non-fixed URDF joint velocity. The Jacobian,
    /// including its linear rows, is expressed at the target-link origin in the
    /// world frame.
    ///
    /// $$
    /// {}^W V_{\mathrm{target}} = J(q) \nu, \qquad
    /// J(q) = \begin{bmatrix} J_\omega(q) \\ J_v(q) \end{bmatrix}.
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error unless `output.len() == 6 * generalized_count()`, or for an
    /// invalid input length, link ID, or workspace.
    pub fn jacobian(
        &self,
        q: &[f64],
        target: LinkId,
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice_length(
            "jacobian output",
            output.len(),
            6 * self.generalized_count(),
        )?;
        let target_index = self.validate_link_id(target)?;
        let local_target = self.forward_kinematics_and_jacobian_kernel(
            q,
            target_index,
            &mut workspace.frames,
            &mut workspace.jacobian,
            &mut workspace.ancestor_path,
            true,
        )?;
        self.write_generalized_jacobian(&workspace.jacobian, &local_target, output);
        Ok(())
    }

    /// Writes the runtime-sized `6 x G` time derivative of the geometric
    /// Jacobian in column-major order.
    ///
    /// Each column stores `[angular_x, angular_y, angular_z, linear_x,
    /// linear_y, linear_z]`, matching [`Robot::jacobian`]. It uses the same
    /// world-frame target-origin convention and generalized-column ordering.
    /// The result
    /// combines with [`Robot::jacobian`] as `A = J * nu_dot + J_dot * nu`.
    /// Columns of joints outside the target's ancestor chain are zero; fixed
    /// joints do not occupy columns. A root target yields an all-zero matrix.
    /// In general, the target spatial acceleration is
    ///
    /// $$
    /// {}^W A_{\mathrm{target}} = J(q) \dot\nu + \dot J(q, \nu) \nu.
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error unless `output.len() == 6 * generalized_count()`, or for an
    /// invalid input length, link ID, or workspace.
    pub fn jacobian_derivative(
        &self,
        q: &[f64],
        qd: &[f64],
        target: LinkId,
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice_length(
            "jacobian derivative output",
            output.len(),
            6 * self.generalized_count(),
        )?;
        let target_index = self.validate_link_id(target)?;
        self.joint_jacobian_derivative_kernel(
            q,
            qd,
            target_index,
            &mut workspace.frames,
            &mut workspace.angular_velocities,
            &mut workspace.origin_velocities,
            &mut workspace.ancestor_path,
            &mut workspace.jacobian_derivative,
        )?;
        let depth = self.prepare_ancestor_path(target_index, &mut workspace.ancestor_path);
        let path = &workspace.ancestor_path[..depth];
        let local_target = self.jacobian_kernel(
            &workspace.frames,
            target_index,
            path,
            &mut workspace.jacobian,
            true,
        )?;
        self.write_generalized_jacobian_derivative(
            qd,
            &local_target,
            &workspace.jacobian,
            &workspace.jacobian_derivative,
            output,
        );
        Ok(())
    }

    /// Writes a runtime-sized inverse-kinematics solution using the supplied options.
    ///
    /// Each iteration applies a damped-least-squares update,
    ///
    /// $$
    /// \Delta q = J^T\left(JJ^T + \lambda^2 I\right)^{-1} e,
    /// \qquad q_{k+1} = q_k + \Delta q.
    /// $$
    ///
    /// where `e` combines target translation and rotation-vector errors.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid lengths, link ID, workspace, solver input,
    /// numerical failure, limits, or non-convergence.
    pub fn inverse_kinematics(
        &self,
        initial_q: &[f64],
        target: LinkId,
        desired: &Frame,
        options: InverseKinematicsOptions,
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_workspace(workspace)?;
        if self.base_mode() == BaseMode::Floating {
            return Err(Error::FloatingBaseIkUnsupported);
        }
        self.validate_slice("initial_q", initial_q)?;
        self.validate_joint_output("inverse kinematics output", output)?;
        let target_index = self.validate_link_id(target)?;
        let local_desired = self.base.frame().inverse() * *desired;
        self.inverse_kinematics_kernel(
            initial_q,
            target_index,
            &local_desired,
            options,
            &mut workspace.frames,
            &mut workspace.jacobian,
            &mut workspace.q_work,
            &mut workspace.step,
            &mut workspace.ancestor_path,
        )?;
        output.copy_from_slice(&workspace.q_work);
        Ok(())
    }

    /// Computes runtime-sized spatial velocity at a point on a target link.
    ///
    /// The Robot's base state supplies root motion; `tool` selects a point
    /// rigidly attached to the target link. The returned angular-first twist is
    /// expressed in the world frame at that selected point.
    ///
    /// $$
    /// V_{\mathrm{tool}} = J_{\mathrm{tool}}(q) \nu.
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input lengths, link ID, or workspace.
    pub fn forward_velocity_kinematics(
        &self,
        q: &[f64],
        qd: &[f64],
        target: LinkId,
        tool: &Frame,
        workspace: &mut Workspace,
    ) -> Result<Twist> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        let target_index = self.validate_link_id(target)?;
        Ok(self.forward_velocity_for_base(
            q,
            qd,
            target_index,
            tool,
            self.base.frame(),
            self.base.velocity(),
            &mut workspace.ancestor_path,
        ))
    }

    /// Computes world-expressed spatial acceleration of a target link origin.
    ///
    /// The returned angular-first acceleration is
    ///
    /// $$
    /// A_{\mathrm{target}} = J(q) \dot\nu + \dot J(q, \nu) \nu.
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input lengths, link ID, or workspace.
    pub fn forward_acceleration_kinematics(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        target: LinkId,
        workspace: &mut Workspace,
    ) -> Result<Twist> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        let target_index = self.validate_link_id(target)?;
        let depth = self.prepare_ancestor_path(target_index, &mut workspace.ancestor_path);
        let (local_target, relative_velocity, relative_acceleration) = self.motion_for_joints(
            q,
            qd,
            qdd,
            workspace.ancestor_path[..depth].iter().rev().copied(),
        );
        let rotation = self.base.frame().rotation;
        let offset = rotation * local_target.translation.vector;
        let relative_angular = rotation * relative_velocity.angular;
        let relative_linear = rotation * relative_velocity.linear;
        let base_velocity = self.base.velocity();
        let base_acceleration = self.base.acceleration();
        Ok(Twist::new(
            base_acceleration.angular
                + base_velocity.angular.cross(&relative_angular)
                + rotation * relative_acceleration.angular,
            base_acceleration.linear
                + base_acceleration.angular.cross(&offset)
                + base_velocity
                    .angular
                    .cross(&base_velocity.angular.cross(&offset))
                + 2.0 * base_velocity.angular.cross(&relative_linear)
                + rotation * relative_acceleration.linear,
        ))
    }

    /// Writes the runtime-sized `G x G` mass matrix in column-major order.
    ///
    /// Rows and columns follow the generalized-vector ordering: for a floating
    /// base, world-frame angular, world-frame linear, then non-fixed URDF joints.
    ///
    /// Fixed joints do not occupy rows or columns, but their subtree inertia
    /// still contributes to moving ancestors.
    /// It is the inertia term in the manipulator equation
    ///
    /// $$
    /// \tau = M(q) \dot\nu + C(q, \nu) \nu + g(q).
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error unless `output.len() == generalized_count().pow(2)`,
    /// or for an invalid input length or workspace.
    pub fn mass_matrix(
        &self,
        q: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice_length(
            "mass matrix output",
            output.len(),
            self.generalized_count() * self.generalized_count(),
        )?;
        if self.base_mode() == BaseMode::Fixed {
            self.mass_matrix_kernel(q, workspace, output);
        } else {
            self.floating_mass_matrix_kernel(q, workspace, output);
        }
        Ok(())
    }

    /// Writes velocity-product generalized forces `C(q, qd) * qd`.
    ///
    /// Gravity, prescribed base acceleration, and external loads are excluded.
    /// For a floating base, the stored base velocity participates in the result
    /// and output is ordered `[base torque, base force, joint forces]`. The
    /// base wrench is expressed in the world frame at the root origin.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input or output lengths or workspace.
    pub fn velocity_product_forces(
        &self,
        q: &[f64],
        qd: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_output("velocity product output", output)?;
        workspace.step.fill(0.0);
        workspace.link_loads.fill(Wrench::zeros());
        output.fill(0.0);
        let joint_offset = self.base_dof_count();
        let base_load = self.inverse_dynamics_kernel(
            q,
            qd,
            &workspace.step,
            self.base.frame(),
            self.base.velocity(),
            Twist::zeros(),
            Vector3::zeros(),
            Wrench::zeros(),
            DynamicsScratch {
                transforms: &mut workspace.frames,
                angular_velocities: &mut workspace.angular_velocities,
                angular_accelerations: &mut workspace.angular_accelerations,
                origin_accelerations: &mut workspace.origin_accelerations,
                link_accelerations: &mut workspace.link_accelerations,
                link_loads: &mut workspace.link_loads,
            },
            &mut output[joint_offset..],
        )?;
        if joint_offset != 0 {
            write_world_wrench(
                self.base.frame(),
                base_load,
                &mut output[..FLOATING_BASE_DOF],
            );
        }
        Ok(())
    }

    /// Writes runtime-sized Newton-Euler generalized forces into caller-owned output.
    ///
    /// Base pose and classical motion come from [`Robot::base`]. Floating-base
    /// output is ordered `[base torque, base force, joint forces]`, with the
    /// base wrench expressed in the world frame at the root origin.
    ///
    /// $$
    /// \tau = M(q) \dot\nu + C(q, \nu) \nu + g(q).
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error for invalid lengths, link IDs, or workspace.
    pub fn inverse_dynamics(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        self.validate_output("inverse dynamics output", output)?;
        self.inverse_dynamics_for_base(
            q,
            qd,
            qdd,
            self.base.frame(),
            self.base.velocity(),
            self.base.acceleration(),
            loads,
            workspace,
            output,
        )
    }

    /// Writes runtime-sized gravity generalized forces into caller-owned output.
    ///
    /// With no external loads, this is the zero-velocity, zero-acceleration
    /// inverse-dynamics term:
    ///
    /// $$
    /// g(q) = \tau(q, 0, 0).
    /// $$
    ///
    /// For a floating base, the leading six outputs are the world-frame root
    /// wrench in torque-then-force order; remaining entries are joint forces.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid lengths, link IDs, or workspace.
    pub fn gravity(
        &self,
        q: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_output("gravity output", output)?;
        self.gravity_for_base(q, self.base.frame(), loads, workspace, output)
    }

    #[allow(clippy::too_many_arguments)]
    fn inverse_dynamics_for_base(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        base_frame: &Frame,
        base_velocity: Twist,
        base_acceleration: Twist,
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        let root_load = self.prepare_indexed_loads(loads, &mut workspace.link_loads)?;
        output.fill(0.0);
        let joint_offset = self.base_dof_count();
        let base_load = self.inverse_dynamics_kernel(
            q,
            qd,
            qdd,
            base_frame,
            base_velocity,
            base_acceleration,
            Vector3::new(0.0, 0.0, GRAVITY),
            root_load,
            DynamicsScratch {
                transforms: &mut workspace.frames,
                angular_velocities: &mut workspace.angular_velocities,
                angular_accelerations: &mut workspace.angular_accelerations,
                origin_accelerations: &mut workspace.origin_accelerations,
                link_accelerations: &mut workspace.link_accelerations,
                link_loads: &mut workspace.link_loads,
            },
            &mut output[joint_offset..],
        )?;
        if joint_offset != 0 {
            write_world_wrench(base_frame, base_load, &mut output[..FLOATING_BASE_DOF]);
        }
        Ok(())
    }

    fn gravity_for_base(
        &self,
        q: &[f64],
        base_frame: &Frame,
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        let root_load = self.prepare_indexed_loads(loads, &mut workspace.link_loads)?;
        output.fill(0.0);
        let joint_offset = self.base_dof_count();
        let base_load = self.gravity_kernel(
            q,
            base_frame,
            root_load,
            GravityScratch {
                transforms: &mut workspace.frames,
                gravity_at_link: &mut workspace.angular_accelerations,
                link_loads: &mut workspace.link_loads,
            },
            &mut output[joint_offset..],
        )?;
        if joint_offset != 0 {
            write_world_wrench(base_frame, base_load, &mut output[..FLOATING_BASE_DOF]);
        }
        Ok(())
    }

    fn prepare_ancestor_path(&self, target_index: usize, path: &mut [usize]) -> usize {
        let mut current = target_index;
        let mut depth = 0;
        while current != 0 {
            let joint_index = current - 1;
            path[depth] = joint_index;
            depth += 1;
            current = self.joint_parents[joint_index];
        }
        depth
    }

    fn target_frame_kernel(&self, q: &[f64], path: &[usize]) -> Result<Frame> {
        self.validate_slice("q", q)?;
        let mut frame = Frame::identity();
        for &joint_index in path.iter().rev() {
            frame *= self.joint_kinematics[joint_index].frame(self.joint_value(q, joint_index));
        }
        Ok(frame)
    }

    fn forward_kinematics_and_jacobian_kernel(
        &self,
        q: &[f64],
        target_index: usize,
        frames: &mut [Frame],
        output: &mut [f64],
        path: &mut [usize],
        clear_output: bool,
    ) -> Result<Frame> {
        let depth = self.prepare_ancestor_path(target_index, path);
        self.target_frames_kernel(q, &path[..depth], frames)?;
        self.jacobian_kernel(frames, target_index, &path[..depth], output, clear_output)
    }

    fn target_frames_kernel(&self, q: &[f64], path: &[usize], frames: &mut [Frame]) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice_length("frame workspace", frames.len(), self.model_joint_count())?;
        let mut frame = Frame::identity();
        for &joint_index in path.iter().rev() {
            frame *= self.joint_kinematics[joint_index].frame(self.joint_value(q, joint_index));
            frames[joint_index] = frame;
        }
        Ok(())
    }

    fn jacobian_kernel(
        &self,
        frames: &[Frame],
        target_index: usize,
        path: &[usize],
        output: &mut [f64],
        clear_output: bool,
    ) -> Result<Frame> {
        self.validate_slice_length("frame workspace", frames.len(), self.model_joint_count())?;
        self.validate_slice_length("jacobian output", output.len(), 6 * self.joint_count())?;
        if clear_output {
            output.fill(0.0);
        }
        let target_frame = frame_for_target(frames, target_index);
        for &joint_index in path {
            let Some(dof_index) = self.joint_dof_indices[joint_index] else {
                continue;
            };
            let joint_frame = frames[joint_index];
            let column = &mut output[6 * dof_index..6 * dof_index + 6];
            let joint = self.joint_kinematics[joint_index];
            match joint.joint_type {
                JointType::Revolute => {
                    let axis = joint_frame.rotation * joint.axis.as_ref();
                    let linear = axis
                        .cross(&(target_frame.translation.vector - joint_frame.translation.vector));
                    column[..3].copy_from_slice(axis.as_slice());
                    column[3..].copy_from_slice(linear.as_slice());
                }
                JointType::Prismatic => {
                    let axis = joint_frame.rotation * joint.axis.as_ref();
                    column[3..].copy_from_slice(axis.as_slice());
                }
                JointType::Fixed => unreachable!("fixed joints have no DOF index"),
            }
        }
        Ok(target_frame)
    }

    fn write_generalized_jacobian(
        &self,
        joint_jacobian: &[f64],
        local_target: &Frame,
        output: &mut [f64],
    ) {
        output.fill(0.0);
        let base_columns = self.base_dof_count();
        let rotation = self.base.frame().rotation;
        if base_columns != 0 {
            let offset = rotation * local_target.translation.vector;
            for axis_index in 0..3 {
                let axis = Vector3::ith(axis_index, 1.0);
                let angular_column = &mut output[6 * axis_index..6 * axis_index + 6];
                angular_column[..3].copy_from_slice(axis.as_slice());
                angular_column[3..].copy_from_slice(axis.cross(&offset).as_slice());
                output[6 * (axis_index + 3) + 3 + axis_index] = 1.0;
            }
        }
        for dof_index in 0..self.joint_count() {
            let source = &joint_jacobian[6 * dof_index..6 * dof_index + 6];
            let angular = rotation * Vector3::from_column_slice(&source[..3]);
            let linear = rotation * Vector3::from_column_slice(&source[3..]);
            let column_index = base_columns + dof_index;
            let column = &mut output[6 * column_index..6 * column_index + 6];
            column[..3].copy_from_slice(angular.as_slice());
            column[3..].copy_from_slice(linear.as_slice());
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn joint_jacobian_derivative_kernel(
        &self,
        q: &[f64],
        qd: &[f64],
        target_index: usize,
        frames: &mut [Frame],
        angular_velocities: &mut [Vector3<f64>],
        origin_velocities: &mut [Vector3<f64>],
        path_workspace: &mut [usize],
        output: &mut [f64],
    ) -> Result<()> {
        output.fill(0.0);
        if target_index == 0 {
            return Ok(());
        }
        let depth = self.prepare_ancestor_path(target_index, path_workspace);
        let path = &path_workspace[..depth];
        self.target_frames_kernel(q, path, frames)?;
        let mut angular = Vector3::zeros();
        let mut linear = Vector3::zeros();
        let mut parent_frame = Frame::identity();
        for &joint_index in path.iter().rev() {
            let joint = self.joint_kinematics[joint_index];
            let frame = frames[joint_index];
            let offset = frame.translation.vector - parent_frame.translation.vector;
            let axis: Vector3<f64> = frame.rotation * joint.axis.as_ref();
            let mut child_angular = angular;
            let mut child_linear = linear + angular.cross(&offset);
            let velocity = self.joint_value(qd, joint_index);
            match joint.joint_type {
                JointType::Revolute => child_angular += axis * velocity,
                JointType::Prismatic => child_linear += axis * velocity,
                JointType::Fixed => {}
            }
            angular_velocities[joint_index] = child_angular;
            origin_velocities[joint_index] = child_linear;
            angular = child_angular;
            linear = child_linear;
            parent_frame = frame;
        }
        let target_frame = frame_for_target(frames, target_index);
        let end_position = target_frame.translation.vector;
        let end_velocity = linear;
        for &joint_index in path {
            let Some(dof_index) = self.joint_dof_indices[joint_index] else {
                continue;
            };
            let joint = self.joint_kinematics[joint_index];
            let frame = frames[joint_index];
            let axis: Vector3<f64> = frame.rotation * joint.axis.as_ref();
            let axis_rate = angular_velocities[joint_index].cross(&axis);
            let column = &mut output[6 * dof_index..6 * dof_index + 6];
            match joint.joint_type {
                JointType::Revolute => {
                    let moment_arm = end_position - frame.translation.vector;
                    let origin_velocity = origin_velocities[joint_index];
                    let linear_rate = axis_rate.cross(&moment_arm)
                        + axis.cross(&(end_velocity - origin_velocity));
                    column[..3].copy_from_slice(axis_rate.as_slice());
                    column[3..].copy_from_slice(linear_rate.as_slice());
                }
                JointType::Prismatic => column[3..].copy_from_slice(axis_rate.as_slice()),
                JointType::Fixed => unreachable!("fixed joints have no DOF index"),
            }
        }
        Ok(())
    }

    fn write_generalized_jacobian_derivative(
        &self,
        qd: &[f64],
        local_target: &Frame,
        joint_jacobian: &[f64],
        joint_derivative: &[f64],
        output: &mut [f64],
    ) {
        output.fill(0.0);
        let base_columns = self.base_dof_count();
        let rotation = self.base.frame().rotation;
        let base_omega = self.base.velocity().angular;
        let mut local_velocity = Vector3::zeros();
        for (joint_index, &velocity) in qd.iter().enumerate() {
            let column = &joint_jacobian[6 * joint_index..6 * joint_index + 6];
            local_velocity += Vector3::from_column_slice(&column[3..]) * velocity;
        }
        if base_columns != 0 {
            let offset = rotation * local_target.translation.vector;
            let offset_rate = base_omega.cross(&offset) + rotation * local_velocity;
            for axis_index in 0..3 {
                let axis = Vector3::ith(axis_index, 1.0);
                output[6 * axis_index + 3..6 * axis_index + 6]
                    .copy_from_slice(axis.cross(&offset_rate).as_slice());
            }
        }
        for dof_index in 0..self.joint_count() {
            let source = &joint_jacobian[6 * dof_index..6 * dof_index + 6];
            let derivative = &joint_derivative[6 * dof_index..6 * dof_index + 6];
            let world_angular = rotation * Vector3::from_column_slice(&source[..3]);
            let world_linear = rotation * Vector3::from_column_slice(&source[3..]);
            let angular = base_omega.cross(&world_angular)
                + rotation * Vector3::from_column_slice(&derivative[..3]);
            let linear = base_omega.cross(&world_linear)
                + rotation * Vector3::from_column_slice(&derivative[3..]);
            let column_index = base_columns + dof_index;
            let column = &mut output[6 * column_index..6 * column_index + 6];
            column[..3].copy_from_slice(angular.as_slice());
            column[3..].copy_from_slice(linear.as_slice());
        }
    }

    #[inline]
    #[allow(clippy::too_many_arguments)]
    fn forward_velocity_for_base(
        &self,
        q: &[f64],
        qd: &[f64],
        target_index: usize,
        tool: &Frame,
        base_frame: &Frame,
        base_velocity: Twist,
        path: &mut [usize],
    ) -> Twist {
        let depth = self.prepare_ancestor_path(target_index, path);
        let (local_target, local_velocity) =
            self.velocity_for_joints(q, qd, path[..depth].iter().rev().copied(), tool);
        let offset = base_frame.rotation
            * (local_target.translation.vector + local_target.rotation * tool.translation.vector);
        let relative_angular = base_frame.rotation * local_velocity.angular;
        let relative_linear = base_frame.rotation * local_velocity.linear;
        Twist::new(
            base_velocity.angular + relative_angular,
            base_velocity.linear + base_velocity.angular.cross(&offset) + relative_linear,
        )
    }

    #[inline]
    fn velocity_for_joints(
        &self,
        q: &[f64],
        qd: &[f64],
        joint_indices: impl Iterator<Item = usize>,
        tool: &Frame,
    ) -> (Frame, Twist) {
        let mut frame = Frame::identity();
        let mut angular = Vector3::zeros();
        let mut linear = Vector3::zeros();
        for joint_index in joint_indices {
            let parent_position = frame.translation.vector;
            let joint = self.joint_kinematics[joint_index];
            frame *= joint.frame(self.joint_value(q, joint_index));
            linear += angular.cross(&(frame.translation.vector - parent_position));
            let axis = frame.rotation * joint.axis.as_ref();
            let velocity = self.joint_value(qd, joint_index);
            match joint.joint_type {
                JointType::Revolute => angular += axis * velocity,
                JointType::Prismatic => linear += axis * velocity,
                JointType::Fixed => {}
            }
        }
        linear += angular.cross(&(frame.rotation * tool.translation.vector));
        (frame, Twist::new(angular, linear))
    }

    #[inline]
    fn motion_for_joints(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        joint_indices: impl Iterator<Item = usize>,
    ) -> (Frame, Twist, Twist) {
        let mut frame = Frame::identity();
        let mut omega = Vector3::zeros();
        let mut velocity = Vector3::zeros();
        let mut alpha = Vector3::zeros();
        let mut acceleration = Vector3::zeros();
        for joint_index in joint_indices {
            let parent_position = frame.translation.vector;
            let joint = self.joint_kinematics[joint_index];
            frame *= joint.frame(self.joint_value(q, joint_index));
            let offset = frame.translation.vector - parent_position;
            let mut child_omega = omega;
            let mut child_velocity = velocity + omega.cross(&offset);
            let mut child_alpha = alpha;
            let mut child_acceleration =
                acceleration + alpha.cross(&offset) + omega.cross(&omega.cross(&offset));
            let axis = frame.rotation * joint.axis.as_ref();
            let joint_velocity = self.joint_value(qd, joint_index);
            let acceleration_value = self.joint_value(qdd, joint_index);
            match joint.joint_type {
                JointType::Revolute => {
                    child_alpha += axis * acceleration_value + omega.cross(&axis) * joint_velocity;
                    child_omega += axis * joint_velocity;
                }
                JointType::Prismatic => {
                    child_velocity += axis * joint_velocity;
                    child_acceleration +=
                        axis * acceleration_value + 2.0 * joint_velocity * omega.cross(&axis);
                }
                JointType::Fixed => {}
            }
            omega = child_omega;
            velocity = child_velocity;
            alpha = child_alpha;
            acceleration = child_acceleration;
        }
        (
            frame,
            Twist::new(omega, velocity),
            Twist::new(alpha, acceleration),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn inverse_dynamics_kernel(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        base_frame: &Frame,
        base_velocity: Twist,
        base_acceleration: Twist,
        world_gravity: Vector3<f64>,
        root_load: Wrench,
        scratch: DynamicsScratch<'_>,
        output: &mut [f64],
    ) -> Result<Wrench> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        self.validate_dynamics_scratch(&scratch)?;
        self.validate_joint_output("inverse dynamics joint output", output)?;
        let base_rotation_inverse = base_frame.rotation.inverse();
        let base_omega = base_rotation_inverse * base_velocity.angular;
        let base_angular_acceleration = base_rotation_inverse * base_acceleration.angular;
        let base_origin_acceleration =
            base_rotation_inverse * (world_gravity + base_acceleration.linear);

        for i in 0..self.model_joint_count() {
            let joint = self.joint_kinematics[i];
            let link = self.link_dynamics[i + 1];
            let parent = self.joint_parents[i];
            let (parent_omega, parent_alpha, parent_acceleration) = if parent == 0 {
                (
                    base_omega,
                    base_angular_acceleration,
                    base_origin_acceleration,
                )
            } else {
                (
                    scratch.angular_velocities[parent - 1],
                    scratch.angular_accelerations[parent - 1],
                    scratch.origin_accelerations[parent - 1],
                )
            };
            let position = self.joint_value(q, i);
            let velocity = self.joint_value(qd, i);
            let acceleration_value = self.joint_value(qdd, i);
            let transform = joint.frame(position);
            let rotation_inverse = transform.rotation.inverse();
            let translation = transform.translation.vector;
            let axis = joint.axis.as_ref();
            let rotated_omega = rotation_inverse * parent_omega;
            let rotated_alpha = rotation_inverse * parent_alpha;
            let translated_acceleration = rotation_inverse
                * (parent_acceleration
                    + parent_alpha.cross(&translation)
                    + parent_omega.cross(&parent_omega.cross(&translation)));
            let (omega, alpha, acceleration) = match joint.joint_type {
                JointType::Revolute => {
                    let alpha = rotated_alpha
                        + acceleration_value * axis
                        + rotated_omega.cross(&(velocity * axis));
                    (
                        rotated_omega + velocity * axis,
                        alpha,
                        translated_acceleration,
                    )
                }
                JointType::Prismatic => (
                    rotated_omega,
                    rotated_alpha,
                    translated_acceleration
                        + acceleration_value * axis
                        + 2.0 * velocity * rotated_omega.cross(axis),
                ),
                JointType::Fixed => (rotated_omega, rotated_alpha, translated_acceleration),
            };
            scratch.angular_velocities[i] = omega;
            scratch.angular_accelerations[i] = alpha;
            scratch.origin_accelerations[i] = acceleration;
            let center = &link.center_of_mass;
            scratch.link_accelerations[i] =
                acceleration + alpha.cross(center) + omega.cross(&omega.cross(center));
            scratch.transforms[i] = transform;
        }

        let root = self.link_dynamics[0];
        let root_center_acceleration = base_origin_acceleration
            + base_angular_acceleration.cross(&root.center_of_mass)
            + base_omega.cross(&base_omega.cross(&root.center_of_mass));
        let root_force = root.mass * root_center_acceleration;
        let mut accumulated_root_load = add_wrench(
            root_load,
            Wrench::new(
                root.center_of_mass.cross(&root_force)
                    + root.inertia * base_angular_acceleration
                    + base_omega.cross(&(root.inertia * base_omega)),
                root_force,
            ),
        );
        for i in (0..self.model_joint_count()).rev() {
            let joint = self.joint_kinematics[i];
            let link = self.link_dynamics[i + 1];
            let inertial_force = link.mass * scratch.link_accelerations[i];
            let angular_momentum = link.inertia * scratch.angular_velocities[i];
            let inertial_load = Wrench::new(
                link.center_of_mass.cross(&inertial_force)
                    + link.inertia * scratch.angular_accelerations[i]
                    + scratch.angular_velocities[i].cross(&angular_momentum),
                inertial_force,
            );
            scratch.link_loads[i] = add_wrench(scratch.link_loads[i], inertial_load);
            if let Some(dof_index) = self.joint_dof_indices[i] {
                output[dof_index] = match joint.joint_type {
                    JointType::Revolute => scratch.link_loads[i].torque.dot(joint.axis.as_ref()),
                    JointType::Prismatic => scratch.link_loads[i].force.dot(joint.axis.as_ref()),
                    JointType::Fixed => unreachable!("fixed joints have no DOF index"),
                };
            }
            let parent = self.joint_parents[i];
            if parent != 0 {
                let parent_load = wrench_to_parent(&scratch.transforms[i], scratch.link_loads[i]);
                scratch.link_loads[parent - 1] =
                    add_wrench(scratch.link_loads[parent - 1], parent_load);
            } else {
                accumulated_root_load = add_wrench(
                    accumulated_root_load,
                    wrench_to_parent(&scratch.transforms[i], scratch.link_loads[i]),
                );
            }
        }
        Ok(accumulated_root_load)
    }

    fn mass_matrix_kernel(&self, q: &[f64], workspace: &mut Workspace, output: &mut [f64]) {
        let joint_count = self.joint_count();
        let model_joint_count = self.model_joint_count();
        output.fill(0.0);
        for index in 0..model_joint_count {
            workspace.frames[index] =
                self.joint_kinematics[index].frame(self.joint_value(q, index));
            let link = self.link_dynamics[index + 1];
            workspace.composite_masses[index] = link.mass;
            workspace.composite_moments[index] = link.first_moment;
            workspace.composite_inertias[index] = link.origin_inertia;
        }
        // Composite rigid-body pass: accumulate each subtree inertia, expressed
        // about the parent link origin, into the parent.
        for index in (0..model_joint_count).rev() {
            let parent = self.joint_parents[index];
            if parent == 0 {
                continue;
            }
            let transform = &workspace.frames[index];
            let translation = transform.translation.vector;
            let rotation = transform.rotation.to_rotation_matrix();
            let rotated_moment = rotation * workspace.composite_moments[index];
            let rotated_inertia =
                rotation * workspace.composite_inertias[index] * rotation.transpose();
            let mass = workspace.composite_masses[index];
            let parent_index = parent - 1;
            workspace.composite_masses[parent_index] += mass;
            workspace.composite_moments[parent_index] += mass * translation + rotated_moment;
            // R I_o R^T - m[t]x[t]x - [t]x[h]x - [h]x[t]x with h = R h_child.
            workspace.composite_inertias[parent_index] += rotated_inertia
                + (mass * translation.norm_squared() + 2.0 * translation.dot(&rotated_moment))
                    * Matrix3::identity()
                - mass * translation * translation.transpose()
                - translation * rotated_moment.transpose()
                - rotated_moment * translation.transpose();
        }
        // Mass-matrix entries: F = I^c S in the child link frame, then F is
        // propagated up the ancestor chain while M(i, j) = S_j^T F.
        for &index in self.active_joint_indices.iter() {
            let dof_index = self.joint_dof_indices[index].expect("active joint has a DOF index");
            let joint = self.joint_kinematics[index];
            let axis: Vector3<f64> = *joint.axis.as_ref();
            let mass = workspace.composite_masses[index];
            let moment = workspace.composite_moments[index];
            let inertia = workspace.composite_inertias[index];
            let mut force = match joint.joint_type {
                JointType::Revolute => Wrench::new(inertia * axis, axis.cross(&moment)),
                JointType::Prismatic => Wrench::new(moment.cross(&axis), mass * axis),
                JointType::Fixed => unreachable!("fixed joints were skipped above"),
            };
            let mut current = index;
            loop {
                let current_joint = self.joint_kinematics[current];
                let current_axis: Vector3<f64> = *current_joint.axis.as_ref();
                let entry = match current_joint.joint_type {
                    JointType::Revolute => current_axis.dot(&force.torque),
                    JointType::Prismatic => current_axis.dot(&force.force),
                    JointType::Fixed => 0.0,
                };
                if let Some(current_dof) = self.joint_dof_indices[current] {
                    output[current_dof * joint_count + dof_index] = entry;
                    output[dof_index * joint_count + current_dof] = entry;
                }
                let parent = self.joint_parents[current];
                if parent == 0 {
                    break;
                }
                force = wrench_to_parent(&workspace.frames[current], force);
                current = parent - 1;
            }
        }
    }

    fn floating_mass_matrix_kernel(
        &self,
        q: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) {
        let model_joint_count = self.model_joint_count();
        let generalized_count = self.generalized_count();
        output.fill(0.0);

        let root = self.link_dynamics[0];
        let mut root_mass = root.mass;
        let mut root_moment = root.first_moment;
        let mut root_inertia = root.origin_inertia;
        for index in 0..model_joint_count {
            workspace.frames[index] =
                self.joint_kinematics[index].frame(self.joint_value(q, index));
            let link = self.link_dynamics[index + 1];
            workspace.composite_masses[index] = link.mass;
            workspace.composite_moments[index] = link.first_moment;
            workspace.composite_inertias[index] = link.origin_inertia;
        }
        for index in (0..model_joint_count).rev() {
            let transform = &workspace.frames[index];
            let translation = transform.translation.vector;
            let rotation = transform.rotation.to_rotation_matrix();
            let rotated_moment = rotation * workspace.composite_moments[index];
            let rotated_inertia =
                rotation * workspace.composite_inertias[index] * rotation.transpose();
            let mass = workspace.composite_masses[index];
            let transformed_moment = mass * translation + rotated_moment;
            let transformed_inertia = rotated_inertia
                + (mass * translation.norm_squared() + 2.0 * translation.dot(&rotated_moment))
                    * Matrix3::identity()
                - mass * translation * translation.transpose()
                - translation * rotated_moment.transpose()
                - rotated_moment * translation.transpose();
            let parent = self.joint_parents[index];
            if parent == 0 {
                root_mass += mass;
                root_moment += transformed_moment;
                root_inertia += transformed_inertia;
            } else {
                let parent_index = parent - 1;
                workspace.composite_masses[parent_index] += mass;
                workspace.composite_moments[parent_index] += transformed_moment;
                workspace.composite_inertias[parent_index] += transformed_inertia;
            }
        }

        let base_rotation = self.base.frame().rotation;
        for column in 0..FLOATING_BASE_DOF {
            let world_axis = Vector3::ith(column % 3, 1.0);
            let local_axis = base_rotation.inverse() * world_axis;
            let local_load = if column < 3 {
                Wrench::new(root_inertia * local_axis, local_axis.cross(&root_moment))
            } else {
                Wrench::new(root_moment.cross(&local_axis), root_mass * local_axis)
            };
            let world_load = Wrench::new(
                base_rotation * local_load.torque,
                base_rotation * local_load.force,
            );
            write_wrench_to_column(output, generalized_count, column, world_load);
        }

        for &index in self.active_joint_indices.iter() {
            let dof_index = self.joint_dof_indices[index].expect("active joint has a DOF index");
            let joint = self.joint_kinematics[index];
            let axis: Vector3<f64> = *joint.axis.as_ref();
            let mass = workspace.composite_masses[index];
            let moment = workspace.composite_moments[index];
            let inertia = workspace.composite_inertias[index];
            let mut force = match joint.joint_type {
                JointType::Revolute => Wrench::new(inertia * axis, axis.cross(&moment)),
                JointType::Prismatic => Wrench::new(moment.cross(&axis), mass * axis),
                JointType::Fixed => unreachable!("fixed joints were skipped"),
            };
            let joint_column = FLOATING_BASE_DOF + dof_index;
            let mut current = index;
            loop {
                let current_joint = self.joint_kinematics[current];
                let current_axis: Vector3<f64> = *current_joint.axis.as_ref();
                let entry = match current_joint.joint_type {
                    JointType::Revolute => current_axis.dot(&force.torque),
                    JointType::Prismatic => current_axis.dot(&force.force),
                    JointType::Fixed => 0.0,
                };
                if let Some(current_dof) = self.joint_dof_indices[current] {
                    let current_row = FLOATING_BASE_DOF + current_dof;
                    output[joint_column * generalized_count + current_row] = entry;
                    output[current_row * generalized_count + joint_column] = entry;
                }
                let parent = self.joint_parents[current];
                force = wrench_to_parent(&workspace.frames[current], force);
                if parent == 0 {
                    break;
                }
                current = parent - 1;
            }
            let world_force =
                Wrench::new(base_rotation * force.torque, base_rotation * force.force);
            for base_row in 0..FLOATING_BASE_DOF {
                let entry = wrench_component(world_force, base_row);
                output[joint_column * generalized_count + base_row] = entry;
                output[base_row * generalized_count + joint_column] = entry;
            }
        }
    }

    fn gravity_kernel(
        &self,
        q: &[f64],
        base_frame: &Frame,
        root_load: Wrench,
        scratch: GravityScratch<'_>,
        output: &mut [f64],
    ) -> Result<Wrench> {
        self.validate_slice("q", q)?;
        self.validate_slice_length(
            "transform workspace",
            scratch.transforms.len(),
            self.model_joint_count(),
        )?;
        self.validate_slice_length(
            "gravity workspace",
            scratch.gravity_at_link.len(),
            self.model_joint_count(),
        )?;
        self.validate_slice_length(
            "load workspace",
            scratch.link_loads.len(),
            self.model_joint_count(),
        )?;
        self.validate_joint_output("gravity joint output", output)?;
        let base_gravity = base_frame.rotation.inverse() * Vector3::new(0.0, 0.0, GRAVITY);
        for i in 0..self.model_joint_count() {
            scratch.transforms[i] = self.joint_kinematics[i].frame(self.joint_value(q, i));
            let parent = self.joint_parents[i];
            let parent_gravity = if parent == 0 {
                base_gravity
            } else {
                scratch.gravity_at_link[parent - 1]
            };
            scratch.gravity_at_link[i] = scratch.transforms[i].rotation.inverse() * parent_gravity;
        }
        let root = self.link_dynamics[0];
        let root_force = root.mass * base_gravity;
        let mut accumulated_root_load = add_wrench(
            root_load,
            Wrench::new(root.center_of_mass.cross(&root_force), root_force),
        );
        for i in (0..self.model_joint_count()).rev() {
            let joint = self.joint_kinematics[i];
            let link = self.link_dynamics[i + 1];
            let force = link.mass * scratch.gravity_at_link[i];
            let gravity_load = Wrench::new(link.center_of_mass.cross(&force), force);
            scratch.link_loads[i] = add_wrench(scratch.link_loads[i], gravity_load);
            if let Some(dof_index) = self.joint_dof_indices[i] {
                output[dof_index] = match joint.joint_type {
                    JointType::Revolute => scratch.link_loads[i].torque.dot(joint.axis.as_ref()),
                    JointType::Prismatic => scratch.link_loads[i].force.dot(joint.axis.as_ref()),
                    JointType::Fixed => unreachable!("fixed joints have no DOF index"),
                };
            }
            let parent = self.joint_parents[i];
            if parent != 0 {
                let parent_load = wrench_to_parent(&scratch.transforms[i], scratch.link_loads[i]);
                scratch.link_loads[parent - 1] =
                    add_wrench(scratch.link_loads[parent - 1], parent_load);
            } else {
                accumulated_root_load = add_wrench(
                    accumulated_root_load,
                    wrench_to_parent(&scratch.transforms[i], scratch.link_loads[i]),
                );
            }
        }
        Ok(accumulated_root_load)
    }

    #[allow(clippy::too_many_arguments)]
    fn inverse_kinematics_kernel(
        &self,
        initial_q: &[f64],
        target_index: usize,
        desired: &Frame,
        options: InverseKinematicsOptions,
        frames: &mut [Frame],
        jacobian: &mut [f64],
        q_work: &mut [f64],
        step: &mut [f64],
        path: &mut [usize],
    ) -> Result<()> {
        self.validate_slice("initial_q", initial_q)?;
        self.validate_joint_output("IK joint workspace", q_work)?;
        self.validate_joint_output("IK step workspace", step)?;
        validate_inverse_kinematics_options(options)?;
        if !initial_q.iter().all(|value| value.is_finite()) {
            return Err(Error::NonFiniteIkInput {
                input: "initial joint vector",
            });
        }
        if !desired
            .translation
            .vector
            .iter()
            .chain(desired.rotation.coords.iter())
            .all(|value| value.is_finite())
        {
            return Err(Error::NonFiniteIkInput {
                input: "target frame",
            });
        }
        q_work.copy_from_slice(initial_q);
        let depth = self.prepare_ancestor_path(target_index, path);
        let path = &path[..depth];
        jacobian.fill(0.0);
        let damping_squared = options.damping * options.damping;
        for iteration in 0..=options.max_iterations {
            self.target_frames_kernel(q_work, path, frames)?;
            let current = self.jacobian_kernel(frames, target_index, path, jacobian, false)?;
            let translation_error = desired.translation.vector - current.translation.vector;
            let rotation_error = (desired.rotation * current.rotation.inverse()).scaled_axis();
            let translation_error_norm = translation_error.norm();
            let rotation_error_norm = rotation_error.norm();
            if translation_error_norm <= options.translation_tolerance
                && rotation_error_norm <= options.rotation_tolerance
            {
                self.validate_inverse_kinematics_solution(q_work)?;
                return Ok(());
            }
            if iteration == options.max_iterations {
                return Err(Error::IkNotConverged {
                    iterations: options.max_iterations,
                    translation_error: translation_error_norm,
                    rotation_error: rotation_error_norm,
                });
            }
            let error = SVector::<f64, 6>::from_iterator(
                rotation_error
                    .iter()
                    .chain(translation_error.iter())
                    .copied(),
            );
            let mut regularized = SMatrix::<f64, 6, 6>::identity() * damping_squared;
            for &joint_index in path.iter().rev() {
                let Some(dof_index) = self.joint_dof_indices[joint_index] else {
                    continue;
                };
                let column = &jacobian[6 * dof_index..6 * dof_index + 6];
                for row in 0..6 {
                    for col in 0..=row {
                        regularized[(row, col)] += column[row] * column[col];
                    }
                }
            }
            // nalgebra's Cholesky decomposition reads only the lower triangle.
            let Some(weighted_error) = regularized.cholesky().map(|factor| factor.solve(&error))
            else {
                return Err(Error::IkNumericalFailure {
                    iteration: iteration + 1,
                });
            };
            let mut step_norm_squared = 0.0;
            for &joint_index in path.iter().rev() {
                let Some(dof_index) = self.joint_dof_indices[joint_index] else {
                    continue;
                };
                let column = &jacobian[6 * dof_index..6 * dof_index + 6];
                step[dof_index] = column
                    .iter()
                    .zip(weighted_error.iter())
                    .map(|(lhs, rhs)| lhs * rhs)
                    .sum();
                step_norm_squared += step[dof_index] * step[dof_index];
            }
            let step_norm = step_norm_squared.sqrt();
            if !step_norm.is_finite() {
                return Err(Error::IkNumericalFailure {
                    iteration: iteration + 1,
                });
            }
            let scale = if step_norm > options.max_step_norm {
                options.max_step_norm / step_norm
            } else {
                1.0
            };
            for &joint_index in path.iter().rev() {
                if let Some(dof_index) = self.joint_dof_indices[joint_index] {
                    q_work[dof_index] += scale * step[dof_index];
                }
            }
        }
        unreachable!("inverse-kinematics loop always returns")
    }

    fn validate_inverse_kinematics_solution(&self, q: &[f64]) -> Result<()> {
        for (&joint_index, &position) in self.active_joint_indices.iter().zip(q) {
            let joint = &self.joints[joint_index];
            if joint.is_over_limit(position) {
                return Err(Error::IkJointLimitViolation {
                    joint_index,
                    joint: joint.name().to_owned(),
                    position,
                    lower: joint.lower_limit(),
                    upper: joint.upper_limit(),
                });
            }
        }
        Ok(())
    }

    fn prepare_indexed_loads(
        &self,
        loads: &[IndexedLoad],
        output: &mut [Wrench],
    ) -> Result<Wrench> {
        output.fill(Wrench::zeros());
        let mut root_load = Wrench::zeros();
        for load in loads {
            let link_index = self.validate_link_id(load.link)?;
            if link_index == 0 {
                root_load = add_wrench(root_load, load.wrench);
            } else {
                output[link_index - 1] = add_wrench(output[link_index - 1], load.wrench);
            }
        }
        Ok(root_load)
    }

    fn validate_dynamics_scratch(&self, scratch: &DynamicsScratch<'_>) -> Result<()> {
        for (name, actual) in [
            ("transform workspace", scratch.transforms.len()),
            (
                "angular velocity workspace",
                scratch.angular_velocities.len(),
            ),
            (
                "angular acceleration workspace",
                scratch.angular_accelerations.len(),
            ),
            (
                "origin acceleration workspace",
                scratch.origin_accelerations.len(),
            ),
            (
                "link acceleration workspace",
                scratch.link_accelerations.len(),
            ),
            ("load workspace", scratch.link_loads.len()),
        ] {
            self.validate_slice_length(name, actual, self.model_joint_count())?;
        }
        Ok(())
    }
}

fn validate_inverse_kinematics_options(options: InverseKinematicsOptions) -> Result<()> {
    if options.max_iterations == 0 {
        return Err(Error::InvalidIkOptions {
            option: "max_iterations",
            reason: "must be greater than zero",
        });
    }
    for (name, value) in [
        ("translation_tolerance", options.translation_tolerance),
        ("rotation_tolerance", options.rotation_tolerance),
        ("damping", options.damping),
        ("max_step_norm", options.max_step_norm),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return Err(Error::InvalidIkOptions {
                option: name,
                reason: "must be finite and greater than zero",
            });
        }
    }
    Ok(())
}

fn frame_for_target(frames: &[Frame], target_index: usize) -> Frame {
    if target_index == 0 {
        Frame::identity()
    } else {
        frames[target_index - 1]
    }
}

fn wrench_to_parent(transform: &Frame, wrench: Wrench) -> Wrench {
    let force = transform.rotation * wrench.force;
    Wrench::new(
        transform.rotation * wrench.torque + transform.translation.vector.cross(&force),
        force,
    )
}

fn add_wrench(lhs: Wrench, rhs: Wrench) -> Wrench {
    Wrench::new(lhs.torque + rhs.torque, lhs.force + rhs.force)
}

fn write_world_wrench(base_frame: &Frame, local: Wrench, output: &mut [f64]) {
    let torque = base_frame.rotation * local.torque;
    let force = base_frame.rotation * local.force;
    output[..3].copy_from_slice(torque.as_slice());
    output[3..6].copy_from_slice(force.as_slice());
}

fn wrench_component(wrench: Wrench, index: usize) -> f64 {
    if index < 3 {
        wrench.torque[index]
    } else {
        wrench.force[index - 3]
    }
}

fn write_wrench_to_column(output: &mut [f64], rows: usize, column: usize, wrench: Wrench) {
    for row in 0..FLOATING_BASE_DOF {
        output[column * rows + row] = wrench_component(wrench, row);
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
