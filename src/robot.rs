use std::{
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
};

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::{Error, Frame, Joint, JointType, Link, Result, Twist, Wrench, urdf::tree_model};

mod workspace;

pub use workspace::{IndexedLoad, LinkId, Workspace};

const GRAVITY: f64 = 9.80665;
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
    joint_parents: Box<[usize]>,
    link_depths: Box<[usize]>,
    leaf_links: Box<[usize]>,
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
        let robot = urdf_rs::read_file(path)?;
        let model = tree_model(&robot)?;
        let model_id = NEXT_MODEL_ID.fetch_add(1, Ordering::Relaxed);
        assert_ne!(
            model_id, UNOWNED_MODEL_ID,
            "robot model identifier overflow"
        );
        let mut link_depths = vec![0; model.links.len()];
        for (joint_index, &parent) in model.joint_parents.iter().enumerate() {
            link_depths[joint_index + 1] = link_depths[parent] + 1;
        }
        Ok(Self {
            model_id,
            name: robot.name,
            joints: model.joints.into_boxed_slice(),
            links: model.links.into_boxed_slice(),
            joint_parents: model.joint_parents.into_boxed_slice(),
            link_depths: link_depths.into_boxed_slice(),
            leaf_links: model.leaf_links.into_boxed_slice(),
        })
    }

    /// Returns the robot name declared in the URDF.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns all links in topological order, starting with the root link.
    pub fn links(&self) -> &[Link] {
        &self.links
    }

    /// Returns all joints in the same topological order as their child links.
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

    /// Returns the number of joints in the model.
    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Allocates reusable storage for runtime-sized calculations on this model.
    pub fn workspace(&self) -> Workspace {
        Workspace::new(self.model_id, self.joint_count())
    }

    fn validate_slice(&self, name: &'static str, slice: &[f64]) -> Result<()> {
        self.validate_slice_length(name, slice.len(), self.joint_count())
    }

    fn validate_output(&self, name: &'static str, output: &[f64]) -> Result<()> {
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
        if workspace.model_id == self.model_id && workspace.joint_count == self.joint_count() {
            Ok(())
        } else {
            Err(Error::InvalidWorkspace)
        }
    }

    /// Computes a target link frame using runtime-sized input and workspace.
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
        self.target_frame_kernel(q, &workspace.ancestor_path[..depth])
    }

    /// Writes a runtime-sized `6 x N` geometric Jacobian in column-major order.
    ///
    /// Each column stores `[angular_x, angular_y, angular_z, linear_x,
    /// linear_y, linear_z]`.
    ///
    /// # Errors
    ///
    /// Returns an error unless `output.len() == 6 * joint_count()`, or for an
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
        self.validate_slice_length("jacobian output", output.len(), 6 * self.joint_count())?;
        let target_index = self.validate_link_id(target)?;
        self.forward_kinematics_and_jacobian_kernel(
            q,
            target_index,
            &mut workspace.frames,
            output,
            &mut workspace.ancestor_path,
            true,
        )?;
        Ok(())
    }

    /// Writes a runtime-sized inverse-kinematics solution using the supplied options.
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
        self.validate_slice("initial_q", initial_q)?;
        self.validate_output("inverse kinematics output", output)?;
        let target_index = self.validate_link_id(target)?;
        self.inverse_kinematics_kernel(
            initial_q,
            target_index,
            desired,
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
    /// # Errors
    ///
    /// Returns an error for invalid input lengths, link ID, or workspace.
    pub fn forward_velocity_kinematics(
        &self,
        q: &[f64],
        qd: &[f64],
        target: LinkId,
        base: &Frame,
        tool: &Frame,
        workspace: &mut Workspace,
    ) -> Result<Twist> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        let target_index = self.validate_link_id(target)?;
        self.velocity_kernel(
            q,
            qd,
            target_index,
            base,
            tool,
            &mut workspace.frames,
            &mut workspace.jacobian,
            &mut workspace.ancestor_path,
        )
    }

    /// Computes runtime-sized spatial acceleration of a target link origin.
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
        self.acceleration_kernel(q, qd, qdd, target_index, &mut workspace.ancestor_path)
    }

    /// Writes the runtime-sized `N x N` joint-space mass matrix in column-major
    /// order.
    ///
    /// The matrix is symmetric positive semi-definite. Rows and columns of
    /// fixed joints are zero; their subtree inertia still contributes to the
    /// moving ancestors.
    ///
    /// # Errors
    ///
    /// Returns an error unless `output.len() == joint_count() * joint_count()`,
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
            self.joint_count() * self.joint_count(),
        )?;
        self.mass_matrix_kernel(q, workspace, output);
        Ok(())
    }

    /// Writes the runtime-sized `N x N` Coriolis and centrifugal matrix in
    /// column-major order.
    ///
    /// The matrix uses the Christoffel factorization, so `C(q, qd) qd + g(q)`
    /// equals the RNEA bias `inverse_dynamics(q, qd, 0)` and `dM/dt - 2C` is
    /// skew-symmetric. Rows and columns of fixed joints are zero.
    ///
    /// # Errors
    ///
    /// Returns an error unless `output.len() == joint_count() * joint_count()`,
    /// or for invalid input lengths or workspace.
    pub fn coriolis_matrix(
        &self,
        q: &[f64],
        qd: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice_length(
            "coriolis matrix output",
            output.len(),
            self.joint_count() * self.joint_count(),
        )?;
        self.coriolis_matrix_kernel(q, qd, workspace, output);
        Ok(())
    }

    /// Writes runtime-sized Newton-Euler joint forces into caller-owned output.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid lengths, link IDs, or workspace.
    #[allow(clippy::too_many_arguments)]
    pub fn inverse_dynamics(
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
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        self.validate_output("inverse dynamics output", output)?;
        self.prepare_indexed_loads(loads, &mut workspace.link_loads)?;
        self.inverse_dynamics_kernel(
            q,
            qd,
            qdd,
            base_frame,
            base_velocity,
            base_acceleration,
            DynamicsScratch {
                transforms: &mut workspace.frames,
                angular_velocities: &mut workspace.angular_velocities,
                angular_accelerations: &mut workspace.angular_accelerations,
                origin_accelerations: &mut workspace.origin_accelerations,
                link_accelerations: &mut workspace.link_accelerations,
                link_loads: &mut workspace.link_loads,
            },
            output,
        )
    }

    /// Writes runtime-sized gravity joint forces into caller-owned output.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid lengths, link IDs, or workspace.
    pub fn gravity(
        &self,
        q: &[f64],
        base_frame: &Frame,
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_output("gravity output", output)?;
        self.prepare_indexed_loads(loads, &mut workspace.link_loads)?;
        self.gravity_kernel(
            q,
            base_frame,
            GravityScratch {
                transforms: &mut workspace.frames,
                gravity_at_link: &mut workspace.angular_accelerations,
                link_loads: &mut workspace.link_loads,
            },
            output,
        )
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
            frame *= self.joints[joint_index].frame(q[joint_index]);
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
        self.validate_slice_length("frame workspace", frames.len(), self.joint_count())?;
        let mut frame = Frame::identity();
        for &joint_index in path.iter().rev() {
            frame *= self.joints[joint_index].frame(q[joint_index]);
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
        self.validate_slice_length("frame workspace", frames.len(), self.joint_count())?;
        self.validate_slice_length("jacobian output", output.len(), 6 * self.joint_count())?;
        if clear_output {
            output.fill(0.0);
        }
        let target_frame = frame_for_target(frames, target_index);
        for &joint_index in path {
            let joint_frame = frames[joint_index];
            let column = &mut output[6 * joint_index..6 * joint_index + 6];
            match self.joints[joint_index].joint_type() {
                JointType::Revolute => {
                    let axis = joint_frame.rotation * self.joints[joint_index].axis().as_ref();
                    let linear = axis
                        .cross(&(target_frame.translation.vector - joint_frame.translation.vector));
                    column[..3].copy_from_slice(axis.as_slice());
                    column[3..].copy_from_slice(linear.as_slice());
                }
                JointType::Prismatic => {
                    let axis = joint_frame.rotation * self.joints[joint_index].axis().as_ref();
                    column[3..].copy_from_slice(axis.as_slice());
                }
                JointType::Fixed => {}
            }
        }
        Ok(target_frame)
    }

    #[allow(clippy::too_many_arguments)]
    fn velocity_kernel(
        &self,
        q: &[f64],
        qd: &[f64],
        target_index: usize,
        base: &Frame,
        tool: &Frame,
        frames: &mut [Frame],
        jacobian: &mut [f64],
        path: &mut [usize],
    ) -> Result<Twist> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        if target_index == 0 {
            return Ok(Twist::zeros());
        }
        let depth = self.prepare_ancestor_path(target_index, path);
        let path = &path[..depth];
        self.target_frames_kernel(q, path, frames)?;
        let end = self.jacobian_kernel(frames, target_index, path, jacobian, true)?;
        let offset_world = end.rotation * tool.translation.vector;
        let mut angular = Vector3::zeros();
        let mut linear = Vector3::zeros();
        for &joint_index in path {
            let column = &jacobian[6 * joint_index..6 * joint_index + 6];
            let column_angular = Vector3::new(column[0], column[1], column[2]);
            let column_linear =
                Vector3::new(column[3], column[4], column[5]) + column_angular.cross(&offset_world);
            angular += column_angular * qd[joint_index];
            linear += column_linear * qd[joint_index];
        }
        Ok(Twist::new(base.rotation * angular, base.rotation * linear))
    }

    fn acceleration_kernel(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        target_index: usize,
        path: &mut [usize],
    ) -> Result<Twist> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        if target_index == 0 {
            return Ok(Twist::zeros());
        }
        let target_depth = self.link_depths[target_index];
        if target_depth == target_index {
            return Ok(self.acceleration_for_joints(q, qd, qdd, 0..target_depth));
        }
        let depth = self.prepare_ancestor_path(target_index, path);
        let path = &path[..depth];
        Ok(self.acceleration_for_joints(q, qd, qdd, path.iter().rev().copied()))
    }

    #[inline]
    fn acceleration_for_joints(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        joint_indices: impl Iterator<Item = usize>,
    ) -> Twist {
        let mut frame = Frame::identity();
        let mut omega = Vector3::zeros();
        let mut alpha = Vector3::zeros();
        let mut linear = Vector3::zeros();
        for joint_index in joint_indices {
            let parent_frame = frame;
            frame *= self.joints[joint_index].frame(q[joint_index]);
            let offset = frame.translation.vector - parent_frame.translation.vector;
            let mut child_omega = omega;
            let mut child_alpha = alpha;
            let mut child_linear =
                linear + alpha.cross(&offset) + omega.cross(&omega.cross(&offset));
            match self.joints[joint_index].joint_type() {
                JointType::Revolute => {
                    let axis = frame.rotation * self.joints[joint_index].axis().as_ref();
                    child_alpha += axis * qdd[joint_index] + omega.cross(&axis) * qd[joint_index];
                    child_omega += axis * qd[joint_index];
                }
                JointType::Prismatic => {
                    let axis = frame.rotation * self.joints[joint_index].axis().as_ref();
                    child_linear +=
                        axis * qdd[joint_index] + 2.0 * qd[joint_index] * omega.cross(&axis);
                }
                JointType::Fixed => {}
            }
            omega = child_omega;
            alpha = child_alpha;
            linear = child_linear;
        }
        Twist::new(alpha, linear)
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
        scratch: DynamicsScratch<'_>,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        self.validate_dynamics_scratch(&scratch)?;
        self.validate_output("inverse dynamics output", output)?;
        let base_rotation_inverse = base_frame.rotation.inverse();
        let base_omega = base_rotation_inverse * base_velocity.angular;
        let base_angular_acceleration = base_rotation_inverse * base_acceleration.angular;
        let base_acceleration =
            base_rotation_inverse * (Vector3::new(0.0, 0.0, GRAVITY) + base_acceleration.linear);

        for i in 0..self.joint_count() {
            let joint = &self.joints[i];
            let link = &self.links[i + 1];
            let parent = self.joint_parents[i];
            let (parent_omega, parent_alpha, parent_acceleration) = if parent == 0 {
                (base_omega, base_angular_acceleration, base_acceleration)
            } else {
                (
                    scratch.angular_velocities[parent - 1],
                    scratch.angular_accelerations[parent - 1],
                    scratch.origin_accelerations[parent - 1],
                )
            };
            let transform = joint.frame(q[i]);
            let rotation_inverse = transform.rotation.inverse();
            let translation = transform.translation.vector;
            let axis = joint.axis().as_ref();
            let rotated_omega = rotation_inverse * parent_omega;
            let rotated_alpha = rotation_inverse * parent_alpha;
            let translated_acceleration = rotation_inverse
                * (parent_acceleration
                    + parent_alpha.cross(&translation)
                    + parent_omega.cross(&parent_omega.cross(&translation)));
            let (omega, alpha, acceleration) = match joint.joint_type() {
                JointType::Revolute => {
                    let alpha =
                        rotated_alpha + qdd[i] * axis + rotated_omega.cross(&(qd[i] * axis));
                    (rotated_omega + qd[i] * axis, alpha, translated_acceleration)
                }
                JointType::Prismatic => (
                    rotated_omega,
                    rotated_alpha,
                    translated_acceleration
                        + qdd[i] * axis
                        + 2.0 * qd[i] * rotated_omega.cross(axis),
                ),
                JointType::Fixed => (rotated_omega, rotated_alpha, translated_acceleration),
            };
            scratch.angular_velocities[i] = omega;
            scratch.angular_accelerations[i] = alpha;
            scratch.origin_accelerations[i] = acceleration;
            let center = link.center_of_mass();
            scratch.link_accelerations[i] =
                acceleration + alpha.cross(center) + omega.cross(&omega.cross(center));
            scratch.transforms[i] = transform;
        }

        for i in (0..self.joint_count()).rev() {
            let joint = &self.joints[i];
            let link = &self.links[i + 1];
            let inertial_force = link.mass() * scratch.link_accelerations[i];
            let angular_momentum = link.inertia() * scratch.angular_velocities[i];
            let inertial_load = Wrench::new(
                link.center_of_mass().cross(&inertial_force)
                    + link.inertia() * scratch.angular_accelerations[i]
                    + scratch.angular_velocities[i].cross(&angular_momentum),
                inertial_force,
            );
            scratch.link_loads[i] = add_wrench(scratch.link_loads[i], inertial_load);
            output[i] = joint.active_force(scratch.link_loads[i]);
            let parent = self.joint_parents[i];
            if parent != 0 {
                let parent_load = wrench_to_parent(&scratch.transforms[i], scratch.link_loads[i]);
                scratch.link_loads[parent - 1] =
                    add_wrench(scratch.link_loads[parent - 1], parent_load);
            }
        }
        Ok(())
    }

    fn mass_matrix_kernel(&self, q: &[f64], workspace: &mut Workspace, output: &mut [f64]) {
        let joint_count = self.joint_count();
        output.fill(0.0);
        for (index, joint) in self.joints.iter().enumerate() {
            workspace.frames[index] = joint.frame(q[index]);
            let link = &self.links[index + 1];
            let mass = link.mass();
            let center = *link.center_of_mass();
            workspace.composite_masses[index] = mass;
            workspace.composite_moments[index] = mass * center;
            // Rotational inertia about the link origin: I_o = I_com - m[c]x[c]x.
            workspace.composite_inertias[index] = link.inertia() - mass * skew_square(center);
        }
        // Composite rigid-body pass: accumulate each subtree inertia, expressed
        // about the parent link origin, into the parent.
        for index in (0..joint_count).rev() {
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
        for (index, joint) in self.joints.iter().enumerate() {
            if joint.joint_type() == JointType::Fixed {
                continue;
            }
            let axis: Vector3<f64> = *joint.axis().as_ref();
            let mass = workspace.composite_masses[index];
            let moment = workspace.composite_moments[index];
            let inertia = workspace.composite_inertias[index];
            let mut force = match joint.joint_type() {
                JointType::Revolute => Wrench::new(inertia * axis, axis.cross(&moment)),
                JointType::Prismatic => Wrench::new(moment.cross(&axis), mass * axis),
                JointType::Fixed => unreachable!("fixed joints were skipped above"),
            };
            let mut current = index;
            loop {
                let current_joint = &self.joints[current];
                let current_axis: Vector3<f64> = *current_joint.axis().as_ref();
                let entry = match current_joint.joint_type() {
                    JointType::Revolute => current_axis.dot(&force.torque),
                    JointType::Prismatic => current_axis.dot(&force.force),
                    JointType::Fixed => 0.0,
                };
                output[current * joint_count + index] = entry;
                output[index * joint_count + current] = entry;
                let parent = self.joint_parents[current];
                if parent == 0 {
                    break;
                }
                force = wrench_to_parent(&workspace.frames[current], force);
                current = parent - 1;
            }
        }
    }

    /// Computes `C(q, qd)` as half the velocity-Jacobian of the RNEA bias
    /// force: the Christoffel factorization satisfies `C = (1/2) db/dv` for
    /// `b(q, v) = RNEA(q, v, 0)`, so one directional-derivative pass along
    /// `e_j` produces matrix column `j`. The base pass stores each link's
    /// local transform and angular velocity; gravity drops out because it
    /// does not depend on velocities.
    fn coriolis_matrix_kernel(
        &self,
        q: &[f64],
        qd: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) {
        let joint_count = self.joint_count();
        output.fill(0.0);
        for (index, (&position, &velocity)) in q.iter().zip(qd.iter()).enumerate() {
            let joint = &self.joints[index];
            let transform = joint.frame(position);
            let parent = self.joint_parents[index];
            let parent_omega = if parent == 0 {
                Vector3::zeros()
            } else {
                workspace.angular_velocities[parent - 1]
            };
            let rotated_omega = transform.rotation.inverse() * parent_omega;
            workspace.frames[index] = transform;
            workspace.angular_velocities[index] = match joint.joint_type() {
                JointType::Revolute => rotated_omega + velocity * joint.axis().as_ref(),
                JointType::Prismatic | JointType::Fixed => rotated_omega,
            };
        }
        for column in 0..joint_count {
            if self.joints[column].joint_type() == JointType::Fixed {
                continue;
            }
            for (index, &velocity) in qd.iter().enumerate() {
                let joint = &self.joints[index];
                let transform = &workspace.frames[index];
                let rotation_inverse = transform.rotation.inverse();
                let translation = transform.translation.vector;
                let axis: Vector3<f64> = *joint.axis().as_ref();
                let parent = self.joint_parents[index];
                let (parent_omega, parent_domega, parent_dalpha, parent_dacceleration) =
                    if parent == 0 {
                        (
                            Vector3::zeros(),
                            Vector3::zeros(),
                            Vector3::zeros(),
                            Vector3::zeros(),
                        )
                    } else {
                        (
                            workspace.angular_velocities[parent - 1],
                            workspace.derivative_omegas[parent - 1],
                            workspace.derivative_alphas[parent - 1],
                            workspace.derivative_accelerations[parent - 1],
                        )
                    };
                let rotated_omega = rotation_inverse * parent_omega;
                let rotated_domega = rotation_inverse * parent_domega;
                let source = f64::from(index == column);
                let mut dalpha = rotation_inverse * parent_dalpha;
                let mut dacceleration = rotation_inverse
                    * (parent_dacceleration
                        + parent_dalpha.cross(&translation)
                        + parent_domega.cross(&parent_omega.cross(&translation))
                        + parent_omega.cross(&parent_domega.cross(&translation)));
                let domega = match joint.joint_type() {
                    JointType::Revolute => {
                        dalpha += rotated_domega.cross(&(velocity * axis))
                            + source * rotated_omega.cross(&axis);
                        rotated_domega + source * axis
                    }
                    JointType::Prismatic => {
                        dacceleration += 2.0
                            * (source * rotated_omega.cross(&axis)
                                + velocity * rotated_domega.cross(&axis));
                        rotated_domega
                    }
                    JointType::Fixed => rotated_domega,
                };
                workspace.derivative_omegas[index] = domega;
                workspace.derivative_alphas[index] = dalpha;
                workspace.derivative_accelerations[index] = dacceleration;
                let omega = workspace.angular_velocities[index];
                let link = &self.links[index + 1];
                let center = *link.center_of_mass();
                let link_acceleration_derivative = dacceleration
                    + dalpha.cross(&center)
                    + domega.cross(&omega.cross(&center))
                    + omega.cross(&domega.cross(&center));
                let inertial_force = link.mass() * link_acceleration_derivative;
                workspace.link_loads[index] = Wrench::new(
                    center.cross(&inertial_force)
                        + link.inertia() * dalpha
                        + domega.cross(&(link.inertia() * omega))
                        + omega.cross(&(link.inertia() * domega)),
                    inertial_force,
                );
            }
            for index in (0..joint_count).rev() {
                let joint = &self.joints[index];
                let load = workspace.link_loads[index];
                output[column * joint_count + index] = 0.5 * joint.active_force(load);
                let parent = self.joint_parents[index];
                if parent != 0 {
                    let parent_load = wrench_to_parent(&workspace.frames[index], load);
                    workspace.link_loads[parent - 1] =
                        add_wrench(workspace.link_loads[parent - 1], parent_load);
                }
            }
        }
    }

    fn gravity_kernel(
        &self,
        q: &[f64],
        base_frame: &Frame,
        scratch: GravityScratch<'_>,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice_length(
            "transform workspace",
            scratch.transforms.len(),
            self.joint_count(),
        )?;
        self.validate_slice_length(
            "gravity workspace",
            scratch.gravity_at_link.len(),
            self.joint_count(),
        )?;
        self.validate_slice_length(
            "load workspace",
            scratch.link_loads.len(),
            self.joint_count(),
        )?;
        self.validate_output("gravity output", output)?;
        let base_gravity = base_frame.rotation.inverse() * Vector3::new(0.0, 0.0, GRAVITY);
        for (i, &position) in q.iter().enumerate() {
            scratch.transforms[i] = self.joints[i].frame(position);
            let parent = self.joint_parents[i];
            let parent_gravity = if parent == 0 {
                base_gravity
            } else {
                scratch.gravity_at_link[parent - 1]
            };
            scratch.gravity_at_link[i] = scratch.transforms[i].rotation.inverse() * parent_gravity;
        }
        for i in (0..self.joint_count()).rev() {
            let joint = &self.joints[i];
            let link = &self.links[i + 1];
            let force = link.mass() * scratch.gravity_at_link[i];
            let gravity_load = Wrench::new(link.center_of_mass().cross(&force), force);
            scratch.link_loads[i] = add_wrench(scratch.link_loads[i], gravity_load);
            output[i] = joint.active_force(scratch.link_loads[i]);
            let parent = self.joint_parents[i];
            if parent != 0 {
                let parent_load = wrench_to_parent(&scratch.transforms[i], scratch.link_loads[i]);
                scratch.link_loads[parent - 1] =
                    add_wrench(scratch.link_loads[parent - 1], parent_load);
            }
        }
        Ok(())
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
        self.validate_output("IK joint workspace", q_work)?;
        self.validate_output("IK step workspace", step)?;
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
                if self.joints[joint_index].joint_type() == JointType::Fixed {
                    continue;
                }
                let column = &jacobian[6 * joint_index..6 * joint_index + 6];
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
                if self.joints[joint_index].joint_type() == JointType::Fixed {
                    continue;
                }
                let column = &jacobian[6 * joint_index..6 * joint_index + 6];
                step[joint_index] = column
                    .iter()
                    .zip(weighted_error.iter())
                    .map(|(lhs, rhs)| lhs * rhs)
                    .sum();
                step_norm_squared += step[joint_index] * step[joint_index];
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
                if self.joints[joint_index].joint_type() != JointType::Fixed {
                    q_work[joint_index] += scale * step[joint_index];
                }
            }
        }
        unreachable!("inverse-kinematics loop always returns")
    }

    fn validate_inverse_kinematics_solution(&self, q: &[f64]) -> Result<()> {
        for (joint_index, (joint, &position)) in self.joints.iter().zip(q).enumerate() {
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

    fn prepare_indexed_loads(&self, loads: &[IndexedLoad], output: &mut [Wrench]) -> Result<()> {
        output.fill(Wrench::zeros());
        for load in loads {
            let link_index = self.validate_link_id(load.link)?;
            if link_index != 0 {
                output[link_index - 1] = add_wrench(output[link_index - 1], load.wrench);
            }
        }
        Ok(())
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
            self.validate_slice_length(name, actual, self.joint_count())?;
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

/// Returns the square `[v]x[v]x = v v^T - |v|^2 I` of the cross-product matrix.
fn skew_square(vector: Vector3<f64>) -> Matrix3<f64> {
    vector * vector.transpose() - vector.norm_squared() * Matrix3::identity()
}
