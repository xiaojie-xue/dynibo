use std::{
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
};

use nalgebra::{SMatrix, SVector, Vector3};

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
    leaf_links: Box<[usize]>,
}

struct AccelerationScratch<'a> {
    frames: &'a mut [Frame],
    angular_velocities: &'a mut [Vector3<f64>],
    angular_accelerations: &'a mut [Vector3<f64>],
    linear_accelerations: &'a mut [Vector3<f64>],
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
        Ok(Self {
            model_id,
            name: robot.name,
            joints: model.joints.into_boxed_slice(),
            links: model.links.into_boxed_slice(),
            joint_parents: model.joint_parents.into_boxed_slice(),
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
        self.link_frames_kernel(q, &mut workspace.frames)?;
        Ok(frame_for_target(&workspace.frames, target_index))
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
        self.acceleration_kernel(
            q,
            qd,
            qdd,
            target_index,
            AccelerationScratch {
                frames: &mut workspace.frames,
                angular_velocities: &mut workspace.angular_velocities,
                angular_accelerations: &mut workspace.angular_accelerations,
                linear_accelerations: &mut workspace.origin_accelerations,
            },
        )
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

    fn link_frames_kernel(&self, q: &[f64], frames: &mut [Frame]) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice_length("frame workspace", frames.len(), self.joint_count())?;
        for i in 0..self.joint_count() {
            let parent = self.joint_parents[i];
            let parent_frame = if parent == 0 {
                Frame::identity()
            } else {
                frames[parent - 1]
            };
            frames[i] = parent_frame * self.joints[i].frame(q[i]);
        }
        Ok(())
    }

    fn forward_kinematics_and_jacobian_kernel(
        &self,
        q: &[f64],
        target_index: usize,
        frames: &mut [Frame],
        output: &mut [f64],
    ) -> Result<Frame> {
        self.link_frames_kernel(q, frames)?;
        self.jacobian_kernel(frames, target_index, output)
    }

    fn jacobian_kernel(
        &self,
        frames: &[Frame],
        target_index: usize,
        output: &mut [f64],
    ) -> Result<Frame> {
        self.validate_slice_length("frame workspace", frames.len(), self.joint_count())?;
        self.validate_slice_length("jacobian output", output.len(), 6 * self.joint_count())?;
        output.fill(0.0);
        let target_frame = frame_for_target(frames, target_index);
        let mut current = target_index;
        while current != 0 {
            let joint_index = current - 1;
            let joint_frame = frames[joint_index];
            let axis = joint_frame.rotation * self.joints[joint_index].axis().as_ref();
            let column = &mut output[6 * joint_index..6 * joint_index + 6];
            match self.joints[joint_index].joint_type() {
                JointType::Revolute => {
                    let linear = axis
                        .cross(&(target_frame.translation.vector - joint_frame.translation.vector));
                    column[..3].copy_from_slice(axis.as_slice());
                    column[3..].copy_from_slice(linear.as_slice());
                }
                JointType::Prismatic => column[3..].copy_from_slice(axis.as_slice()),
                JointType::Fixed => {}
            }
            current = self.joint_parents[joint_index];
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
    ) -> Result<Twist> {
        self.validate_slice("qd", qd)?;
        let end = self.forward_kinematics_and_jacobian_kernel(q, target_index, frames, jacobian)?;
        let offset_world = end.rotation * tool.translation.vector;
        let mut angular = Vector3::zeros();
        let mut linear = Vector3::zeros();
        for i in 0..self.joint_count() {
            let column = &jacobian[6 * i..6 * i + 6];
            let column_angular = Vector3::new(column[0], column[1], column[2]);
            let column_linear =
                Vector3::new(column[3], column[4], column[5]) + column_angular.cross(&offset_world);
            angular += column_angular * qd[i];
            linear += column_linear * qd[i];
        }
        Ok(Twist::new(base.rotation * angular, base.rotation * linear))
    }

    fn acceleration_kernel(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        target_index: usize,
        scratch: AccelerationScratch<'_>,
    ) -> Result<Twist> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        self.validate_slice_length("frame workspace", scratch.frames.len(), self.joint_count())?;
        self.validate_slice_length(
            "angular velocity workspace",
            scratch.angular_velocities.len(),
            self.joint_count(),
        )?;
        self.validate_slice_length(
            "angular acceleration workspace",
            scratch.angular_accelerations.len(),
            self.joint_count(),
        )?;
        self.validate_slice_length(
            "linear acceleration workspace",
            scratch.linear_accelerations.len(),
            self.joint_count(),
        )?;
        if target_index == 0 {
            return Ok(Twist::zeros());
        }
        for i in 0..self.joint_count() {
            let parent = self.joint_parents[i];
            let (parent_frame, omega, alpha, linear) = if parent == 0 {
                (
                    Frame::identity(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
            } else {
                (
                    scratch.frames[parent - 1],
                    scratch.angular_velocities[parent - 1],
                    scratch.angular_accelerations[parent - 1],
                    scratch.linear_accelerations[parent - 1],
                )
            };
            let frame = parent_frame * self.joints[i].frame(q[i]);
            let offset = frame.translation.vector - parent_frame.translation.vector;
            let axis = frame.rotation * self.joints[i].axis().as_ref();
            let mut child_omega = omega;
            let mut child_alpha = alpha;
            let mut child_linear =
                linear + alpha.cross(&offset) + omega.cross(&omega.cross(&offset));
            match self.joints[i].joint_type() {
                JointType::Revolute => {
                    child_alpha += axis * qdd[i] + omega.cross(&axis) * qd[i];
                    child_omega += axis * qd[i];
                }
                JointType::Prismatic => {
                    child_linear += axis * qdd[i] + 2.0 * qd[i] * omega.cross(&axis);
                }
                JointType::Fixed => {}
            }
            scratch.frames[i] = frame;
            scratch.angular_velocities[i] = child_omega;
            scratch.angular_accelerations[i] = child_alpha;
            scratch.linear_accelerations[i] = child_linear;
        }
        let index = target_index - 1;
        Ok(Twist::new(
            scratch.angular_accelerations[index],
            scratch.linear_accelerations[index],
        ))
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
                        + 2.0 * qd[i] * parent_omega.cross(&(transform.rotation * axis)),
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

        output.fill(0.0);
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
        output.fill(0.0);
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
    ) -> Result<()> {
        self.validate_slice("initial_q", initial_q)?;
        self.validate_output("IK joint workspace", q_work)?;
        self.validate_output("IK step workspace", step)?;
        validate_inverse_kinematics_options(options)?;
        if !initial_q.iter().all(|value| value.is_finite()) {
            return Err(Error::NonFiniteInput {
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
            return Err(Error::NonFiniteInput {
                input: "target frame",
            });
        }
        q_work.copy_from_slice(initial_q);
        let damping_squared = options.damping * options.damping;
        for iteration in 0..=options.max_iterations {
            let current = self.forward_kinematics_and_jacobian_kernel(
                q_work,
                target_index,
                frames,
                jacobian,
            )?;
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
                return Err(Error::NotConverged {
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
            for column in jacobian.chunks_exact(6) {
                for row in 0..6 {
                    for col in 0..6 {
                        regularized[(row, col)] += column[row] * column[col];
                    }
                }
            }
            let Some(weighted_error) = regularized.cholesky().map(|factor| factor.solve(&error))
            else {
                return Err(Error::NumericalFailure {
                    iteration: iteration + 1,
                });
            };
            let mut step_norm_squared = 0.0;
            for (joint, column) in jacobian.chunks_exact(6).enumerate() {
                step[joint] = column
                    .iter()
                    .zip(weighted_error.iter())
                    .map(|(lhs, rhs)| lhs * rhs)
                    .sum();
                step_norm_squared += step[joint] * step[joint];
            }
            let step_norm = step_norm_squared.sqrt();
            if !step_norm.is_finite() {
                return Err(Error::NumericalFailure {
                    iteration: iteration + 1,
                });
            }
            let scale = if step_norm > options.max_step_norm {
                options.max_step_norm / step_norm
            } else {
                1.0
            };
            for i in 0..self.joint_count() {
                q_work[i] += scale * step[i];
            }
        }
        unreachable!("inverse-kinematics loop always returns")
    }

    fn validate_inverse_kinematics_solution(&self, q: &[f64]) -> Result<()> {
        for (joint_index, (joint, &position)) in self.joints.iter().zip(q).enumerate() {
            if joint.is_over_limit(position) {
                return Err(Error::JointLimitViolation {
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
        return Err(Error::InvalidOptions(
            "max_iterations must be greater than zero",
        ));
    }
    for (name, value) in [
        ("translation_tolerance", options.translation_tolerance),
        ("rotation_tolerance", options.rotation_tolerance),
        ("damping", options.damping),
        ("max_step_norm", options.max_step_norm),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return Err(Error::InvalidOptions(match name {
                "translation_tolerance" => {
                    "translation_tolerance must be finite and greater than zero"
                }
                "rotation_tolerance" => "rotation_tolerance must be finite and greater than zero",
                "damping" => "damping must be finite and greater than zero",
                _ => "max_step_norm must be finite and greater than zero",
            }));
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
