use std::{
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
};

use nalgebra::{SMatrix, SVector, Vector3};

use crate::{
    Error, Frame, Jacobian, Joint, JointType, JointVector, Link, Result, Twist, Wrench,
    urdf::tree_model,
};

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
    /// Returns conservative tolerances and damping suitable for general use.
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

/// Wrench applied at a link origin and expressed in that link's frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Load<'a> {
    /// Link at whose origin the wrench is applied.
    pub link: &'a Link,
    /// Wrench expressed in the selected link's coordinate frame.
    pub wrench: Wrench,
}

/// Runtime-topology tree robot with fixed-size joint-space calculation inputs and outputs.
#[derive(Clone, Debug)]
pub struct Robot {
    model_id: u64,
    name: String,
    joints: Box<[Joint]>,
    links: Box<[Link]>,
    joint_parents: Box<[usize]>,
    leaf_links: Box<[usize]>,
}

impl Robot {
    /// Loads and validates a tree robot model from a URDF file.
    ///
    /// Links and joints are stored in topological order, with the root link at
    /// index zero.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be parsed or its kinematic graph is
    /// invalid or contains an unsupported joint type.
    pub fn from_urdf(path: impl AsRef<Path>) -> Result<Self> {
        let robot = urdf_rs::read_file(path)?;
        let mut model = tree_model(&robot)?;
        let model_id = NEXT_MODEL_ID.fetch_add(1, Ordering::Relaxed);
        assert_ne!(
            model_id, UNOWNED_MODEL_ID,
            "robot model identifier overflow"
        );
        for link in &mut model.links {
            link.set_model_id(model_id);
        }
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
    /// Returns [`Error::UnknownLink`] if the model has no link named `name`.
    pub fn link(&self, name: &str) -> Result<&Link> {
        self.links
            .iter()
            .find(|link| link.name() == name)
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

    /// Checks that a fixed-size joint vector matches the loaded model.
    fn validate_joint_count<const N: usize>(&self) -> Result<()> {
        if self.joints.len() == N {
            Ok(())
        } else {
            Err(Error::WrongJointCount {
                expected: self.joints.len(),
                actual: N,
            })
        }
    }

    /// Returns a link's index after checking that it belongs to this model.
    fn validate_link(&self, link: &Link) -> Result<usize> {
        let index = link.index();
        if link.model_id() == self.model_id && index < self.links.len() {
            Ok(index)
        } else {
            Err(Error::InvalidLink {
                name: link.name().to_owned(),
            })
        }
    }

    /// Computes the target link frame relative to the root frame.
    ///
    /// # Errors
    ///
    /// Returns an error if `N` differs from [`Self::joint_count`] or `target`
    /// is not a link in this model.
    pub fn forward_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: &Link,
    ) -> Result<Frame> {
        self.validate_joint_count::<N>()?;
        let target_index = self.validate_link(target)?;
        if target_index == 0 {
            return Ok(Frame::identity());
        }
        Ok(self.link_frames(q)[target_index - 1])
    }

    /// Computes each non-root link frame in topological order.
    fn link_frames<const N: usize>(&self, q: &JointVector<N>) -> [Frame; N] {
        let mut frames: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        for i in 0..N {
            let parent = self.joint_parents[i];
            let parent_frame = if parent == 0 {
                Frame::identity()
            } else {
                frames[parent - 1]
            };
            frames[i] = parent_frame * self.joints[i].frame(q[i]);
        }
        frames
    }

    /// Computes one link frame and its root-frame geometric Jacobian.
    fn forward_kinematics_and_jacobian<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: &Link,
    ) -> Result<(Frame, Jacobian<N>)> {
        self.validate_joint_count::<N>()?;
        let target_index = self.validate_link(target)?;
        let frames = self.link_frames(q);
        let target_frame = if target_index == 0 {
            Frame::identity()
        } else {
            frames[target_index - 1]
        };
        let mut jacobian: Jacobian<N> = Jacobian::zeros();

        let mut current = target_index;
        while current != 0 {
            let joint_index = current - 1;
            let joint_frame = frames[joint_index];
            let axis = joint_frame.rotation * self.joints[joint_index].axis().as_ref();
            match self.joints[joint_index].joint_type() {
                JointType::Revolute => {
                    let linear = axis
                        .cross(&(target_frame.translation.vector - joint_frame.translation.vector));
                    jacobian
                        .fixed_view_mut::<3, 1>(0, joint_index)
                        .copy_from(&axis);
                    jacobian
                        .fixed_view_mut::<3, 1>(3, joint_index)
                        .copy_from(&linear);
                }
                JointType::Prismatic => {
                    jacobian
                        .fixed_view_mut::<3, 1>(3, joint_index)
                        .copy_from(&axis);
                }
                JointType::Fixed => {}
            }
            current = self.joint_parents[joint_index];
        }

        Ok((target_frame, jacobian))
    }

    /// Computes the target link's geometric Jacobian in the root frame.
    ///
    /// Rows 0 through 2 are angular velocity and rows 3 through 5 are linear
    /// velocity at the target link origin.
    ///
    /// # Errors
    ///
    /// Returns an error if `N` differs from [`Self::joint_count`] or `target`
    /// is invalid.
    pub fn jacobian<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: &Link,
    ) -> Result<Jacobian<N>> {
        Ok(self.forward_kinematics_and_jacobian(q, target)?.1)
    }

    /// Solves for a joint vector that reaches `desired` at `target`.
    ///
    /// This uses damped least squares with [`InverseKinematicsOptions::default`].
    /// The iteration is unconstrained, but a converged result outside a URDF
    /// joint limit is returned as an error.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid dimension or link, non-finite input,
    /// numerical failure, limit violation, or failure to converge.
    pub fn inverse_kinematics<const N: usize>(
        &self,
        initial_q: &JointVector<N>,
        target: &Link,
        desired: &Frame,
    ) -> Result<JointVector<N>> {
        self.inverse_kinematics_with_options(
            initial_q,
            target,
            desired,
            InverseKinematicsOptions::default(),
        )
    }

    /// Solves inverse kinematics using configurable damped least squares.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid solver options, an invalid dimension or
    /// link, non-finite input, numerical failure, limit violation, or failure
    /// to converge.
    pub fn inverse_kinematics_with_options<const N: usize>(
        &self,
        initial_q: &JointVector<N>,
        target: &Link,
        desired: &Frame,
        options: InverseKinematicsOptions,
    ) -> Result<JointVector<N>> {
        self.validate_joint_count::<N>()?;
        self.validate_link(target)?;
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

        let mut q = *initial_q;
        let damping_squared = options.damping * options.damping;
        for iteration in 0..=options.max_iterations {
            let (current, jacobian) = self.forward_kinematics_and_jacobian(&q, target)?;
            let translation_error = desired.translation.vector - current.translation.vector;
            let rotation_error = (desired.rotation * current.rotation.inverse()).scaled_axis();
            let translation_error_norm = translation_error.norm();
            let rotation_error_norm = rotation_error.norm();
            if translation_error_norm <= options.translation_tolerance
                && rotation_error_norm <= options.rotation_tolerance
            {
                self.validate_inverse_kinematics_solution(&q)?;
                return Ok(q);
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
            let regularized = jacobian * jacobian.transpose()
                + SMatrix::<f64, 6, 6>::identity() * damping_squared;
            let Some(weighted_error) = regularized.cholesky().map(|factor| factor.solve(&error))
            else {
                return Err(Error::NumericalFailure {
                    iteration: iteration + 1,
                });
            };
            let mut step = jacobian.transpose() * weighted_error;
            let step_norm = step.norm();
            if !step_norm.is_finite() {
                return Err(Error::NumericalFailure {
                    iteration: iteration + 1,
                });
            }
            if step_norm > options.max_step_norm {
                step *= options.max_step_norm / step_norm;
            }
            q += step;
        }

        unreachable!("inverse-kinematics loop always returns")
    }

    /// Checks a converged inverse-kinematics result against all joint limits.
    fn validate_inverse_kinematics_solution<const N: usize>(
        &self,
        q: &JointVector<N>,
    ) -> Result<()> {
        for (joint_index, (joint, &position)) in self.joints.iter().zip(q.iter()).enumerate() {
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

    /// Computes spatial velocity at a point fixed to a target link.
    ///
    /// `tool` defines the point relative to the target link. The resulting
    /// angular and linear velocity are rotated by `base` and expressed in that
    /// frame's orientation.
    ///
    /// # Errors
    ///
    /// Returns an error if `N` differs from [`Self::joint_count`] or `target`
    /// is invalid.
    pub fn forward_velocity_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        target: &Link,
        base: &Frame,
        tool: &Frame,
    ) -> Result<Twist> {
        let (end, mut jacobian) = self.forward_kinematics_and_jacobian(q, target)?;
        let offset_world = end.rotation * tool.translation.vector;
        for i in 0..N {
            let angular = jacobian.fixed_view::<3, 1>(0, i).into_owned();
            let shifted =
                jacobian.fixed_view::<3, 1>(3, i).into_owned() + angular.cross(&offset_world);
            jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&shifted);
        }
        let vector = jacobian * qd;
        Ok(Twist::new(
            base.rotation * vector.fixed_rows::<3>(0).into_owned(),
            base.rotation * vector.fixed_rows::<3>(3).into_owned(),
        ))
    }

    /// Computes spatial acceleration of a target link origin in the root frame.
    ///
    /// # Errors
    ///
    /// Returns an error if `N` differs from [`Self::joint_count`] or `target`
    /// is invalid.
    pub fn forward_acceleration_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        target: &Link,
    ) -> Result<Twist> {
        self.validate_joint_count::<N>()?;
        let target_index = self.validate_link(target)?;
        Ok(self.link_acceleration(q, qd, qdd, target_index))
    }

    /// Computes joint forces and the root reaction wrench using Newton-Euler dynamics.
    ///
    /// The returned tuple is `(joint_force, wrench_at_base)`. Wrenches in
    /// `loads` must be expressed in their selected link frames.
    /// Gravity is included along the negative world Z direction through the
    /// equivalent upward inertial acceleration.
    ///
    /// # Errors
    ///
    /// Returns an error if `N` differs from [`Self::joint_count`] or an external
    /// load references a link from another model.
    #[allow(clippy::too_many_arguments)]
    pub fn inverse_dynamics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        base_frame: &Frame,
        base_velocity: Twist,
        base_acceleration: Twist,
        loads: &[Load<'_>],
    ) -> Result<(JointVector<N>, Wrench)> {
        self.validate_joint_count::<N>()?;
        let mut transforms: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        let mut angular_velocities: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut angular_accelerations: [Vector3<f64>; N] =
            std::array::from_fn(|_| Vector3::zeros());
        let mut origin_accelerations: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut link_accelerations: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());

        let base_rotation_inverse = base_frame.rotation.inverse();
        let base_omega = base_rotation_inverse * base_velocity.angular;
        let base_angular_acceleration = base_rotation_inverse * base_acceleration.angular;
        let base_acceleration =
            base_rotation_inverse * (Vector3::new(0.0, 0.0, GRAVITY) + base_acceleration.linear);

        for i in 0..N {
            let joint = &self.joints[i];
            let link = &self.links[i + 1];
            let parent = self.joint_parents[i];
            let (parent_omega, parent_angular_acceleration, parent_acceleration) = if parent == 0 {
                (base_omega, base_angular_acceleration, base_acceleration)
            } else {
                (
                    angular_velocities[parent - 1],
                    angular_accelerations[parent - 1],
                    origin_accelerations[parent - 1],
                )
            };
            let transform = joint.frame(q[i]);
            let rotation_inverse = transform.rotation.inverse();
            let translation = transform.translation.vector;
            let axis = joint.axis().as_ref();
            let rotated_omega = rotation_inverse * parent_omega;
            let rotated_angular_acceleration = rotation_inverse * parent_angular_acceleration;
            let translated_acceleration = rotation_inverse
                * (parent_acceleration
                    + parent_angular_acceleration.cross(&translation)
                    + parent_omega.cross(&parent_omega.cross(&translation)));
            let (omega, angular_acceleration, acceleration) = match joint.joint_type() {
                JointType::Revolute => {
                    let angular_acceleration = rotated_angular_acceleration
                        + qdd[i] * axis
                        + rotated_omega.cross(&(qd[i] * axis));
                    (
                        rotated_omega + qd[i] * axis,
                        angular_acceleration,
                        translated_acceleration,
                    )
                }
                JointType::Prismatic => (
                    rotated_omega,
                    rotated_angular_acceleration,
                    translated_acceleration
                        + qdd[i] * axis
                        + 2.0 * qd[i] * parent_omega.cross(&(transform.rotation * axis)),
                ),
                JointType::Fixed => (
                    rotated_omega,
                    rotated_angular_acceleration,
                    translated_acceleration,
                ),
            };
            angular_velocities[i] = omega;
            angular_accelerations[i] = angular_acceleration;
            origin_accelerations[i] = acceleration;
            let center = link.center_of_mass();
            link_accelerations[i] = acceleration
                + angular_acceleration.cross(center)
                + omega.cross(&omega.cross(center));
            transforms[i] = transform;
        }

        let mut joint_force = JointVector::zeros();
        let mut link_loads: [Wrench; N] = std::array::from_fn(|_| Wrench::zeros());
        let mut base_load = Wrench::zeros();
        for load in loads {
            let link_index = self.validate_link(load.link)?;
            if link_index == 0 {
                base_load = add_wrench(base_load, load.wrench);
            } else {
                let index = link_index - 1;
                link_loads[index] = add_wrench(link_loads[index], load.wrench);
            }
        }

        for i in (0..N).rev() {
            let joint = &self.joints[i];
            let link = &self.links[i + 1];
            let inertial_force = link.mass() * link_accelerations[i];
            let angular_momentum = link.inertia() * angular_velocities[i];
            let inertial_load = Wrench::new(
                link.center_of_mass().cross(&inertial_force)
                    + link.inertia() * angular_accelerations[i]
                    + angular_velocities[i].cross(&angular_momentum),
                inertial_force,
            );
            link_loads[i] = add_wrench(link_loads[i], inertial_load);
            joint_force[i] = joint.active_force(link_loads[i]);

            let parent_load = wrench_to_parent(&transforms[i], link_loads[i]);
            let parent = self.joint_parents[i];
            if parent == 0 {
                base_load = add_wrench(base_load, parent_load);
            } else {
                link_loads[parent - 1] = add_wrench(link_loads[parent - 1], parent_load);
            }
        }
        Ok((joint_force, base_load))
    }

    /// Computes gravity joint forces and the resulting root reaction wrench.
    ///
    /// The returned tuple is `(joint_force, wrench_at_base)`. Wrenches in
    /// `loads` must be expressed in their selected link frames.
    ///
    /// # Errors
    ///
    /// Returns an error if `N` differs from [`Self::joint_count`] or an external
    /// load references a link from another model.
    pub fn gravity<const N: usize>(
        &self,
        q: &JointVector<N>,
        base_frame: &Frame,
        loads: &[Load<'_>],
    ) -> Result<(JointVector<N>, Wrench)> {
        self.validate_joint_count::<N>()?;
        let base_gravity = base_frame.rotation.inverse() * Vector3::new(0.0, 0.0, GRAVITY);
        let mut transforms: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        let mut gravity_at_link: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        for i in 0..N {
            transforms[i] = self.joints[i].frame(q[i]);
            let parent = self.joint_parents[i];
            let parent_gravity = if parent == 0 {
                base_gravity
            } else {
                gravity_at_link[parent - 1]
            };
            gravity_at_link[i] = transforms[i].rotation.inverse() * parent_gravity;
        }

        let mut torque = JointVector::zeros();
        let mut link_loads: [Wrench; N] = std::array::from_fn(|_| Wrench::zeros());
        let mut base_load = Wrench::zeros();
        for load in loads {
            let link_index = self.validate_link(load.link)?;
            if link_index == 0 {
                base_load = add_wrench(base_load, load.wrench);
            } else {
                let index = link_index - 1;
                link_loads[index] = add_wrench(link_loads[index], load.wrench);
            }
        }

        for i in (0..N).rev() {
            let joint = &self.joints[i];
            let link = &self.links[i + 1];
            let force = link.mass() * gravity_at_link[i];
            let gravity_load = Wrench::new(link.center_of_mass().cross(&force), force);
            link_loads[i] = add_wrench(link_loads[i], gravity_load);
            torque[i] = joint.active_force(link_loads[i]);

            let parent_load = wrench_to_parent(&transforms[i], link_loads[i]);
            let parent = self.joint_parents[i];
            if parent == 0 {
                base_load = add_wrench(base_load, parent_load);
            } else {
                link_loads[parent - 1] = add_wrench(link_loads[parent - 1], parent_load);
            }
        }
        Ok((torque, base_load))
    }

    /// Propagates joint motion to the spatial acceleration of one link origin.
    fn link_acceleration<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        target_index: usize,
    ) -> Twist {
        if target_index == 0 {
            return Twist::zeros();
        }
        let mut frames: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        let mut angular_velocities: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut angular_accelerations: [Vector3<f64>; N] =
            std::array::from_fn(|_| Vector3::zeros());
        let mut linear_accelerations: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());

        for i in 0..N {
            let parent = self.joint_parents[i];
            let (parent_frame, angular_velocity, angular_acceleration, linear_acceleration) =
                if parent == 0 {
                    (
                        Frame::identity(),
                        Vector3::zeros(),
                        Vector3::zeros(),
                        Vector3::zeros(),
                    )
                } else {
                    (
                        frames[parent - 1],
                        angular_velocities[parent - 1],
                        angular_accelerations[parent - 1],
                        linear_accelerations[parent - 1],
                    )
                };
            let frame = parent_frame * self.joints[i].frame(q[i]);
            let offset = frame.translation.vector - parent_frame.translation.vector;
            let axis = frame.rotation * self.joints[i].axis().as_ref();
            let mut child_angular_velocity = angular_velocity;
            let mut child_angular_acceleration = angular_acceleration;
            let mut child_linear_acceleration = linear_acceleration
                + angular_acceleration.cross(&offset)
                + angular_velocity.cross(&angular_velocity.cross(&offset));

            match self.joints[i].joint_type() {
                JointType::Revolute => {
                    child_angular_acceleration +=
                        axis * qdd[i] + angular_velocity.cross(&axis) * qd[i];
                    child_angular_velocity += axis * qd[i];
                }
                JointType::Prismatic => {
                    child_linear_acceleration +=
                        axis * qdd[i] + 2.0 * qd[i] * angular_velocity.cross(&axis);
                }
                JointType::Fixed => {}
            }

            frames[i] = frame;
            angular_velocities[i] = child_angular_velocity;
            angular_accelerations[i] = child_angular_acceleration;
            linear_accelerations[i] = child_linear_acceleration;
        }

        let index = target_index - 1;
        Twist::new(angular_accelerations[index], linear_accelerations[index])
    }
}

/// Checks that all inverse-kinematics options are finite and strictly positive.
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

/// Transforms a child-frame wrench to its parent frame.
fn wrench_to_parent(transform: &Frame, wrench: Wrench) -> Wrench {
    let force = transform.rotation * wrench.force;
    Wrench::new(
        transform.rotation * wrench.torque + transform.translation.vector.cross(&force),
        force,
    )
}

/// Adds two wrenches expressed at the same point and in the same frame.
fn add_wrench(lhs: Wrench, rhs: Wrench) -> Wrench {
    Wrench::new(lhs.torque + rhs.torque, lhs.force + rhs.force)
}
