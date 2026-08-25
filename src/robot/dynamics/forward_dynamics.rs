use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::{BaseMode, BaseState, Error, Frame, JointType, Result, Twist, Wrench};

use super::super::{FLOATING_BASE_DOF, IndexedLoad, Model, Robot, Workspace};

const GRAVITY: f64 = 9.80665;
type Matrix6 = SMatrix<f64, 6, 6>;
type Vector6 = SVector<f64, 6>;

impl Robot {
    /// Writes generalized accelerations computed with the articulated-body algorithm.
    ///
    /// For a floating base, `generalized_forces` and `output` are ordered as
    /// world-frame angular base components, world-frame linear base components,
    /// then non-fixed URDF joints. A floating base's stored acceleration is
    /// ignored because forward dynamics solves for it; a fixed base still
    /// requires zero stored motion.
    ///
    /// External loads use the same resisting-wrench convention as
    /// [`Robot::inverse_dynamics`].
    ///
    /// # Errors
    ///
    /// Returns an error for invalid lengths or load link IDs, or when an
    /// articulated inertia is singular.
    #[allow(clippy::too_many_arguments)]
    pub fn forward_dynamics(
        &mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        generalized_forces: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> Result<()> {
        self.model.forward_dynamics(
            base,
            q,
            qd,
            generalized_forces,
            loads,
            &mut self.workspace,
            output,
        )
    }
}

impl Model {
    #[allow(clippy::too_many_arguments)]
    fn forward_dynamics(
        &self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        generalized_forces: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_base_state(base)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_output("forward dynamics generalized forces", generalized_forces)?;
        self.validate_output("forward dynamics output", output)?;

        let root_load = self.prepare_indexed_loads(loads, &mut workspace.link_loads)?;
        let root_rotation_inverse = base.frame().rotation.inverse();
        let root_velocity = Twist::new(
            root_rotation_inverse * base.velocity().angular,
            root_rotation_inverse * base.velocity().linear,
        );

        let root_link = self.link_dynamics[0];
        let mut root_inertia = rigid_body_inertia(
            root_link.mass,
            root_link.first_moment,
            root_link.origin_inertia,
        );
        let root_momentum = inertia_apply(&root_inertia, root_velocity);
        let mut root_bias_force = add_wrench(force_cross(root_velocity, root_momentum), root_load);

        // First pass: transforms, velocities, velocity bias, body inertia, and bias force.
        for index in 0..self.model_joint_count() {
            let joint = self.joint_kinematics[index];
            let transform = joint.frame(self.joint_value(q, index));
            let parent = self.parent_link_indices[index];
            let parent_velocity = if parent == 0 {
                root_velocity
            } else {
                workspace.spatial_velocities[parent - 1]
            };
            let motion_subspace = joint_motion_subspace(joint.joint_type, *joint.axis.as_ref());
            let joint_velocity = scale_twist(motion_subspace, self.joint_value(qd, index));
            let velocity = add_twist(motion_to_child(&transform, parent_velocity), joint_velocity);
            let link = self.link_dynamics[index + 1];
            let inertia = rigid_body_inertia(link.mass, link.first_moment, link.origin_inertia);
            let momentum = inertia_apply(&inertia, velocity);

            workspace.frames[index] = transform;
            workspace.spatial_velocities[index] = velocity;
            workspace.bias_accelerations[index] = motion_cross(velocity, joint_velocity);
            workspace.articulated_inertias[index] = inertia;
            workspace.articulated_bias_forces[index] =
                add_wrench(force_cross(velocity, momentum), workspace.link_loads[index]);
        }

        let joint_offset = self.base_dof_count();

        // Second pass: eliminate active joint accelerations and propagate each
        // articulated subtree into its parent.
        for index in (0..self.model_joint_count()).rev() {
            let joint = self.joint_kinematics[index];
            let inertia = workspace.articulated_inertias[index];
            let bias_force = workspace.articulated_bias_forces[index];
            let bias_acceleration = workspace.bias_accelerations[index];
            let (reduced_inertia, reduced_bias_force) = if let Some(dof_index) =
                self.joint_dof_indices[index]
            {
                let motion_subspace = joint_motion_subspace(joint.joint_type, *joint.axis.as_ref());
                let articulated_u = inertia_apply(&inertia, motion_subspace);
                let articulated_d = motion_force_dot(motion_subspace, articulated_u);
                if !articulated_d.is_finite() || articulated_d <= 0.0 {
                    return Err(Error::ForwardDynamicsSingularJointInertia {
                        joint_index: dof_index,
                    });
                }
                let joint_bias = generalized_forces[joint_offset + dof_index]
                    - motion_force_dot(motion_subspace, bias_force);
                workspace.articulated_u[index] = articulated_u;
                workspace.articulated_d[index] = articulated_d;
                workspace.articulated_joint_bias[index] = joint_bias;

                let u = wrench_vector(articulated_u);
                let reduced_inertia = inertia - (u * u.transpose()) / articulated_d;
                let reduced_bias_force = add_wrench(
                    add_wrench(
                        bias_force,
                        inertia_apply(&reduced_inertia, bias_acceleration),
                    ),
                    scale_wrench(articulated_u, joint_bias / articulated_d),
                );
                (reduced_inertia, reduced_bias_force)
            } else {
                (
                    inertia,
                    add_wrench(bias_force, inertia_apply(&inertia, bias_acceleration)),
                )
            };

            let parent = self.parent_link_indices[index];
            let parent_inertia =
                transform_inertia_to_parent(&workspace.frames[index], &reduced_inertia);
            let parent_bias_force =
                super::wrench_to_parent(&workspace.frames[index], reduced_bias_force);
            if parent == 0 {
                root_inertia += parent_inertia;
                root_bias_force = add_wrench(root_bias_force, parent_bias_force);
            } else {
                workspace.articulated_inertias[parent - 1] += parent_inertia;
                workspace.articulated_bias_forces[parent - 1] = add_wrench(
                    workspace.articulated_bias_forces[parent - 1],
                    parent_bias_force,
                );
            }
        }

        output.fill(0.0);
        let gravity_local = root_rotation_inverse * Vector3::new(0.0, 0.0, GRAVITY);
        let root_acceleration = match self.base_mode() {
            BaseMode::Fixed => Twist::new(Vector3::zeros(), gravity_local),
            BaseMode::Floating => {
                let world_base_force = Wrench::new(
                    Vector3::from_column_slice(&generalized_forces[..3]),
                    Vector3::from_column_slice(&generalized_forces[3..FLOATING_BASE_DOF]),
                );
                let local_base_force = Wrench::new(
                    root_rotation_inverse * world_base_force.torque,
                    root_rotation_inverse * world_base_force.force,
                );
                let right_hand_side = wrench_vector(sub_wrench(local_base_force, root_bias_force));
                let symmetric_inertia = (root_inertia + root_inertia.transpose()) * 0.5;
                let eigenvalues = symmetric_inertia.symmetric_eigen().eigenvalues;
                let minimum_eigenvalue = eigenvalues.min();
                let maximum_eigenvalue = eigenvalues.max();
                if !minimum_eigenvalue.is_finite()
                    || !maximum_eigenvalue.is_finite()
                    || maximum_eigenvalue <= 0.0
                    || minimum_eigenvalue <= maximum_eigenvalue * f64::EPSILON.sqrt()
                {
                    return Err(Error::ForwardDynamicsSingularBaseInertia);
                }
                let Some(factorization) = symmetric_inertia.cholesky() else {
                    return Err(Error::ForwardDynamicsSingularBaseInertia);
                };
                let acceleration = twist_from_vector(factorization.solve(&right_hand_side));
                if !twist_is_finite(acceleration) {
                    return Err(Error::ForwardDynamicsSingularBaseInertia);
                }

                let physical_linear_local = acceleration.linear - gravity_local
                    + root_velocity.angular.cross(&root_velocity.linear);
                let world_angular = base.frame().rotation * acceleration.angular;
                let world_linear = base.frame().rotation * physical_linear_local;
                output[..3].copy_from_slice(world_angular.as_slice());
                output[3..FLOATING_BASE_DOF].copy_from_slice(world_linear.as_slice());
                acceleration
            }
        };

        // Third pass: recover joint accelerations and complete link accelerations.
        for index in 0..self.model_joint_count() {
            let parent = self.parent_link_indices[index];
            let parent_acceleration = if parent == 0 {
                root_acceleration
            } else {
                workspace.spatial_accelerations[parent - 1]
            };
            let mut acceleration = add_twist(
                motion_to_child(&workspace.frames[index], parent_acceleration),
                workspace.bias_accelerations[index],
            );
            if let Some(dof_index) = self.joint_dof_indices[index] {
                let joint_acceleration = (workspace.articulated_joint_bias[index]
                    - motion_force_dot(acceleration, workspace.articulated_u[index]))
                    / workspace.articulated_d[index];
                if !joint_acceleration.is_finite() {
                    return Err(Error::ForwardDynamicsSingularJointInertia {
                        joint_index: dof_index,
                    });
                }
                let motion_subspace = joint_motion_subspace(
                    self.joint_kinematics[index].joint_type,
                    *self.joint_kinematics[index].axis.as_ref(),
                );
                acceleration = add_twist(
                    acceleration,
                    scale_twist(motion_subspace, joint_acceleration),
                );
                output[joint_offset + dof_index] = joint_acceleration;
            }
            workspace.spatial_accelerations[index] = acceleration;
        }
        Ok(())
    }
}

fn rigid_body_inertia(mass: f64, moment: Vector3<f64>, inertia: Matrix3<f64>) -> Matrix6 {
    let moment_cross = cross_matrix(moment);
    let mut output = Matrix6::zeros();
    output.fixed_view_mut::<3, 3>(0, 0).copy_from(&inertia);
    output.fixed_view_mut::<3, 3>(0, 3).copy_from(&moment_cross);
    output
        .fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&(-moment_cross));
    output
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&(mass * Matrix3::identity()));
    output
}

fn cross_matrix(value: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -value.z, value.y, value.z, 0.0, -value.x, -value.y, value.x, 0.0,
    )
}

fn motion_transform(transform: &Frame) -> Matrix6 {
    let rotation_inverse = transform.rotation.inverse().to_rotation_matrix();
    let rotation = rotation_inverse.matrix();
    let mut output = Matrix6::zeros();
    output.fixed_view_mut::<3, 3>(0, 0).copy_from(rotation);
    output.fixed_view_mut::<3, 3>(3, 3).copy_from(rotation);
    output
        .fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&(-rotation * cross_matrix(transform.translation.vector)));
    output
}

fn transform_inertia_to_parent(transform: &Frame, inertia: &Matrix6) -> Matrix6 {
    let motion_transform = motion_transform(transform);
    motion_transform.transpose() * inertia * motion_transform
}

fn motion_to_child(transform: &Frame, value: Twist) -> Twist {
    let rotation_inverse = transform.rotation.inverse();
    Twist::new(
        rotation_inverse * value.angular,
        rotation_inverse * (value.linear + value.angular.cross(&transform.translation.vector)),
    )
}

fn motion_cross(lhs: Twist, rhs: Twist) -> Twist {
    Twist::new(
        lhs.angular.cross(&rhs.angular),
        lhs.linear.cross(&rhs.angular) + lhs.angular.cross(&rhs.linear),
    )
}

fn force_cross(motion: Twist, force: Wrench) -> Wrench {
    Wrench::new(
        motion.angular.cross(&force.torque) + motion.linear.cross(&force.force),
        motion.angular.cross(&force.force),
    )
}

fn joint_motion_subspace(joint_type: JointType, axis: Vector3<f64>) -> Twist {
    match joint_type {
        JointType::Revolute => Twist::new(axis, Vector3::zeros()),
        JointType::Prismatic => Twist::new(Vector3::zeros(), axis),
        JointType::Fixed => Twist::zeros(),
    }
}

fn inertia_apply(inertia: &Matrix6, motion: Twist) -> Wrench {
    wrench_from_vector(inertia * motion.to_vector())
}

fn motion_force_dot(motion: Twist, force: Wrench) -> f64 {
    motion.angular.dot(&force.torque) + motion.linear.dot(&force.force)
}

fn add_twist(lhs: Twist, rhs: Twist) -> Twist {
    Twist::new(lhs.angular + rhs.angular, lhs.linear + rhs.linear)
}

fn scale_twist(value: Twist, scale: f64) -> Twist {
    Twist::new(scale * value.angular, scale * value.linear)
}

fn add_wrench(lhs: Wrench, rhs: Wrench) -> Wrench {
    Wrench::new(lhs.torque + rhs.torque, lhs.force + rhs.force)
}

fn sub_wrench(lhs: Wrench, rhs: Wrench) -> Wrench {
    Wrench::new(lhs.torque - rhs.torque, lhs.force - rhs.force)
}

fn scale_wrench(value: Wrench, scale: f64) -> Wrench {
    Wrench::new(scale * value.torque, scale * value.force)
}

fn wrench_vector(value: Wrench) -> Vector6 {
    Vector6::from_iterator(value.torque.iter().chain(value.force.iter()).copied())
}

fn wrench_from_vector(value: Vector6) -> Wrench {
    Wrench::new(
        Vector3::new(value[0], value[1], value[2]),
        Vector3::new(value[3], value[4], value[5]),
    )
}

fn twist_from_vector(value: Vector6) -> Twist {
    Twist::new(
        Vector3::new(value[0], value[1], value[2]),
        Vector3::new(value[3], value[4], value[5]),
    )
}

fn twist_is_finite(value: Twist) -> bool {
    value
        .angular
        .iter()
        .chain(value.linear.iter())
        .all(|component| component.is_finite())
}
