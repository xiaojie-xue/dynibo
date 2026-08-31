use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::{BaseState, Error, Frame, JointType, Result, Twist, Wrench};

use super::super::{
    FLOATING_BASE_DOF, FloatingRobot, IndexedLoad, Model, Robot, RootMode, Workspace,
    base_dof_count,
};

const GRAVITY: f64 = 9.80665;
type Matrix6 = SMatrix<f64, 6, 6>;
type Vector6 = SVector<f64, 6>;

impl Robot {
    /// Writes generalized accelerations computed with the articulated-body algorithm.
    ///
    /// The fixed-base input and output contain only non-fixed URDF joints.
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
        q: &[f64],
        qd: &[f64],
        generalized_forces: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> Result<()> {
        self.model.forward_dynamics(
            RootMode::Fixed,
            &self.world_from_root,
            Twist::zeros(),
            q,
            qd,
            generalized_forces,
            loads,
            &mut self.workspace,
            output,
        )
    }
}

impl FloatingRobot {
    /// Writes generalized accelerations computed with the articulated-body algorithm.
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
            RootMode::Floating,
            base.frame(),
            base.velocity(),
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
        base_mode: RootMode,
        base_frame: &Frame,
        base_velocity: Twist,
        q: &[f64],
        qd: &[f64],
        generalized_forces: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_output(
            base_mode,
            "forward dynamics generalized forces",
            generalized_forces,
        )?;
        self.validate_output(base_mode, "forward dynamics output", output)?;

        let root_load = self.prepare_indexed_loads(loads, &mut workspace.link_loads)?;
        let root_rotation_inverse = base_frame.rotation.inverse();
        let root_velocity = Twist::new(
            root_rotation_inverse * base_velocity.angular,
            root_rotation_inverse * base_velocity.linear,
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

        let joint_offset = base_dof_count(base_mode);

        // Second pass: eliminate active joint accelerations and propagate each
        // articulated subtree into its parent.
        for index in (0..self.model_joint_count()).rev() {
            let joint = self.joint_kinematics[index];
            let parent = self.parent_link_indices[index];
            let skip_root_propagation = parent == 0 && matches!(base_mode, RootMode::Fixed);
            let inertia = workspace.articulated_inertias[index];
            let bias_force = workspace.articulated_bias_forces[index];
            let bias_acceleration = workspace.bias_accelerations[index];
            let (reduced_inertia, reduced_bias_force) = if let Some(dof_index) =
                self.joint_dof_indices[index]
            {
                let motion_subspace = joint_motion_subspace(joint.joint_type, *joint.axis.as_ref());
                let articulated_u =
                    inertia_apply_joint(&inertia, joint.joint_type, *joint.axis.as_ref());
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

                // A fixed root has prescribed acceleration and no inertia solve.
                // Keep the validated joint terms needed by the third pass, but
                // do not reduce/transform a subtree into that unused root state.
                if skip_root_propagation {
                    continue;
                }

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
                if skip_root_propagation {
                    continue;
                }
                (
                    inertia,
                    add_wrench(bias_force, inertia_apply(&inertia, bias_acceleration)),
                )
            };

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
        let root_acceleration = match base_mode {
            RootMode::Fixed => Twist::new(Vector3::zeros(), gravity_local),
            RootMode::Floating => {
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
                let world_angular = base_frame.rotation * acceleration.angular;
                let world_linear = base_frame.rotation * physical_linear_local;
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

fn transform_inertia_to_parent(transform: &Frame, inertia: &Matrix6) -> Matrix6 {
    // Angular-first symmetric inertia is [A B; B^T D]. Rotate the three
    // independent blocks, then translate them with P = [translation]x.
    // This is X^T I X without constructing a dense spatial transform X.
    let rotation = transform.rotation.to_rotation_matrix().into_inner();
    let angular = rotation * inertia.fixed_view::<3, 3>(0, 0) * rotation.transpose();
    let coupling = rotation * inertia.fixed_view::<3, 3>(0, 3) * rotation.transpose();
    let linear = rotation * inertia.fixed_view::<3, 3>(3, 3) * rotation.transpose();
    let position_cross = cross_matrix(transform.translation.vector);
    let translated_coupling = coupling + position_cross * linear;
    let translated_angular =
        angular + position_cross * coupling.transpose() - translated_coupling * position_cross;
    let mut output = Matrix6::zeros();
    output
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&translated_angular);
    output
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&translated_coupling);
    output
        .fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&translated_coupling.transpose());
    output.fixed_view_mut::<3, 3>(3, 3).copy_from(&linear);
    output
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

fn inertia_apply_joint(inertia: &Matrix6, joint_type: JointType, axis: Vector3<f64>) -> Wrench {
    let offset = match joint_type {
        JointType::Revolute => 0,
        JointType::Prismatic => 3,
        JointType::Fixed => return Wrench::zeros(),
    };
    // Match only exact cardinal axes, including their negative directions.
    // Nearly aligned and arbitrary axes must retain all their components.
    let aligned = if axis.x.abs() == 1.0 && axis.y == 0.0 && axis.z == 0.0 {
        Some((0, axis.x))
    } else if axis.y.abs() == 1.0 && axis.x == 0.0 && axis.z == 0.0 {
        Some((1, axis.y))
    } else if axis.z.abs() == 1.0 && axis.x == 0.0 && axis.y == 0.0 {
        Some((2, axis.z))
    } else {
        None
    };
    let product = if let Some((index, sign)) = aligned {
        inertia.column(offset + index) * sign
    } else {
        inertia.fixed_columns::<3>(offset) * axis
    };
    wrench_from_vector(product)
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

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{Translation3, UnitQuaternion};

    #[test]
    fn block_inertia_transform_matches_dense_spatial_congruence() {
        for sample in 0..16 {
            let t = (sample + 1) as f64;
            // General symmetric positive-definite articulated inertia, not
            // just the more restricted spatial inertia of a single rigid body.
            let factor = Matrix6::from_fn(|row, column| {
                (t * 0.13 + (row * 6 + column) as f64 * 0.27).sin()
                    + if row == column { 2.0 } else { 0.0 }
            });
            let inertia = factor * factor.transpose();
            let frame = Frame::from_parts(
                Translation3::new(0.13 * t, -0.07 * t, 0.03 * t),
                UnitQuaternion::from_euler_angles(0.11 * t, -0.17 * t, 0.23 * t),
            );
            // Assemble the reference transformation by acting on the six
            // basis twists, independently of the optimized block expression.
            let dense_transform = Matrix6::from_fn(|row, column| {
                let mut basis = Vector6::zeros();
                basis[column] = 1.0;
                motion_to_child(&frame, twist_from_vector(basis)).to_vector()[row]
            });
            let expected = dense_transform.transpose() * inertia * dense_transform;
            let actual = transform_inertia_to_parent(&frame, &inertia);
            assert_relative_eq!(actual, expected, epsilon = 1e-11, max_relative = 1e-12);
        }
    }

    #[test]
    fn specialized_joint_product_preserves_signed_and_non_cardinal_axes() {
        let factor = Matrix6::from_fn(|row, column| {
            ((row * 6 + column + 1) as f64 * 0.19).cos() + if row == column { 3.0 } else { 0.0 }
        });
        let inertia = factor * factor.transpose();
        let axes = [
            Vector3::x(),
            -Vector3::x(),
            Vector3::y(),
            -Vector3::y(),
            Vector3::z(),
            -Vector3::z(),
            Vector3::new(1.0, 1e-9, 0.0).normalize(),
            Vector3::new(0.0, -1.0, 1e-9).normalize(),
            Vector3::new(1e-9, 0.0, 1.0).normalize(),
            Vector3::new(0.3, -0.4, 0.5).normalize(),
        ];
        for joint_type in [JointType::Revolute, JointType::Prismatic, JointType::Fixed] {
            for axis in axes {
                let motion = joint_motion_subspace(joint_type, axis);
                let expected = inertia * motion.to_vector();
                let actual = wrench_vector(inertia_apply_joint(&inertia, joint_type, axis));
                assert_relative_eq!(actual, expected, epsilon = 1e-12, max_relative = 1e-12);
            }
        }
    }
}
