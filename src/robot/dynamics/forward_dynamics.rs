use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::{BaseState, Error, Frame, JointType, Result, Twist, Wrench};

use super::super::topology::{child_link_index, incoming_joint_index};
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
    #[allow(
        clippy::needless_range_loop,
        reason = "joint indices address parallel model and workspace arrays in tree order"
    )]
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
        if !generalized_forces.iter().all(|value| value.is_finite()) {
            return Err(Error::NonFiniteInput {
                input: "generalized forces",
            });
        }

        let root_load = self.prepare_indexed_loads(loads, &mut workspace.link_loads)?;
        let parent_from_child = &mut workspace.frames;
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
        for joint_index in 0..self.model_joint_count() {
            let joint = self.joint_kinematics[joint_index];
            let transform = joint.frame(self.joint_value(q, joint_index));
            let parent_link_index = self.parent_link_indices[joint_index];
            let parent_velocity = if parent_link_index == 0 {
                root_velocity
            } else {
                workspace.spatial_velocities[incoming_joint_index(parent_link_index)]
            };
            let motion_subspace = joint_motion_subspace(joint.joint_type, *joint.axis.as_ref());
            let joint_velocity = scale_twist(motion_subspace, self.joint_value(qd, joint_index));
            let velocity = add_twist(motion_to_child(&transform, parent_velocity), joint_velocity);
            let link = self.link_dynamics[child_link_index(joint_index)];
            let inertia = rigid_body_inertia(link.mass, link.first_moment, link.origin_inertia);
            let momentum = inertia_apply(&inertia, velocity);

            parent_from_child[joint_index] = transform;
            workspace.spatial_velocities[joint_index] = velocity;
            workspace.bias_accelerations[joint_index] = motion_cross(velocity, joint_velocity);
            workspace.articulated_inertias[joint_index] = inertia;
            workspace.articulated_bias_forces[joint_index] = add_wrench(
                force_cross(velocity, momentum),
                workspace.link_loads[joint_index],
            );
        }

        let joint_offset = base_dof_count(base_mode);

        // Second pass: eliminate active joint accelerations and propagate each
        // articulated subtree into its parent.
        for joint_index in (0..self.model_joint_count()).rev() {
            let joint = self.joint_kinematics[joint_index];
            let parent_link_index = self.parent_link_indices[joint_index];
            let skip_root_propagation =
                parent_link_index == 0 && matches!(base_mode, RootMode::Fixed);
            let inertia = workspace.articulated_inertias[joint_index];
            let bias_force = workspace.articulated_bias_forces[joint_index];
            let bias_acceleration = workspace.bias_accelerations[joint_index];
            let (reduced_inertia, reduced_bias_force) = if let Some(dof_index) =
                self.joint_dof_indices[joint_index]
            {
                let motion_subspace = joint_motion_subspace(joint.joint_type, *joint.axis.as_ref());
                let articulated_u =
                    inertia_apply_joint(&inertia, joint.joint_type, *joint.axis.as_ref());
                let articulated_d = motion_force_dot(motion_subspace, articulated_u);
                if !articulated_d.is_finite() {
                    return Err(Error::NumericalFailure {
                        operation: "joint articulated inertia",
                    });
                }
                if articulated_d <= 0.0 {
                    return Err(Error::ForwardDynamicsSingularJointInertia {
                        joint_index: dof_index,
                    });
                }
                let joint_bias = generalized_forces[joint_offset + dof_index]
                    - motion_force_dot(motion_subspace, bias_force);
                workspace.articulated_u[joint_index] = articulated_u;
                workspace.articulated_d[joint_index] = articulated_d;
                workspace.articulated_joint_bias[joint_index] = joint_bias;

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
                transform_inertia_to_parent(&parent_from_child[joint_index], &reduced_inertia);
            let parent_bias_force =
                super::wrench_to_parent(&parent_from_child[joint_index], reduced_bias_force);
            if parent_link_index == 0 {
                root_inertia += parent_inertia;
                root_bias_force = add_wrench(root_bias_force, parent_bias_force);
            } else {
                let parent_joint_index = incoming_joint_index(parent_link_index);
                workspace.articulated_inertias[parent_joint_index] += parent_inertia;
                workspace.articulated_bias_forces[parent_joint_index] = add_wrench(
                    workspace.articulated_bias_forces[parent_joint_index],
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
                // Average without overflowing a finite diagonal before scaling.
                let symmetric_inertia = root_inertia * 0.5 + root_inertia.transpose() * 0.5;
                let acceleration =
                    twist_from_vector(solve_base_inertia(symmetric_inertia, right_hand_side)?);

                let physical_linear_local = acceleration.linear - gravity_local
                    + root_velocity.angular.cross(&root_velocity.linear);
                let world_angular = base_frame.rotation * acceleration.angular;
                let world_linear = base_frame.rotation * physical_linear_local;
                if !twist_is_finite(Twist::new(world_angular, world_linear)) {
                    return Err(Error::NumericalFailure {
                        operation: "floating-base acceleration",
                    });
                }
                output[..3].copy_from_slice(world_angular.as_slice());
                output[3..FLOATING_BASE_DOF].copy_from_slice(world_linear.as_slice());
                acceleration
            }
        };

        // Third pass: recover joint accelerations and complete link accelerations.
        for joint_index in 0..self.model_joint_count() {
            let parent_link_index = self.parent_link_indices[joint_index];
            let parent_acceleration = if parent_link_index == 0 {
                root_acceleration
            } else {
                workspace.spatial_accelerations[incoming_joint_index(parent_link_index)]
            };
            let mut acceleration = add_twist(
                motion_to_child(&parent_from_child[joint_index], parent_acceleration),
                workspace.bias_accelerations[joint_index],
            );
            if let Some(dof_index) = self.joint_dof_indices[joint_index] {
                let joint_acceleration = (workspace.articulated_joint_bias[joint_index]
                    - motion_force_dot(acceleration, workspace.articulated_u[joint_index]))
                    / workspace.articulated_d[joint_index];
                if !joint_acceleration.is_finite() {
                    return Err(Error::NumericalFailure {
                        operation: "joint acceleration",
                    });
                }
                let motion_subspace = joint_motion_subspace(
                    self.joint_kinematics[joint_index].joint_type,
                    *self.joint_kinematics[joint_index].axis.as_ref(),
                );
                acceleration = add_twist(
                    acceleration,
                    scale_twist(motion_subspace, joint_acceleration),
                );
                output[joint_offset + dof_index] = joint_acceleration;
            }
            workspace.spatial_accelerations[joint_index] = acceleration;
        }
        Ok(())
    }
}

// Diagonal equilibration removes coordinate-unit and body-scale differences
// before condition testing. B = D^-1 A D^-1, B y = D^-1 b, x = D^-1 y.
// All storage is fixed-size; the condition check reuses Cholesky solves without
// storing a matrix inverse or allocating on the heap.
fn solve_base_inertia(inertia: Matrix6, rhs: Vector6) -> Result<Vector6> {
    let numerical_failure = || Error::NumericalFailure {
        operation: "floating-base inertia solve",
    };
    if !inertia.iter().chain(rhs.iter()).all(|v| v.is_finite()) {
        return Err(numerical_failure());
    }
    let diagonal = inertia.diagonal();
    if diagonal.iter().any(|v| *v <= 0.0) {
        return Err(Error::ForwardDynamicsSingularBaseInertia);
    }
    let scale = diagonal.map(f64::sqrt);
    // Sequential division avoids overflow/underflow in products of scales.
    let scaled = Matrix6::from_fn(|row, col| inertia[(row, col)] / scale[row] / scale[col]);
    let scaled_rhs = rhs.component_div(&scale);
    if !scaled
        .iter()
        .chain(scaled_rhs.iter())
        .all(|v| v.is_finite())
    {
        return Err(numerical_failure());
    }
    let factor = scaled
        .cholesky()
        .ok_or(Error::ForwardDynamicsSingularBaseInertia)?;
    let matrix_norm = (0..6)
        .map(|col| scaled.column(col).iter().map(|v| v.abs()).sum::<f64>())
        .fold(0.0, f64::max);
    // At size six, solving for the six basis vectors is cheaper than a second
    // eigendecomposition. Their absolute column sums give ||B^-1||_1.
    let mut inverse_norm = 0.0_f64;
    for column in 0..6 {
        let inverse_column = factor.solve(&Vector6::ith(column, 1.0));
        if !inverse_column.iter().all(|v| v.is_finite()) {
            return Err(Error::ForwardDynamicsIllConditionedBaseInertia);
        }
        inverse_norm = inverse_norm.max(inverse_column.iter().map(|v| v.abs()).sum());
    }
    let reciprocal_condition = (1.0 / matrix_norm) / inverse_norm;
    if reciprocal_condition <= f64::EPSILON.sqrt() {
        return Err(Error::ForwardDynamicsIllConditionedBaseInertia);
    }
    let y = factor.solve(&scaled_rhs);
    let solution = y.component_div(&scale);
    if !y.iter().chain(solution.iter()).all(|v| v.is_finite()) {
        return Err(numerical_failure());
    }
    // Normwise backward-error check. Rescale y and b together so the check
    // itself does not overflow for large but representable loads/solutions.
    let magnitude = y.amax().max(scaled_rhs.amax());
    if magnitude > 0.0 {
        let normalized_y = y / magnitude;
        let normalized_rhs = scaled_rhs / magnitude;
        let residual = scaled * normalized_y - normalized_rhs;
        // B is symmetric, so its infinity norm equals its one-norm above.
        let denominator = matrix_norm * normalized_y.amax() + normalized_rhs.amax();
        if !residual.iter().all(|v| v.is_finite())
            || residual.amax() > 128.0 * f64::EPSILON * denominator
        {
            return Err(numerical_failure());
        }
    }
    Ok(solution)
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
    fn scaled_base_solve_is_invariant_to_diagonal_units_and_mass_scale() {
        let factor = Matrix6::from_fn(|r, c| {
            if r == c {
                2.0
            } else {
                ((r + c + 1) as f64).sin() * 0.1
            }
        });
        let conditioned = factor * factor.transpose();
        let expected = Vector6::new(0.2, -0.3, 0.4, -0.5, 0.6, -0.7);
        for mass_scale in [1e-100_f64, 1e-6, 1.0, 1e6, 1e100] {
            let scales = Vector6::new(1e-9, 1e-6, 1e-3, 1.0, 1e3, 1e9) * mass_scale.sqrt();
            let inertia = Matrix6::from_fn(|r, c| scales[r] * conditioned[(r, c)] * scales[c]);
            let rhs = scales.component_mul(&(conditioned * expected));
            let result = solve_base_inertia(inertia, rhs).unwrap();
            assert_relative_eq!(result.component_mul(&scales), expected, epsilon = 1e-13);
        }
    }

    #[test]
    fn base_solve_distinguishes_singularity_conditioning_and_overflow() {
        assert!(matches!(
            solve_base_inertia(Matrix6::zeros(), Vector6::zeros()),
            Err(Error::ForwardDynamicsSingularBaseInertia)
        ));
        let mut ill_conditioned = Matrix6::identity();
        ill_conditioned[(0, 1)] = 1.0 - 1e-12;
        ill_conditioned[(1, 0)] = 1.0 - 1e-12;
        assert!(matches!(
            solve_base_inertia(ill_conditioned, Vector6::repeat(1.0)),
            Err(Error::ForwardDynamicsIllConditionedBaseInertia)
        ));
        assert!(matches!(
            solve_base_inertia(Matrix6::identity() * 1e-300, Vector6::repeat(1e300)),
            Err(Error::NumericalFailure { .. })
        ));
        let mut singular = Matrix6::identity();
        singular[(0, 1)] = 1.0;
        singular[(1, 0)] = 1.0;
        assert!(matches!(
            solve_base_inertia(singular, Vector6::zeros()),
            Err(Error::ForwardDynamicsSingularBaseInertia)
        ));
    }

    #[test]
    fn base_solve_rejects_non_finite_matrix_and_rhs_entries() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for index in 0..6 {
                let mut rhs = Vector6::zeros();
                rhs[index] = value;
                assert!(matches!(
                    solve_base_inertia(Matrix6::identity(), rhs),
                    Err(Error::NumericalFailure {
                        operation: "floating-base inertia solve"
                    })
                ));
                for column in 0..6 {
                    let mut inertia = Matrix6::identity();
                    inertia[(index, column)] = value;
                    assert!(matches!(
                        solve_base_inertia(inertia, Vector6::zeros()),
                        Err(Error::NumericalFailure {
                            operation: "floating-base inertia solve"
                        })
                    ));
                }
            }
        }
        assert_eq!(
            solve_base_inertia(Matrix6::identity(), Vector6::repeat(2.0)).unwrap(),
            Vector6::repeat(2.0)
        );
    }

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
