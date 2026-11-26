use nalgebra::{SVector, Vector3};

use crate::{
    BaseState, Error, Frame, JointType, Result, Twist, Wrench,
    spatial::{
        cross::{force_cross, motion_cross},
        inertia::{ArticulatedInertia, Matrix6},
        transform::{force_to_parent, motion_to_child},
    },
};

use super::super::topology::{child_link_index, incoming_joint_index};
use super::super::{
    BaseKind, FLOATING_BASE_DOF, Fixed, Floating, FloatingRobot, IndexedLoad, Model, Robot,
    Workspace,
};

type Vector6 = SVector<f64, 6>;

struct ForwardDynamicsScratch<'a> {
    link_loads: &'a mut [Wrench],
    spatial_velocities: &'a mut [Twist],
    bias_accelerations: &'a mut [Twist],
    spatial_accelerations: &'a mut [Twist],
    articulated_inertias: &'a mut [ArticulatedInertia],
    articulated_bias_forces: &'a mut [Wrench],
    articulated_u: &'a mut [Wrench],
    articulated_d: &'a mut [f64],
    articulated_joint_bias: &'a mut [f64],
}

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
        self.model.forward_dynamics::<Fixed>(
            &self.world_from_root,
            Twist::zeros(),
            self.world_gravity,
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
    /// Writes generalized accelerations computed with the articulated-body algorithm (ABA).
    ///
    /// `generalized_forces` and `output` have length `G = J + 6`, prepending
    /// world-frame angular then linear base components before the `J` joints.
    /// Uses `base.frame()` and `base.velocity()`; `base.acceleration()` is ignored.
    /// External loads use the same resisting-wrench convention as
    /// [`FloatingRobot::inverse_dynamics`].
    ///
    /// # Errors
    ///
    /// Returns an error for invalid lengths or load link IDs, or when an
    /// articulated inertia is singular or ill-conditioned.
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
        self.model.forward_dynamics::<Floating>(
            base.frame(),
            base.velocity(),
            self.world_gravity,
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
    pub(super) fn forward_dynamics<B: BaseKind>(
        &self,
        base_frame: &Frame,
        base_velocity: Twist,
        world_gravity: Vector3<f64>,
        q: &[f64],
        qd: &[f64],
        generalized_forces: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_output::<B>("forward dynamics generalized forces", generalized_forces)?;
        self.validate_output::<B>("forward dynamics output", output)?;
        if !generalized_forces.iter().all(|value| value.is_finite()) {
            return Err(Error::NonFiniteInput {
                input: "generalized forces",
            });
        }
        self.populate_parent_from_child(q, &mut workspace.frames)?;
        self.forward_dynamics_kernel::<B>(
            base_frame,
            base_velocity,
            world_gravity,
            &workspace.frames,
            qd,
            generalized_forces,
            loads,
            ForwardDynamicsScratch {
                link_loads: &mut workspace.link_loads,
                spatial_velocities: &mut workspace.spatial_velocities,
                bias_accelerations: &mut workspace.bias_accelerations,
                spatial_accelerations: &mut workspace.spatial_accelerations,
                articulated_inertias: &mut workspace.articulated_inertias,
                articulated_bias_forces: &mut workspace.articulated_bias_forces,
                articulated_u: &mut workspace.articulated_u,
                articulated_d: &mut workspace.articulated_d,
                articulated_joint_bias: &mut workspace.articulated_joint_bias,
            },
            output,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::robot) fn forward_dynamics_from_transforms<B: BaseKind>(
        &self,
        base_frame: &Frame,
        base_velocity: Twist,
        world_gravity: Vector3<f64>,
        parent_from_child: &[Frame],
        qd: &[f64],
        generalized_forces: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_output::<B>("forward dynamics generalized forces", generalized_forces)?;
        self.validate_output::<B>("forward dynamics output", output)?;
        if !generalized_forces.iter().all(|value| value.is_finite()) {
            return Err(Error::NonFiniteInput {
                input: "generalized forces",
            });
        }
        self.forward_dynamics_kernel::<B>(
            base_frame,
            base_velocity,
            world_gravity,
            parent_from_child,
            qd,
            generalized_forces,
            loads,
            ForwardDynamicsScratch {
                link_loads: &mut workspace.link_loads,
                spatial_velocities: &mut workspace.spatial_velocities,
                bias_accelerations: &mut workspace.bias_accelerations,
                spatial_accelerations: &mut workspace.spatial_accelerations,
                articulated_inertias: &mut workspace.articulated_inertias,
                articulated_bias_forces: &mut workspace.articulated_bias_forces,
                articulated_u: &mut workspace.articulated_u,
                articulated_d: &mut workspace.articulated_d,
                articulated_joint_bias: &mut workspace.articulated_joint_bias,
            },
            output,
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[allow(
        clippy::needless_range_loop,
        reason = "joint indices address parallel model and workspace arrays in tree order"
    )]
    fn forward_dynamics_kernel<B: BaseKind>(
        &self,
        base_frame: &Frame,
        base_velocity: Twist,
        world_gravity: Vector3<f64>,
        parent_from_child: &[Frame],
        qd: &[f64],
        generalized_forces: &[f64],
        loads: &[IndexedLoad],
        scratch: ForwardDynamicsScratch<'_>,
        output: &mut [f64],
    ) -> Result<()> {
        let root_load = self.prepare_indexed_loads(loads, scratch.link_loads)?;
        let root_rotation_inverse = base_frame.rotation.inverse();
        let root_velocity = Twist::new(
            root_rotation_inverse * base_velocity.angular,
            root_rotation_inverse * base_velocity.linear,
        );

        let mut root_inertia =
            ArticulatedInertia::from_spatial(self.link_dynamics[0].spatial_inertia);
        let root_momentum = root_inertia.apply(root_velocity);
        let mut root_bias_force = force_cross(root_velocity, root_momentum) + root_load;

        // First pass: velocities, velocity bias, body inertia, and bias force.
        for joint_index in 0..self.model_joint_count() {
            let joint = self.joint_kinematics[joint_index];
            let transform = parent_from_child[joint_index];
            let parent_link_index = self.parent_link_indices[joint_index];
            let parent_velocity = if parent_link_index == 0 {
                root_velocity
            } else {
                scratch.spatial_velocities[incoming_joint_index(parent_link_index)]
            };
            let motion_subspace = joint_motion_subspace(joint.joint_type, *joint.axis.as_ref());
            let joint_velocity = self.joint_value(qd, joint_index) * motion_subspace;
            let velocity = motion_to_child(&transform, parent_velocity) + joint_velocity;
            let link = self.link_dynamics[child_link_index(joint_index)];
            let inertia = ArticulatedInertia::from_spatial(link.spatial_inertia);
            let momentum = inertia.apply(velocity);

            scratch.spatial_velocities[joint_index] = velocity;
            scratch.bias_accelerations[joint_index] = motion_cross(velocity, joint_velocity);
            scratch.articulated_inertias[joint_index] = inertia;
            scratch.articulated_bias_forces[joint_index] =
                force_cross(velocity, momentum) + scratch.link_loads[joint_index];
        }

        // Second pass: eliminate active joint accelerations and propagate each
        // articulated subtree into its parent.
        for joint_index in (0..self.model_joint_count()).rev() {
            let joint = self.joint_kinematics[joint_index];
            let parent_link_index = self.parent_link_indices[joint_index];
            let skip_root_propagation = parent_link_index == 0 && !B::IS_FLOATING;
            let inertia = scratch.articulated_inertias[joint_index];
            let bias_force = scratch.articulated_bias_forces[joint_index];
            let bias_acceleration = scratch.bias_accelerations[joint_index];
            let (reduced_inertia, reduced_bias_force) = if let Some(dof_index) =
                self.joint_dof_indices[joint_index]
            {
                let motion_subspace = joint_motion_subspace(joint.joint_type, *joint.axis.as_ref());
                let articulated_u = inertia.apply_joint(joint.joint_type, *joint.axis.as_ref());
                let articulated_d = motion_subspace.dot_wrench(articulated_u);
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
                let joint_bias =
                    generalized_forces[B::DOF + dof_index] - motion_subspace.dot_wrench(bias_force);
                scratch.articulated_u[joint_index] = articulated_u;
                scratch.articulated_d[joint_index] = articulated_d;
                scratch.articulated_joint_bias[joint_index] = joint_bias;

                // A fixed root has prescribed acceleration and no inertia solve.
                // Keep the validated joint terms needed by the third pass, but
                // do not reduce/transform a subtree into that unused root state.
                if skip_root_propagation {
                    continue;
                }

                let u = articulated_u.to_vector();
                let reduced_inertia =
                    ArticulatedInertia(inertia.0 - (u * u.transpose()) / articulated_d);
                let reduced_bias_force = bias_force
                    + reduced_inertia.apply(bias_acceleration)
                    + (joint_bias / articulated_d) * articulated_u;
                (reduced_inertia, reduced_bias_force)
            } else {
                if skip_root_propagation {
                    continue;
                }
                (inertia, bias_force + inertia.apply(bias_acceleration))
            };

            let parent_inertia =
                reduced_inertia.transform_to_parent(&parent_from_child[joint_index]);
            let parent_bias_force =
                force_to_parent(&parent_from_child[joint_index], reduced_bias_force);
            if parent_link_index == 0 {
                root_inertia += parent_inertia;
                root_bias_force += parent_bias_force;
            } else {
                let parent_joint_index = incoming_joint_index(parent_link_index);
                scratch.articulated_inertias[parent_joint_index] += parent_inertia;
                scratch.articulated_bias_forces[parent_joint_index] += parent_bias_force;
            }
        }

        output.fill(0.0);
        let gravity_local = root_rotation_inverse * (-world_gravity);
        let root_acceleration = if !B::IS_FLOATING {
            Twist::new(Vector3::zeros(), gravity_local)
        } else {
            let world_base_force = Wrench::new(
                Vector3::from_column_slice(&generalized_forces[..3]),
                Vector3::from_column_slice(&generalized_forces[3..FLOATING_BASE_DOF]),
            );
            let local_base_force = Wrench::new(
                root_rotation_inverse * world_base_force.torque,
                root_rotation_inverse * world_base_force.force,
            );
            let right_hand_side = (local_base_force - root_bias_force).to_vector();
            // Average without overflowing a finite diagonal before scaling.
            let symmetric_inertia = root_inertia.0 * 0.5 + root_inertia.0.transpose() * 0.5;
            let acceleration =
                Twist::from_vector(solve_base_inertia(symmetric_inertia, right_hand_side)?);

            let physical_linear_local = acceleration.linear - gravity_local
                + root_velocity.angular.cross(&root_velocity.linear);
            let world_angular = base_frame.rotation * acceleration.angular;
            let world_linear = base_frame.rotation * physical_linear_local;
            if !Twist::new(world_angular, world_linear).is_finite() {
                return Err(Error::NumericalFailure {
                    operation: "floating-base acceleration",
                });
            }
            output[..3].copy_from_slice(world_angular.as_slice());
            output[3..FLOATING_BASE_DOF].copy_from_slice(world_linear.as_slice());
            acceleration
        };

        // Third pass: recover joint accelerations and complete link accelerations.
        for joint_index in 0..self.model_joint_count() {
            let parent_link_index = self.parent_link_indices[joint_index];
            let parent_acceleration = if parent_link_index == 0 {
                root_acceleration
            } else {
                scratch.spatial_accelerations[incoming_joint_index(parent_link_index)]
            };
            let mut acceleration =
                motion_to_child(&parent_from_child[joint_index], parent_acceleration)
                    + scratch.bias_accelerations[joint_index];
            if let Some(dof_index) = self.joint_dof_indices[joint_index] {
                let joint_acceleration = (scratch.articulated_joint_bias[joint_index]
                    - acceleration.dot_wrench(scratch.articulated_u[joint_index]))
                    / scratch.articulated_d[joint_index];
                if !joint_acceleration.is_finite() {
                    return Err(Error::NumericalFailure {
                        operation: "joint acceleration",
                    });
                }
                let motion_subspace = joint_motion_subspace(
                    self.joint_kinematics[joint_index].joint_type,
                    *self.joint_kinematics[joint_index].axis.as_ref(),
                );
                acceleration += joint_acceleration * motion_subspace;
                output[B::DOF + dof_index] = joint_acceleration;
            }
            scratch.spatial_accelerations[joint_index] = acceleration;
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

fn joint_motion_subspace(joint_type: JointType, axis: Vector3<f64>) -> Twist {
    match joint_type {
        JointType::Revolute => Twist::new(axis, Vector3::zeros()),
        JointType::Prismatic => Twist::new(Vector3::zeros(), axis),
        JointType::Fixed => Twist::zeros(),
    }
}

#[cfg(test)]
#[cfg_attr(coverage_nightly, coverage(off))]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

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
    fn base_solve_rejects_overflow_when_unscaling_a_finite_solution() {
        // The scaled RHS is 1e200 (finite), but the physical solution is 1e400.
        assert!(matches!(
            solve_base_inertia(Matrix6::identity() * 1e-300, Vector6::repeat(1e100)),
            Err(Error::NumericalFailure {
                operation: "floating-base inertia solve"
            })
        ));
        let solution =
            solve_base_inertia(Matrix6::identity() * 1e-300, Vector6::repeat(1e-100)).unwrap();
        assert_relative_eq!(solution, Vector6::repeat(1e200), max_relative = 1e-14);
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
}
