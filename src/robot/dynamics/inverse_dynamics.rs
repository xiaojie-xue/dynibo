use nalgebra::Vector3;

use crate::{
    BaseState, Frame, JointType, Result, Twist, Wrench, spatial::transform::force_to_parent,
};

use super::super::topology::{child_link_index, incoming_joint_index};
use super::super::{
    BaseKind, BaseMotion, FLOATING_BASE_DOF, Fixed, Floating, FloatingRobot, IndexedLoad, Model,
    Robot, Workspace,
};
use super::write_world_wrench;

/// Link-local Newton–Euler intermediates, initialized in the outward pass.
struct DynamicsScratch<'a> {
    parent_from_child: &'a mut [Frame],
    angular_velocities: &'a mut [Vector3<f64>],
    angular_accelerations: &'a mut [Vector3<f64>],
    origin_accelerations: &'a mut [Vector3<f64>],
    link_accelerations: &'a mut [Vector3<f64>],
    link_loads: &'a mut [Wrench],
}

/// Reuses dynamics buffers: the outward pass writes local gravity and transforms,
/// then the inward pass accumulates the already prepared external loads.
struct GravityScratch<'a> {
    parent_from_child: &'a mut [Frame],
    gravity_at_link: &'a mut [Vector3<f64>],
    link_loads: &'a mut [Wrench],
}

impl Robot {
    /// Writes velocity-product generalized forces `C(q, qd) * qd`.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input, an invalid output length, or a non-finite numerical result.
    pub fn velocity_product_forces(
        &mut self,
        q: &[f64],
        qd: &[f64],
        output: &mut [f64],
    ) -> Result<()> {
        self.model.velocity_product_forces::<Fixed>(
            BaseMotion::stationary(&self.world_from_root),
            q,
            qd,
            &mut self.workspace,
            output,
        )
    }

    /// Writes Newton-Euler generalized forces into caller-owned output.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input, an invalid output length or load link ID, or a non-finite result.
    #[allow(clippy::too_many_arguments)]
    pub fn inverse_dynamics(
        &mut self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> Result<()> {
        self.model.inverse_dynamics::<Fixed>(
            BaseMotion::stationary(&self.world_from_root),
            self.world_gravity,
            q,
            qd,
            qdd,
            loads,
            &mut self.workspace,
            output,
        )
    }

    /// Writes gravity and external-load generalized forces into caller-owned output.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input, an invalid output length or load link ID, or a non-finite result.
    pub fn gravity(&mut self, q: &[f64], loads: &[IndexedLoad], output: &mut [f64]) -> Result<()> {
        self.model.gravity::<Fixed>(
            &self.world_from_root,
            self.world_gravity,
            q,
            loads,
            &mut self.workspace,
            output,
        )
    }
}

impl FloatingRobot {
    /// Writes velocity-product generalized forces `C(q, qd) * qd`.
    pub fn velocity_product_forces(
        &mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        output: &mut [f64],
    ) -> Result<()> {
        self.model.velocity_product_forces::<Floating>(
            BaseMotion::from_state(base),
            q,
            qd,
            &mut self.workspace,
            output,
        )
    }

    /// Writes Newton-Euler generalized forces into caller-owned output.
    #[allow(clippy::too_many_arguments)]
    pub fn inverse_dynamics(
        &mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> Result<()> {
        self.model.inverse_dynamics::<Floating>(
            BaseMotion::from_state(base),
            self.world_gravity,
            q,
            qd,
            qdd,
            loads,
            &mut self.workspace,
            output,
        )
    }

    /// Writes gravity and external-load generalized forces into caller-owned output.
    pub fn gravity(
        &mut self,
        base: &BaseState,
        q: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> Result<()> {
        self.model.gravity::<Floating>(
            base.frame(),
            self.world_gravity,
            q,
            loads,
            &mut self.workspace,
            output,
        )
    }
}

impl Model {
    /// Writes velocity-product generalized forces `C(q, qd) * qd`.
    ///
    /// Gravity, prescribed base acceleration, and external loads are excluded.
    /// For a floating base, the supplied base velocity participates in the result
    /// and output is ordered `[base torque, base force, joint forces]`. The
    /// base wrench is expressed in the world frame at the root origin.
    pub(super) fn velocity_product_forces<B: BaseKind>(
        &self,
        base: BaseMotion<'_>,
        q: &[f64],
        qd: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_output::<B>("velocity product output", output)?;
        workspace.step.fill(0.0);
        workspace.link_loads.fill(Wrench::zeros());
        output.fill(0.0);
        let base_load = self.inverse_dynamics_kernel_validated::<B>(
            q,
            qd,
            &workspace.step,
            base.frame,
            base.velocity,
            Twist::zeros(),
            Vector3::zeros(),
            Wrench::zeros(),
            DynamicsScratch {
                parent_from_child: &mut workspace.frames,
                angular_velocities: &mut workspace.angular_velocities,
                angular_accelerations: &mut workspace.angular_accelerations,
                origin_accelerations: &mut workspace.origin_accelerations,
                link_accelerations: &mut workspace.link_accelerations,
                link_loads: &mut workspace.link_loads,
            },
            &mut output[B::DOF..],
        )?;
        if B::IS_FLOATING {
            write_world_wrench(base.frame, base_load, &mut output[..FLOATING_BASE_DOF]);
        }
        Self::validate_numerical_result("velocity product forces", output.iter())
    }

    /// Writes runtime-sized Newton-Euler generalized forces into caller-owned output.
    ///
    /// $$
    /// \tau = M(q) \dot\nu + C(q, \nu) \nu + g(q).
    /// $$
    #[allow(clippy::too_many_arguments)]
    pub(super) fn inverse_dynamics<B: BaseKind>(
        &self,
        base: BaseMotion<'_>,
        world_gravity: Vector3<f64>,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        self.validate_output::<B>("inverse dynamics output", output)?;
        let root_load = self.prepare_indexed_loads(loads, &mut workspace.link_loads)?;
        output.fill(0.0);
        let base_load = self.inverse_dynamics_kernel_validated::<B>(
            q,
            qd,
            qdd,
            base.frame,
            base.velocity,
            base.acceleration,
            -world_gravity,
            root_load,
            DynamicsScratch {
                parent_from_child: &mut workspace.frames,
                angular_velocities: &mut workspace.angular_velocities,
                angular_accelerations: &mut workspace.angular_accelerations,
                origin_accelerations: &mut workspace.origin_accelerations,
                link_accelerations: &mut workspace.link_accelerations,
                link_loads: &mut workspace.link_loads,
            },
            &mut output[B::DOF..],
        )?;
        if B::IS_FLOATING {
            write_world_wrench(base.frame, base_load, &mut output[..FLOATING_BASE_DOF]);
        }
        Self::validate_numerical_result("inverse dynamics", output.iter())
    }

    /// Writes runtime-sized gravity generalized forces into caller-owned output.
    ///
    /// With no external loads, this is the zero-velocity, zero-acceleration
    /// inverse-dynamics term:
    ///
    /// $$
    /// g(q) = \tau(q, 0, 0).
    /// $$
    pub(super) fn gravity<B: BaseKind>(
        &self,
        base_frame: &Frame,
        world_gravity: Vector3<f64>,
        q: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_output::<B>("gravity output", output)?;
        let root_load = self.prepare_indexed_loads(loads, &mut workspace.link_loads)?;
        output.fill(0.0);
        let base_load = self.gravity_kernel::<B>(
            q,
            base_frame,
            -world_gravity,
            root_load,
            GravityScratch {
                parent_from_child: &mut workspace.frames,
                gravity_at_link: &mut workspace.angular_accelerations,
                link_loads: &mut workspace.link_loads,
            },
            &mut output[B::DOF..],
        )?;
        if B::IS_FLOATING {
            write_world_wrench(base_frame, base_load, &mut output[..FLOATING_BASE_DOF]);
        }
        Self::validate_numerical_result("gravity", output.iter())
    }

    // Callers must validate user q/qd/qdd before entering this kernel. The
    // velocity-product callers instead supply a freshly zeroed qdd workspace.
    // Keep length checks here for that internal buffer, but do not rescan values.
    // Fixed-base callers do not consume the returned root reaction wrench.
    #[allow(clippy::too_many_arguments)]
    fn inverse_dynamics_kernel_validated<B: BaseKind>(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        base_frame: &Frame,
        base_velocity: Twist,
        base_acceleration: Twist,
        virtual_gravity: Vector3<f64>,
        root_load: Wrench,
        scratch: DynamicsScratch<'_>,
        output: &mut [f64],
    ) -> Result<Wrench> {
        self.validate_slice_length("q", q.len(), self.joint_count())?;
        self.validate_slice_length("qd", qd.len(), self.joint_count())?;
        self.validate_slice_length("qdd", qdd.len(), self.joint_count())?;
        self.validate_dynamics_scratch(&scratch)?;
        self.validate_joint_output("inverse dynamics joint output", output)?;
        let base_rotation_inverse = base_frame.rotation.inverse();
        let base_omega = base_rotation_inverse * base_velocity.angular;
        let base_angular_acceleration = base_rotation_inverse * base_acceleration.angular;
        let base_origin_acceleration =
            base_rotation_inverse * (virtual_gravity + base_acceleration.linear);

        for joint_index in 0..self.model_joint_count() {
            let joint = self.joint_kinematics[joint_index];
            let link = self.link_dynamics[child_link_index(joint_index)];
            let parent_link_index = self.parent_link_indices[joint_index];
            let (parent_omega, parent_alpha, parent_acceleration) = if parent_link_index == 0 {
                (
                    base_omega,
                    base_angular_acceleration,
                    base_origin_acceleration,
                )
            } else {
                let parent_joint_index = incoming_joint_index(parent_link_index);
                (
                    scratch.angular_velocities[parent_joint_index],
                    scratch.angular_accelerations[parent_joint_index],
                    scratch.origin_accelerations[parent_joint_index],
                )
            };
            let position = self.joint_value(q, joint_index);
            let velocity = self.joint_value(qd, joint_index);
            let acceleration_value = self.joint_value(qdd, joint_index);
            let transform = joint.frame(position);
            // Reuse one stack-local matrix for the three rotations below.
            let rotation_inverse = transform.rotation.inverse().to_rotation_matrix();
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
            scratch.angular_velocities[joint_index] = omega;
            scratch.angular_accelerations[joint_index] = alpha;
            scratch.origin_accelerations[joint_index] = acceleration;
            let center = &link.center_of_mass;
            scratch.link_accelerations[joint_index] =
                acceleration + alpha.cross(center) + omega.cross(&omega.cross(center));
            scratch.parent_from_child[joint_index] = transform;
        }

        let mut accumulated_root_load = if B::IS_FLOATING {
            let root = self.link_dynamics[0];
            let root_center_acceleration = base_origin_acceleration
                + base_angular_acceleration.cross(&root.center_of_mass)
                + base_omega.cross(&base_omega.cross(&root.center_of_mass));
            let root_force = root.mass * root_center_acceleration;
            root_load
                + Wrench::new(
                    root.center_of_mass.cross(&root_force)
                        + root.inertia * base_angular_acceleration
                        + base_omega.cross(&(root.inertia * base_omega)),
                    root_force,
                )
        } else {
            Wrench::zeros()
        };
        for joint_index in (0..self.model_joint_count()).rev() {
            let joint = self.joint_kinematics[joint_index];
            let link = self.link_dynamics[child_link_index(joint_index)];
            let inertial_force = link.mass * scratch.link_accelerations[joint_index];
            let angular_momentum = link.inertia * scratch.angular_velocities[joint_index];
            let inertial_load = Wrench::new(
                link.center_of_mass.cross(&inertial_force)
                    + link.inertia * scratch.angular_accelerations[joint_index]
                    + scratch.angular_velocities[joint_index].cross(&angular_momentum),
                inertial_force,
            );
            scratch.link_loads[joint_index] += inertial_load;
            if let Some(dof_index) = self.joint_dof_indices[joint_index] {
                output[dof_index] = match joint.joint_type {
                    JointType::Revolute => scratch.link_loads[joint_index]
                        .torque
                        .dot(joint.axis.as_ref()),
                    JointType::Prismatic => scratch.link_loads[joint_index]
                        .force
                        .dot(joint.axis.as_ref()),
                    JointType::Fixed => unreachable!("fixed joints have no DOF index"),
                };
            }
            let parent_link_index = self.parent_link_indices[joint_index];
            if parent_link_index != 0 {
                let parent_load = force_to_parent(
                    &scratch.parent_from_child[joint_index],
                    scratch.link_loads[joint_index],
                );
                let parent_joint_index = incoming_joint_index(parent_link_index);
                scratch.link_loads[parent_joint_index] += parent_load;
            } else if B::IS_FLOATING {
                accumulated_root_load += force_to_parent(
                    &scratch.parent_from_child[joint_index],
                    scratch.link_loads[joint_index],
                );
            }
        }
        Ok(accumulated_root_load)
    }

    fn gravity_kernel<B: BaseKind>(
        &self,
        q: &[f64],
        base_frame: &Frame,
        virtual_gravity: Vector3<f64>,
        root_load: Wrench,
        scratch: GravityScratch<'_>,
        output: &mut [f64],
    ) -> Result<Wrench> {
        self.validate_slice_length("q", q.len(), self.joint_count())?;
        self.validate_slice_length(
            "transform workspace",
            scratch.parent_from_child.len(),
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
        let base_gravity = base_frame.rotation.inverse() * virtual_gravity;
        for joint_index in 0..self.model_joint_count() {
            scratch.parent_from_child[joint_index] =
                self.joint_kinematics[joint_index].frame(self.joint_value(q, joint_index));
            let parent_link_index = self.parent_link_indices[joint_index];
            let parent_gravity = if parent_link_index == 0 {
                base_gravity
            } else {
                scratch.gravity_at_link[incoming_joint_index(parent_link_index)]
            };
            scratch.gravity_at_link[joint_index] =
                scratch.parent_from_child[joint_index].rotation.inverse() * parent_gravity;
        }
        let mut accumulated_root_load = if B::IS_FLOATING {
            let root = self.link_dynamics[0];
            let root_force = root.mass * base_gravity;
            root_load + Wrench::new(root.center_of_mass.cross(&root_force), root_force)
        } else {
            Wrench::zeros()
        };
        for joint_index in (0..self.model_joint_count()).rev() {
            let joint = self.joint_kinematics[joint_index];
            let link = self.link_dynamics[child_link_index(joint_index)];
            let force = link.mass * scratch.gravity_at_link[joint_index];
            let gravity_load = Wrench::new(link.center_of_mass.cross(&force), force);
            scratch.link_loads[joint_index] += gravity_load;
            if let Some(dof_index) = self.joint_dof_indices[joint_index] {
                output[dof_index] = match joint.joint_type {
                    JointType::Revolute => scratch.link_loads[joint_index]
                        .torque
                        .dot(joint.axis.as_ref()),
                    JointType::Prismatic => scratch.link_loads[joint_index]
                        .force
                        .dot(joint.axis.as_ref()),
                    JointType::Fixed => unreachable!("fixed joints have no DOF index"),
                };
            }
            let parent_link_index = self.parent_link_indices[joint_index];
            if parent_link_index != 0 {
                let parent_load = force_to_parent(
                    &scratch.parent_from_child[joint_index],
                    scratch.link_loads[joint_index],
                );
                let parent_joint_index = incoming_joint_index(parent_link_index);
                scratch.link_loads[parent_joint_index] += parent_load;
            } else if B::IS_FLOATING {
                accumulated_root_load += force_to_parent(
                    &scratch.parent_from_child[joint_index],
                    scratch.link_loads[joint_index],
                );
            }
        }
        Ok(accumulated_root_load)
    }

    pub(super) fn prepare_indexed_loads(
        &self,
        loads: &[IndexedLoad],
        output: &mut [Wrench],
    ) -> Result<Wrench> {
        output.fill(Wrench::zeros());
        let mut root_load = Wrench::zeros();
        for load in loads {
            let link_index = self.validate_link_id(load.link)?;
            if !load.wrench.is_finite() {
                return Err(crate::Error::NonFiniteInput { input: "load" });
            }
            let accumulated = if link_index == 0 {
                &mut root_load
            } else {
                &mut output[incoming_joint_index(link_index)]
            };
            *accumulated += load.wrench;
            if !accumulated.is_finite() {
                return Err(crate::Error::NumericalFailure {
                    operation: "load aggregation",
                });
            }
        }
        Ok(root_load)
    }

    fn validate_dynamics_scratch(&self, scratch: &DynamicsScratch<'_>) -> Result<()> {
        for (name, actual) in [
            ("transform workspace", scratch.parent_from_child.len()),
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

#[cfg(test)]
#[cfg_attr(coverage_nightly, coverage(off))]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use crate::Error;

    fn fixture() -> Robot {
        Robot::from_urdf(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf"))
            .unwrap()
    }

    fn assert_wrong_workspace_length<T>(result: Result<T>, slice: &'static str) {
        assert!(matches!(
            result,
            Err(Error::WrongSliceLength {
                slice: actual,
                ..
            }) if actual == slice
        ));
    }

    #[test]
    fn dynamics_kernels_reject_corrupted_workspace_buffers() {
        let mut robot = fixture();
        let q = [0.0; 4];
        let mut output = [0.0; 4];

        robot.workspace.frames.pop();
        assert_wrong_workspace_length(
            robot.velocity_product_forces(&q, &q, &mut output),
            "transform workspace",
        );

        let mut robot = fixture();
        robot.workspace.angular_velocities.pop();
        assert_wrong_workspace_length(
            robot.inverse_dynamics(&q, &q, &q, &[], &mut output),
            "angular velocity workspace",
        );

        let mut robot = fixture();
        robot.workspace.angular_accelerations.pop();
        assert_wrong_workspace_length(robot.gravity(&q, &[], &mut output), "gravity workspace");

        let mut robot = fixture();
        robot.workspace.link_loads.pop();
        assert_wrong_workspace_length(robot.gravity(&q, &[], &mut output), "load workspace");
    }
}
