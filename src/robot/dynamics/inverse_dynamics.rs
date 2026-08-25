use nalgebra::Vector3;

use crate::{BaseState, Frame, JointType, Result, Twist, Wrench};

use super::super::{FLOATING_BASE_DOF, IndexedLoad, Model, Robot, Workspace};
use super::{add_wrench, wrench_to_parent, write_world_wrench};

const GRAVITY: f64 = 9.80665;

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
    /// Writes velocity-product generalized forces `C(q, qd) * qd`.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid base state or input/output length.
    pub fn velocity_product_forces(
        &mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        output: &mut [f64],
    ) -> Result<()> {
        self.model
            .velocity_product_forces(base, q, qd, &mut self.workspace, output)
    }

    /// Writes Newton-Euler generalized forces into caller-owned output.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid base state, input/output length, or load link ID.
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
        self.model
            .inverse_dynamics(base, q, qd, qdd, loads, &mut self.workspace, output)
    }

    /// Writes gravity and external-load generalized forces into caller-owned output.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid base state, input/output length, or load link ID.
    pub fn gravity(
        &mut self,
        base: &BaseState,
        q: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> Result<()> {
        self.model
            .gravity(base, q, loads, &mut self.workspace, output)
    }
}

impl Model {
    /// Writes velocity-product generalized forces `C(q, qd) * qd`.
    ///
    /// Gravity, prescribed base acceleration, and external loads are excluded.
    /// For a floating base, the supplied base velocity participates in the result
    /// and output is ordered `[base torque, base force, joint forces]`. The
    /// base wrench is expressed in the world frame at the root origin.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input or output lengths.
    fn velocity_product_forces(
        &self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_base_state(base)?;
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
            base.frame(),
            base.velocity(),
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
            write_world_wrench(base.frame(), base_load, &mut output[..FLOATING_BASE_DOF]);
        }
        Ok(())
    }

    /// Writes runtime-sized Newton-Euler generalized forces into caller-owned output.
    ///
    /// Base pose and classical motion come from the supplied [`BaseState`]. Floating-base
    /// output is ordered `[base torque, base force, joint forces]`, with the
    /// base wrench expressed in the world frame at the root origin.
    ///
    /// $$
    /// \tau = M(q) \dot\nu + C(q, \nu) \nu + g(q).
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error for invalid lengths or link IDs.
    #[allow(clippy::too_many_arguments)]
    fn inverse_dynamics(
        &self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_base_state(base)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        self.validate_output("inverse dynamics output", output)?;
        self.inverse_dynamics_for_base(
            q,
            qd,
            qdd,
            base.frame(),
            base.velocity(),
            base.acceleration(),
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
    /// Returns an error for invalid lengths or link IDs.
    fn gravity(
        &self,
        base: &BaseState,
        q: &[f64],
        loads: &[IndexedLoad],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_base_state(base)?;
        self.validate_slice("q", q)?;
        self.validate_output("gravity output", output)?;
        self.gravity_for_base(q, base.frame(), loads, workspace, output)
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
            let parent = self.parent_link_indices[i];
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
            let parent = self.parent_link_indices[i];
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
            let parent = self.parent_link_indices[i];
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
            let parent = self.parent_link_indices[i];
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

#[cfg(test)]
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
        let base = BaseState::fixed();
        let q = [0.0; 4];
        let mut output = [0.0; 4];

        robot.workspace.frames.pop();
        assert_wrong_workspace_length(
            robot.velocity_product_forces(&base, &q, &q, &mut output),
            "transform workspace",
        );

        let mut robot = fixture();
        robot.workspace.frames.pop();
        assert_wrong_workspace_length(
            robot.inverse_dynamics(&base, &q, &q, &q, &[], &mut output),
            "transform workspace",
        );

        let mut robot = fixture();
        robot.workspace.frames.pop();
        assert_wrong_workspace_length(
            robot.gravity(&base, &q, &[], &mut output),
            "transform workspace",
        );

        let mut robot = fixture();
        robot.workspace.angular_accelerations.pop();
        assert_wrong_workspace_length(
            robot.gravity(&base, &q, &[], &mut output),
            "gravity workspace",
        );

        let mut robot = fixture();
        robot.workspace.link_loads.pop();
        assert_wrong_workspace_length(robot.gravity(&base, &q, &[], &mut output), "load workspace");
    }
}
