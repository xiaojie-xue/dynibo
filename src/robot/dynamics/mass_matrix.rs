use nalgebra::{Matrix3, Vector3};

use crate::{BaseState, JointType, Result, Wrench};

use super::super::{FLOATING_BASE_DOF, FloatingRobot, Model, Robot, Workspace};
use super::{wrench_component, wrench_to_parent, write_wrench_to_column};

impl Robot {
    /// Writes the `G x G` mass matrix in column-major order.
    ///
    /// Rows and columns follow the generalized-vector ordering documented by
    /// [`Robot::generalized_count`].
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid input or output length.
    pub fn mass_matrix(&mut self, q: &[f64], output: &mut [f64]) -> Result<()> {
        self.model.fixed_mass_matrix(q, &mut self.workspace, output)
    }
}

impl FloatingRobot {
    /// Writes the `G x G` mass matrix in column-major order.
    pub fn mass_matrix(&mut self, base: &BaseState, q: &[f64], output: &mut [f64]) -> Result<()> {
        self.model
            .floating_mass_matrix(base, q, &mut self.workspace, output)
    }
}

impl Model {
    fn fixed_mass_matrix(
        &self,
        q: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice_length(
            "mass matrix output",
            output.len(),
            self.joint_count() * self.joint_count(),
        )?;
        self.mass_matrix_kernel(q, workspace, output);
        Ok(())
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
    /// or for an invalid input length.
    fn floating_mass_matrix(
        &self,
        base: &BaseState,
        q: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice_length(
            "mass matrix output",
            output.len(),
            (self.joint_count() + FLOATING_BASE_DOF) * (self.joint_count() + FLOATING_BASE_DOF),
        )?;
        self.floating_mass_matrix_kernel(base, q, workspace, output);
        Ok(())
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
            let parent = self.parent_link_indices[index];
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
                let parent = self.parent_link_indices[current];
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
        base: &BaseState,
        q: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) {
        let model_joint_count = self.model_joint_count();
        let generalized_count = self.joint_count() + FLOATING_BASE_DOF;
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
            let parent = self.parent_link_indices[index];
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

        let base_rotation = base.frame().rotation;
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
                let parent = self.parent_link_indices[current];
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
}
