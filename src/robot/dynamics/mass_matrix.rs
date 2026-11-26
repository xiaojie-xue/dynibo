use nalgebra::Vector3;

use crate::{
    BaseState, Frame, JointType, Result, Twist, Wrench,
    spatial::{inertia::SpatialInertia, transform::force_to_parent},
};

use super::super::topology::{child_link_index, incoming_joint_index};
use super::super::{
    BaseKind, FLOATING_BASE_DOF, Fixed, Floating, FloatingRobot, Model, Robot, Workspace,
    generalized_count,
};
use super::{wrench_component, write_wrench_to_column};

impl Robot {
    /// Writes the `G x G` mass matrix in column-major order.
    ///
    /// Rows and columns follow the generalized-vector ordering documented by
    /// [`Robot::generalized_count`].
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input, an invalid output length, or a non-finite numerical result.
    pub fn mass_matrix(&mut self, q: &[f64], output: &mut [f64]) -> Result<()> {
        self.model
            .mass_matrix::<Fixed>(&self.world_from_root, q, &mut self.workspace, output)
    }
}

impl FloatingRobot {
    /// Writes the symmetric `G x G` mass matrix in column-major order.
    ///
    /// Rows and columns prepend world-frame angular then linear base components
    /// before the `J` non-fixed joints (`G = J + 6`). `output` must contain
    /// `G * G` elements.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input, an invalid output length, or a non-finite numerical result.
    pub fn mass_matrix(&mut self, base: &BaseState, q: &[f64], output: &mut [f64]) -> Result<()> {
        self.model
            .mass_matrix::<Floating>(base.frame(), q, &mut self.workspace, output)
    }
}

impl Model {
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
    pub(super) fn mass_matrix<B: BaseKind>(
        &self,
        base_frame: &Frame,
        q: &[f64],
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        let g = generalized_count::<B>(self);
        self.validate_slice("q", q)?;
        self.validate_slice_length("mass matrix output", output.len(), g * g)?;
        self.populate_parent_from_child(q, &mut workspace.frames)?;
        self.mass_matrix_from_transforms::<B>(
            base_frame,
            &workspace.frames,
            &mut workspace.composite_inertias,
            output,
        )
    }

    pub(in crate::robot) fn mass_matrix_from_transforms<B: BaseKind>(
        &self,
        base_frame: &Frame,
        parent_from_child: &[Frame],
        composite_inertias: &mut [SpatialInertia],
        output: &mut [f64],
    ) -> Result<()> {
        let g = generalized_count::<B>(self);
        self.validate_slice_length("mass matrix output", output.len(), g * g)?;
        self.mass_matrix_kernel::<B>(base_frame, parent_from_child, composite_inertias, output);
        Self::validate_numerical_result("mass matrix", output.iter())
    }

    #[allow(
        clippy::needless_range_loop,
        reason = "joint indices address parallel model and workspace arrays in tree order"
    )]
    fn mass_matrix_kernel<B: BaseKind>(
        &self,
        base_frame: &Frame,
        parent_from_child: &[Frame],
        composite_inertias: &mut [SpatialInertia],
        output: &mut [f64],
    ) {
        let model_joint_count = self.model_joint_count();
        let g = generalized_count::<B>(self);
        output.fill(0.0);

        let mut root_inertia = self.link_dynamics[0].spatial_inertia;
        for joint_index in 0..model_joint_count {
            composite_inertias[joint_index] =
                self.link_dynamics[child_link_index(joint_index)].spatial_inertia;
        }
        // Composite rigid-body pass: accumulate each subtree inertia, expressed
        // about the parent link origin, into the parent.
        for joint_index in (0..model_joint_count).rev() {
            let parent_link_index = self.parent_link_indices[joint_index];
            if parent_link_index == 0 {
                if B::IS_FLOATING {
                    root_inertia += composite_inertias[joint_index]
                        .transform_to_parent(&parent_from_child[joint_index]);
                }
                continue;
            }
            let transformed = composite_inertias[joint_index]
                .transform_to_parent(&parent_from_child[joint_index]);
            let parent_joint_index = incoming_joint_index(parent_link_index);
            composite_inertias[parent_joint_index] += transformed;
        }

        let base_rotation = base_frame.rotation;
        if B::IS_FLOATING {
            for column in 0..FLOATING_BASE_DOF {
                let world_axis = Vector3::ith(column % 3, 1.0);
                let local_axis = base_rotation.inverse() * world_axis;
                let motion = if column < 3 {
                    Twist::new(local_axis, Vector3::zeros())
                } else {
                    Twist::new(Vector3::zeros(), local_axis)
                };
                let local_load = root_inertia.apply(motion);
                let world_load = Wrench::new(
                    base_rotation * local_load.torque,
                    base_rotation * local_load.force,
                );
                write_wrench_to_column(output, g, column, world_load);
            }
        }

        // Mass-matrix entries: F = I^c S in the child link frame, then F is
        // propagated up the ancestor chain while M(i, j) = S_j^T F.
        for &joint_index in self.active_joint_indices.iter() {
            let dof_index =
                self.joint_dof_indices[joint_index].expect("active joint has a DOF index");
            let joint = self.joint_kinematics[joint_index];
            let mut force =
                composite_inertias[joint_index].apply_joint(joint.joint_type, *joint.axis.as_ref());
            let joint_column = B::DOF + dof_index;
            let mut current_joint_index = joint_index;
            loop {
                let current_joint = self.joint_kinematics[current_joint_index];
                let current_axis: Vector3<f64> = *current_joint.axis.as_ref();
                let entry = match current_joint.joint_type {
                    JointType::Revolute => current_axis.dot(&force.torque),
                    JointType::Prismatic => current_axis.dot(&force.force),
                    JointType::Fixed => 0.0,
                };
                if let Some(current_dof) = self.joint_dof_indices[current_joint_index] {
                    let current_row = B::DOF + current_dof;
                    output[joint_column * g + current_row] = entry;
                    output[current_row * g + joint_column] = entry;
                }
                let parent_link_index = self.parent_link_indices[current_joint_index];
                if parent_link_index == 0 {
                    if B::IS_FLOATING {
                        force = force_to_parent(&parent_from_child[current_joint_index], force);
                    }
                    break;
                }
                force = force_to_parent(&parent_from_child[current_joint_index], force);
                current_joint_index = incoming_joint_index(parent_link_index);
            }
            if B::IS_FLOATING {
                let world_force =
                    Wrench::new(base_rotation * force.torque, base_rotation * force.force);
                for base_row in 0..FLOATING_BASE_DOF {
                    let entry = wrench_component(world_force, base_row);
                    output[joint_column * g + base_row] = entry;
                    output[base_row * g + joint_column] = entry;
                }
            }
        }
    }
}
