use nalgebra::Vector3;

use crate::{BaseState, Frame, JointType, Result};

use super::super::topology::incoming_joint_index;
use super::super::{
    BaseKind, BaseMotion, Fixed, Floating, FloatingRobot, LinkId, Model, Robot, Workspace,
    generalized_count,
};

struct JacobianDerivativeScratch<'a> {
    root_from_link: &'a mut [Frame],
    angular_velocities: &'a mut [Vector3<f64>],
    origin_velocities: &'a mut [Vector3<f64>],
    jacobian: &'a mut [f64],
    jacobian_derivative: &'a mut [f64],
    ancestor_path: &'a mut [usize],
}

impl Robot {
    /// Writes a world-expressed `6 x G` geometric Jacobian in column-major order.
    ///
    /// Each column stores angular components followed by linear components at
    /// the target-link origin. `G` is [`Robot::generalized_count`].
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input, an invalid output length or link ID, or a non-finite result.
    pub fn jacobian(&mut self, q: &[f64], target: LinkId, output: &mut [f64]) -> Result<()> {
        self.model.jacobian::<Fixed>(
            &self.world_from_root,
            q,
            target,
            &mut self.workspace,
            output,
        )
    }

    /// Writes the time derivative of the geometric Jacobian in column-major order.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input, an invalid output length or link ID, or a non-finite result.
    pub fn jacobian_derivative(
        &mut self,
        q: &[f64],
        qd: &[f64],
        target: LinkId,
        output: &mut [f64],
    ) -> Result<()> {
        self.model.jacobian_derivative::<Fixed>(
            BaseMotion::stationary(&self.world_from_root),
            q,
            qd,
            target,
            &mut self.workspace,
            output,
        )
    }
}

impl FloatingRobot {
    /// Writes a world-expressed `6 x G` geometric Jacobian in column-major order.
    pub fn jacobian(
        &mut self,
        base: &BaseState,
        q: &[f64],
        target: LinkId,
        output: &mut [f64],
    ) -> Result<()> {
        self.model
            .jacobian::<Floating>(base.frame(), q, target, &mut self.workspace, output)
    }

    /// Writes the time derivative of the geometric Jacobian in column-major order.
    pub fn jacobian_derivative(
        &mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        target: LinkId,
        output: &mut [f64],
    ) -> Result<()> {
        self.model.jacobian_derivative::<Floating>(
            BaseMotion::from_state(base),
            q,
            qd,
            target,
            &mut self.workspace,
            output,
        )
    }
}

impl Model {
    /// Writes a runtime-sized `6 x G` geometric Jacobian in column-major order.
    ///
    /// $$
    /// {}^W V_{\mathrm{target}} = J(q) \nu, \qquad
    /// J(q) = \begin{bmatrix} J_\omega(q) \\ J_v(q) \end{bmatrix}.
    /// $$
    pub(super) fn jacobian<B: BaseKind>(
        &self,
        base_frame: &Frame,
        q: &[f64],
        target: LinkId,
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice_length(
            "jacobian output",
            output.len(),
            6 * generalized_count::<B>(self),
        )?;
        let target_index = self.validate_link_id(target)?;
        self.direct_world_jacobian::<B>(q, target_index, base_frame, workspace, output)
    }

    /// Writes the runtime-sized `6 x G` time derivative of the geometric
    /// Jacobian in column-major order.
    ///
    /// $$
    /// {}^W A_{\mathrm{target}} = J(q) \dot\nu + \dot J(q, \nu) \nu.
    /// $$
    pub(super) fn jacobian_derivative<B: BaseKind>(
        &self,
        base: BaseMotion<'_>,
        q: &[f64],
        qd: &[f64],
        target: LinkId,
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice_length(
            "jacobian derivative output",
            output.len(),
            6 * generalized_count::<B>(self),
        )?;
        let target_index = self.validate_link_id(target)?;
        let local_target = self.jacobian_derivative_kernel::<B>(
            q,
            qd,
            target_index,
            JacobianDerivativeScratch {
                root_from_link: &mut workspace.frames,
                angular_velocities: &mut workspace.angular_velocities,
                origin_velocities: &mut workspace.origin_velocities,
                jacobian: &mut workspace.jacobian,
                jacobian_derivative: &mut workspace.jacobian_derivative,
                ancestor_path: &mut workspace.ancestor_path,
            },
        )?;
        self.write_jacobian_derivative::<B>(
            base,
            qd,
            &local_target,
            &workspace.jacobian,
            &workspace.jacobian_derivative,
            output,
        );
        Self::validate_numerical_result("jacobian derivative", output.iter())
    }

    // Entry points have validated q, target and output dimensions. This path
    // writes world-oriented frames relative to the root origin into scratch;
    // IK/J-dot and other kernels rebuild frames before using root-local data.
    fn direct_world_jacobian<B: BaseKind>(
        &self,
        q: &[f64],
        target_index: usize,
        base_frame: &Frame,
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        let world_oriented_from_link = &mut workspace.frames;
        self.validate_slice_length(
            "frame workspace",
            world_oriented_from_link.len(),
            self.model_joint_count(),
        )?;
        self.validate_slice_length(
            "jacobian output",
            workspace.jacobian.len(),
            6 * self.joint_count(),
        )?;
        let depth = self.prepare_ancestor_path(target_index, &mut workspace.ancestor_path);
        let path = &workspace.ancestor_path[..depth];
        // World axes but positions relative to the base origin: large absolute
        // translations must not degrade the target-to-joint moment arms.
        let mut frame = Frame::identity();
        frame.rotation = base_frame.rotation;
        for &joint_index in path.iter().rev() {
            frame *= self.joint_kinematics[joint_index].frame(self.joint_value(q, joint_index));
            world_oriented_from_link[joint_index] = frame;
        }
        let target = frame_for_target(world_oriented_from_link, target_index);
        output.fill(0.0);
        if B::IS_FLOATING {
            for i in 0..3 {
                output[6 * i + i] = 1.0;
                let linear = Vector3::ith(i, 1.0).cross(&target.translation.vector);
                output[6 * i + 3..6 * i + 6].copy_from_slice(linear.as_slice());
                output[6 * (i + 3) + i + 3] = 1.0;
            }
        }
        for &joint_index in path {
            let Some(dof_index) = self.joint_dof_indices[joint_index] else {
                continue;
            };
            let joint = self.joint_kinematics[joint_index];
            let joint_frame = world_oriented_from_link[joint_index];
            let axis = joint_frame.rotation * joint.axis.as_ref();
            let column = &mut output[6 * (B::DOF + dof_index)..6 * (B::DOF + dof_index + 1)];
            match joint.joint_type {
                JointType::Revolute => {
                    column[..3].copy_from_slice(axis.as_slice());
                    column[3..].copy_from_slice(
                        axis.cross(&(target.translation.vector - joint_frame.translation.vector))
                            .as_slice(),
                    );
                }
                JointType::Prismatic => column[3..].copy_from_slice(axis.as_slice()),
                JointType::Fixed => unreachable!(),
            }
        }
        Self::validate_numerical_result("jacobian", output.iter())
    }

    pub(super) fn jacobian_kernel(
        &self,
        root_from_link: &[Frame],
        target_index: usize,
        path: &[usize],
        output: &mut [f64],
        clear_output: bool,
    ) -> Result<Frame> {
        self.validate_slice_length(
            "frame workspace",
            root_from_link.len(),
            self.model_joint_count(),
        )?;
        self.validate_slice_length("jacobian output", output.len(), 6 * self.joint_count())?;
        if clear_output {
            output.fill(0.0);
        }
        let target_frame = frame_for_target(root_from_link, target_index);
        for &joint_index in path {
            let Some(dof_index) = self.joint_dof_indices[joint_index] else {
                continue;
            };
            let joint_frame = root_from_link[joint_index];
            let column = &mut output[6 * dof_index..6 * dof_index + 6];
            let joint = self.joint_kinematics[joint_index];
            match joint.joint_type {
                JointType::Revolute => {
                    let axis = joint_frame.rotation * joint.axis.as_ref();
                    let linear = axis
                        .cross(&(target_frame.translation.vector - joint_frame.translation.vector));
                    column[..3].copy_from_slice(axis.as_slice());
                    column[3..].copy_from_slice(linear.as_slice());
                }
                JointType::Prismatic => {
                    let axis = joint_frame.rotation * joint.axis.as_ref();
                    column[3..].copy_from_slice(axis.as_slice());
                }
                JointType::Fixed => unreachable!("fixed joints have no DOF index"),
            }
        }
        Ok(target_frame)
    }

    fn jacobian_derivative_kernel<B: BaseKind>(
        &self,
        q: &[f64],
        qd: &[f64],
        target_index: usize,
        scratch: JacobianDerivativeScratch<'_>,
    ) -> Result<Frame> {
        let JacobianDerivativeScratch {
            root_from_link,
            angular_velocities,
            origin_velocities,
            jacobian,
            jacobian_derivative,
            ancestor_path,
        } = scratch;
        self.validate_slice_length("jacobian output", jacobian.len(), 6 * self.joint_count())?;
        jacobian_derivative.fill(0.0);
        if target_index == 0 {
            if B::IS_FLOATING {
                jacobian.fill(0.0);
            }
            return Ok(Frame::identity());
        }
        let depth = self.prepare_ancestor_path(target_index, ancestor_path);
        let path = &ancestor_path[..depth];
        self.target_frames_kernel(q, path, root_from_link)?;
        let mut angular = Vector3::zeros();
        let mut linear = Vector3::zeros();
        let mut parent_frame = Frame::identity();
        for &joint_index in path.iter().rev() {
            let joint = self.joint_kinematics[joint_index];
            let frame = root_from_link[joint_index];
            let offset = frame.translation.vector - parent_frame.translation.vector;
            let axis: Vector3<f64> = frame.rotation * joint.axis.as_ref();
            let mut child_angular = angular;
            let mut child_linear = linear + angular.cross(&offset);
            let velocity = self.joint_value(qd, joint_index);
            match joint.joint_type {
                JointType::Revolute => child_angular += axis * velocity,
                JointType::Prismatic => child_linear += axis * velocity,
                JointType::Fixed => {}
            }
            angular_velocities[joint_index] = child_angular;
            origin_velocities[joint_index] = child_linear;
            angular = child_angular;
            linear = child_linear;
            parent_frame = frame;
        }
        let target_frame = frame_for_target(root_from_link, target_index);
        let end_position = target_frame.translation.vector;
        let end_velocity = linear;
        for &joint_index in path {
            let Some(dof_index) = self.joint_dof_indices[joint_index] else {
                continue;
            };
            let joint = self.joint_kinematics[joint_index];
            let frame = root_from_link[joint_index];
            let axis: Vector3<f64> = frame.rotation * joint.axis.as_ref();
            let axis_rate = angular_velocities[joint_index].cross(&axis);
            let column = &mut jacobian_derivative[6 * dof_index..6 * dof_index + 6];
            match joint.joint_type {
                JointType::Revolute => {
                    let moment_arm = end_position - frame.translation.vector;
                    let origin_velocity = origin_velocities[joint_index];
                    let linear_rate = axis_rate.cross(&moment_arm)
                        + axis.cross(&(end_velocity - origin_velocity));
                    column[..3].copy_from_slice(axis_rate.as_slice());
                    column[3..].copy_from_slice(linear_rate.as_slice());
                }
                JointType::Prismatic => column[3..].copy_from_slice(axis_rate.as_slice()),
                JointType::Fixed => unreachable!("fixed joints have no DOF index"),
            }
        }
        if B::IS_FLOATING {
            self.jacobian_kernel(root_from_link, target_index, path, jacobian, true)
        } else {
            Ok(target_frame)
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn write_jacobian_derivative<B: BaseKind>(
        &self,
        base: BaseMotion<'_>,
        qd: &[f64],
        local_target: &Frame,
        joint_jacobian: &[f64],
        joint_derivative: &[f64],
        output: &mut [f64],
    ) {
        output.fill(0.0);
        let rotation = base.frame.rotation;
        let base_omega = base.velocity.angular;
        if B::IS_FLOATING {
            let mut local_velocity = Vector3::zeros();
            for (joint_index, &velocity) in qd.iter().enumerate() {
                let column = &joint_jacobian[6 * joint_index..6 * joint_index + 6];
                local_velocity += Vector3::from_column_slice(&column[3..]) * velocity;
            }
            let offset = rotation * local_target.translation.vector;
            let offset_rate = base_omega.cross(&offset) + rotation * local_velocity;
            for axis_index in 0..3 {
                let axis = Vector3::ith(axis_index, 1.0);
                output[6 * axis_index + 3..6 * axis_index + 6]
                    .copy_from_slice(axis.cross(&offset_rate).as_slice());
            }
        }
        for dof_index in 0..self.joint_count() {
            let derivative = &joint_derivative[6 * dof_index..6 * dof_index + 6];
            let mut angular = rotation * Vector3::from_column_slice(&derivative[..3]);
            let mut linear = rotation * Vector3::from_column_slice(&derivative[3..]);
            if B::IS_FLOATING {
                let source = &joint_jacobian[6 * dof_index..6 * dof_index + 6];
                let world_angular = rotation * Vector3::from_column_slice(&source[..3]);
                let world_linear = rotation * Vector3::from_column_slice(&source[3..]);
                angular += base_omega.cross(&world_angular);
                linear += base_omega.cross(&world_linear);
            }
            let column_index = B::DOF + dof_index;
            let column = &mut output[6 * column_index..6 * column_index + 6];
            column[..3].copy_from_slice(angular.as_slice());
            column[3..].copy_from_slice(linear.as_slice());
        }
    }
}

fn frame_for_target(frames: &[Frame], target_index: usize) -> Frame {
    if target_index == 0 {
        Frame::identity()
    } else {
        frames[incoming_joint_index(target_index)]
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

    #[test]
    fn jacobian_apis_propagate_corrupted_workspace_buffer_errors() {
        let mut robot = fixture();
        let target = robot.link_id("test_link_4").unwrap();
        let q = [0.0; 4];
        let mut output = [0.0; 24];

        robot.workspace.frames.pop();
        assert!(matches!(
            robot.jacobian(&q, target, &mut output),
            Err(Error::WrongSliceLength {
                slice: "frame workspace",
                ..
            })
        ));

        let mut robot = fixture();
        let target = robot.link_id("test_link_4").unwrap();
        robot.workspace.jacobian.pop();
        assert!(matches!(
            robot.jacobian_derivative(&q, &q, target, &mut output),
            Err(Error::WrongSliceLength { .. })
        ));
    }
}
