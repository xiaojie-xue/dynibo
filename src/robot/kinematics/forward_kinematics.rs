use nalgebra::Vector3;

use crate::{Frame, JointType, Result, Twist};

use super::super::{LinkId, Robot, Workspace};

impl Robot {
    /// Computes a target link frame using runtime-sized input and workspace.
    ///
    /// For the joints on the root-to-target path, the returned world pose is
    ///
    /// $$
    /// {}^W T_{\mathrm{target}}(q) = {}^W T_{\mathrm{base}}
    /// \prod_{i \in \mathrm{path}} {}^{i-1}T_i(q_i).
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid input length, link ID, or workspace.
    pub fn forward_kinematics(
        &self,
        q: &[f64],
        target: LinkId,
        workspace: &mut Workspace,
    ) -> Result<Frame> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        let target_index = self.validate_link_id(target)?;
        let depth = self.prepare_ancestor_path(target_index, &mut workspace.ancestor_path);
        Ok(*self.base.frame() * self.target_frame_kernel(q, &workspace.ancestor_path[..depth])?)
    }

    /// Computes runtime-sized spatial velocity at a point on a target link.
    ///
    /// The Robot's base state supplies root motion; `tool` selects a point
    /// rigidly attached to the target link. The returned angular-first twist is
    /// expressed in the world frame at that selected point.
    ///
    /// $$
    /// V_{\mathrm{tool}} = J_{\mathrm{tool}}(q) \nu.
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input lengths, link ID, or workspace.
    pub fn forward_velocity_kinematics(
        &self,
        q: &[f64],
        qd: &[f64],
        target: LinkId,
        tool: &Frame,
        workspace: &mut Workspace,
    ) -> Result<Twist> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        let target_index = self.validate_link_id(target)?;
        Ok(self.forward_velocity_for_base(
            q,
            qd,
            target_index,
            tool,
            self.base.frame(),
            self.base.velocity(),
            &mut workspace.ancestor_path,
        ))
    }

    /// Computes world-expressed spatial acceleration of a target link origin.
    ///
    /// The returned angular-first acceleration is
    ///
    /// $$
    /// A_{\mathrm{target}} = J(q) \dot\nu + \dot J(q, \nu) \nu.
    /// $$
    ///
    /// # Errors
    ///
    /// Returns an error for invalid input lengths, link ID, or workspace.
    pub fn forward_acceleration_kinematics(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        target: LinkId,
        workspace: &mut Workspace,
    ) -> Result<Twist> {
        self.validate_workspace(workspace)?;
        self.validate_slice("q", q)?;
        self.validate_slice("qd", qd)?;
        self.validate_slice("qdd", qdd)?;
        let target_index = self.validate_link_id(target)?;
        let depth = self.prepare_ancestor_path(target_index, &mut workspace.ancestor_path);
        let (local_target, relative_velocity, relative_acceleration) = self.motion_for_joints(
            q,
            qd,
            qdd,
            workspace.ancestor_path[..depth].iter().rev().copied(),
        );
        let rotation = self.base.frame().rotation;
        let offset = rotation * local_target.translation.vector;
        let relative_angular = rotation * relative_velocity.angular;
        let relative_linear = rotation * relative_velocity.linear;
        let base_velocity = self.base.velocity();
        let base_acceleration = self.base.acceleration();
        Ok(Twist::new(
            base_acceleration.angular
                + base_velocity.angular.cross(&relative_angular)
                + rotation * relative_acceleration.angular,
            base_acceleration.linear
                + base_acceleration.angular.cross(&offset)
                + base_velocity
                    .angular
                    .cross(&base_velocity.angular.cross(&offset))
                + 2.0 * base_velocity.angular.cross(&relative_linear)
                + rotation * relative_acceleration.linear,
        ))
    }

    fn target_frame_kernel(&self, q: &[f64], path: &[usize]) -> Result<Frame> {
        self.validate_slice("q", q)?;
        let mut frame = Frame::identity();
        for &joint_index in path.iter().rev() {
            frame *= self.joint_kinematics[joint_index].frame(self.joint_value(q, joint_index));
        }
        Ok(frame)
    }

    #[inline]
    #[allow(clippy::too_many_arguments)]
    fn forward_velocity_for_base(
        &self,
        q: &[f64],
        qd: &[f64],
        target_index: usize,
        tool: &Frame,
        base_frame: &Frame,
        base_velocity: Twist,
        path: &mut [usize],
    ) -> Twist {
        let depth = self.prepare_ancestor_path(target_index, path);
        let (local_target, local_velocity) =
            self.velocity_for_joints(q, qd, path[..depth].iter().rev().copied(), tool);
        let offset = base_frame.rotation
            * (local_target.translation.vector + local_target.rotation * tool.translation.vector);
        let relative_angular = base_frame.rotation * local_velocity.angular;
        let relative_linear = base_frame.rotation * local_velocity.linear;
        Twist::new(
            base_velocity.angular + relative_angular,
            base_velocity.linear + base_velocity.angular.cross(&offset) + relative_linear,
        )
    }

    #[inline]
    fn velocity_for_joints(
        &self,
        q: &[f64],
        qd: &[f64],
        joint_indices: impl Iterator<Item = usize>,
        tool: &Frame,
    ) -> (Frame, Twist) {
        let mut frame = Frame::identity();
        let mut angular = Vector3::zeros();
        let mut linear = Vector3::zeros();
        for joint_index in joint_indices {
            let parent_position = frame.translation.vector;
            let joint = self.joint_kinematics[joint_index];
            frame *= joint.frame(self.joint_value(q, joint_index));
            linear += angular.cross(&(frame.translation.vector - parent_position));
            let axis = frame.rotation * joint.axis.as_ref();
            let velocity = self.joint_value(qd, joint_index);
            match joint.joint_type {
                JointType::Revolute => angular += axis * velocity,
                JointType::Prismatic => linear += axis * velocity,
                JointType::Fixed => {}
            }
        }
        linear += angular.cross(&(frame.rotation * tool.translation.vector));
        (frame, Twist::new(angular, linear))
    }

    #[inline]
    fn motion_for_joints(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        joint_indices: impl Iterator<Item = usize>,
    ) -> (Frame, Twist, Twist) {
        let mut frame = Frame::identity();
        let mut omega = Vector3::zeros();
        let mut velocity = Vector3::zeros();
        let mut alpha = Vector3::zeros();
        let mut acceleration = Vector3::zeros();
        for joint_index in joint_indices {
            let parent_position = frame.translation.vector;
            let joint = self.joint_kinematics[joint_index];
            frame *= joint.frame(self.joint_value(q, joint_index));
            let offset = frame.translation.vector - parent_position;
            let mut child_omega = omega;
            let mut child_velocity = velocity + omega.cross(&offset);
            let mut child_alpha = alpha;
            let mut child_acceleration =
                acceleration + alpha.cross(&offset) + omega.cross(&omega.cross(&offset));
            let axis = frame.rotation * joint.axis.as_ref();
            let joint_velocity = self.joint_value(qd, joint_index);
            let acceleration_value = self.joint_value(qdd, joint_index);
            match joint.joint_type {
                JointType::Revolute => {
                    child_alpha += axis * acceleration_value + omega.cross(&axis) * joint_velocity;
                    child_omega += axis * joint_velocity;
                }
                JointType::Prismatic => {
                    child_velocity += axis * joint_velocity;
                    child_acceleration +=
                        axis * acceleration_value + 2.0 * joint_velocity * omega.cross(&axis);
                }
                JointType::Fixed => {}
            }
            omega = child_omega;
            velocity = child_velocity;
            alpha = child_alpha;
            acceleration = child_acceleration;
        }
        (
            frame,
            Twist::new(omega, velocity),
            Twist::new(alpha, acceleration),
        )
    }
}
