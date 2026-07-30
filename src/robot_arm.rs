use std::path::Path;

use nalgebra::Vector3;

use crate::{
    Error, Frame, Jacobian, JointKind, JointVector, Motion, Result, RobotJoint, RobotLink, Wrench,
    urdf::tree_model,
};

const GRAVITY: f64 = 9.80665;

/// Stable numeric identifier for a link in one loaded robot model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LinkId(usize);

impl LinkId {
    pub const fn from_index(index: usize) -> Self {
        Self(index)
    }

    pub const fn index(self) -> usize {
        self.0
    }
}

/// Wrench applied at a link origin and expressed in that link's frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExternalWrench {
    pub link: LinkId,
    pub wrench: Wrench,
}

/// Runtime-sized serial robot model with fixed-size calculation inputs and outputs.
#[derive(Clone, Debug)]
pub struct RobotArm {
    name: String,
    joints: Box<[RobotJoint]>,
    links: Box<[RobotLink]>,
    joint_parents: Box<[LinkId]>,
    leaf_links: Box<[LinkId]>,
}

impl RobotArm {
    pub fn from_urdf(path: impl AsRef<Path>) -> Result<Self> {
        let robot = urdf_rs::read_file(path)?;
        let model = tree_model(&robot)?;
        Ok(Self {
            name: robot.name,
            joints: model.joints.into_boxed_slice(),
            links: model.links.into_boxed_slice(),
            joint_parents: model
                .joint_parents
                .into_iter()
                .map(LinkId)
                .collect::<Vec<_>>()
                .into_boxed_slice(),
            leaf_links: model
                .leaf_links
                .into_iter()
                .map(LinkId)
                .collect::<Vec<_>>()
                .into_boxed_slice(),
        })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn links(&self) -> &[RobotLink] {
        &self.links
    }

    pub fn joints(&self) -> &[RobotJoint] {
        &self.joints
    }

    pub const fn root_link(&self) -> LinkId {
        LinkId(0)
    }

    pub fn leaf_links(&self) -> &[LinkId] {
        &self.leaf_links
    }

    pub fn link_id(&self, name: &str) -> Option<LinkId> {
        self.links
            .iter()
            .position(|link| link.name() == name)
            .map(LinkId)
    }

    pub fn link_count(&self) -> usize {
        self.links.len()
    }

    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    fn validate_joint_count<const N: usize>(&self) -> Result<()> {
        if self.joints.len() == N {
            Ok(())
        } else {
            Err(Error::WrongJointCount {
                expected: self.joints.len(),
                actual: N,
            })
        }
    }

    fn validate_link(&self, link: LinkId) -> Result<()> {
        if link.0 < self.links.len() {
            Ok(())
        } else {
            Err(Error::InvalidLinkId {
                index: link.0,
                link_count: self.links.len(),
            })
        }
    }

    pub fn forward_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: LinkId,
    ) -> Result<Frame> {
        self.validate_joint_count::<N>()?;
        self.validate_link(target)?;
        if target == self.root_link() {
            return Ok(Frame::identity());
        }
        Ok(self.link_frames(q)[target.0 - 1])
    }

    fn link_frames<const N: usize>(&self, q: &JointVector<N>) -> [Frame; N] {
        let mut frames: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        for i in 0..N {
            let parent = self.joint_parents[i].0;
            let parent_frame = if parent == 0 {
                Frame::identity()
            } else {
                frames[parent - 1]
            };
            frames[i] = parent_frame * self.joints[i].frame(q[i]);
        }
        frames
    }

    /// Computes one link frame and its base-frame geometric Jacobian.
    fn forward_kinematics_and_jacobian<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: LinkId,
    ) -> Result<(Frame, Jacobian<N>)> {
        self.validate_joint_count::<N>()?;
        self.validate_link(target)?;
        let frames = self.link_frames(q);
        let target_frame = if target == self.root_link() {
            Frame::identity()
        } else {
            frames[target.0 - 1]
        };
        let mut jacobian: Jacobian<N> = Jacobian::zeros();

        let mut current = target.0;
        while current != 0 {
            let joint_index = current - 1;
            let joint_frame = frames[joint_index];
            let axis = joint_frame.rotation * self.joints[joint_index].axis().as_ref();
            match self.joints[joint_index].kind() {
                JointKind::Revolute => {
                    let linear = axis
                        .cross(&(target_frame.translation.vector - joint_frame.translation.vector));
                    jacobian
                        .fixed_view_mut::<3, 1>(0, joint_index)
                        .copy_from(&axis);
                    jacobian
                        .fixed_view_mut::<3, 1>(3, joint_index)
                        .copy_from(&linear);
                }
                JointKind::Prismatic => {
                    jacobian
                        .fixed_view_mut::<3, 1>(3, joint_index)
                        .copy_from(&axis);
                }
                JointKind::Fixed => {}
            }
            current = self.joint_parents[joint_index].0;
        }

        Ok((target_frame, jacobian))
    }

    /// Geometric Jacobian in the base frame, angular rows first.
    pub fn jacobian<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: LinkId,
    ) -> Result<Jacobian<N>> {
        Ok(self.forward_kinematics_and_jacobian(q, target)?.1)
    }

    pub fn forward_velocity_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        target: LinkId,
        base: &Frame,
        tool: &Frame,
    ) -> Result<Motion> {
        let (end, mut jacobian) = self.forward_kinematics_and_jacobian(q, target)?;
        let offset_world = end.rotation * tool.translation.vector;
        for i in 0..N {
            let angular = jacobian.fixed_view::<3, 1>(0, i).into_owned();
            let shifted =
                jacobian.fixed_view::<3, 1>(3, i).into_owned() + angular.cross(&offset_world);
            jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&shifted);
        }
        let vector = jacobian * qd;
        Ok(Motion::new(
            base.rotation * vector.fixed_rows::<3>(0).into_owned(),
            base.rotation * vector.fixed_rows::<3>(3).into_owned(),
        ))
    }

    pub fn forward_acceleration_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        target: LinkId,
    ) -> Result<Motion> {
        self.validate_joint_count::<N>()?;
        self.validate_link(target)?;
        Ok(self.link_acceleration(q, qd, qdd, target))
    }

    /// Newton-Euler inverse dynamics.
    /// Returns `(joint_force, wrench_at_base)`.
    #[allow(clippy::too_many_arguments)]
    pub fn inverse_dynamics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        base_frame: &Frame,
        base_velocity: Motion,
        base_acceleration: Motion,
        external_wrenches: &[ExternalWrench],
    ) -> Result<(JointVector<N>, Wrench)> {
        self.validate_joint_count::<N>()?;
        let mut transforms: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        let mut angular_velocities: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut angular_accelerations: [Vector3<f64>; N] =
            std::array::from_fn(|_| Vector3::zeros());
        let mut origin_accelerations: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut link_accelerations: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());

        let base_rotation_inverse = base_frame.rotation.inverse();
        let base_omega = base_rotation_inverse * base_velocity.angular;
        let base_angular_acceleration = base_rotation_inverse * base_acceleration.angular;
        let base_acceleration =
            base_rotation_inverse * (Vector3::new(0.0, 0.0, GRAVITY) + base_acceleration.linear);

        for i in 0..N {
            let joint = &self.joints[i];
            let link = &self.links[i + 1];
            let parent = self.joint_parents[i].0;
            let (parent_omega, parent_angular_acceleration, parent_acceleration) = if parent == 0 {
                (base_omega, base_angular_acceleration, base_acceleration)
            } else {
                (
                    angular_velocities[parent - 1],
                    angular_accelerations[parent - 1],
                    origin_accelerations[parent - 1],
                )
            };
            let transform = joint.frame(q[i]);
            let rotation_inverse = transform.rotation.inverse();
            let translation = transform.translation.vector;
            let axis = joint.axis().as_ref();
            let rotated_omega = rotation_inverse * parent_omega;
            let rotated_angular_acceleration = rotation_inverse * parent_angular_acceleration;
            let translated_acceleration = rotation_inverse
                * (parent_acceleration
                    + parent_angular_acceleration.cross(&translation)
                    + parent_omega.cross(&parent_omega.cross(&translation)));
            let (omega, angular_acceleration, acceleration) = match joint.kind() {
                JointKind::Revolute => {
                    let angular_acceleration = rotated_angular_acceleration
                        + qdd[i] * axis
                        + rotated_omega.cross(&(qd[i] * axis));
                    (
                        rotated_omega + qd[i] * axis,
                        angular_acceleration,
                        translated_acceleration,
                    )
                }
                JointKind::Prismatic => (
                    rotated_omega,
                    rotated_angular_acceleration,
                    translated_acceleration
                        + qdd[i] * axis
                        + 2.0 * qd[i] * parent_omega.cross(&(transform.rotation * axis)),
                ),
                JointKind::Fixed => (
                    rotated_omega,
                    rotated_angular_acceleration,
                    translated_acceleration,
                ),
            };
            angular_velocities[i] = omega;
            angular_accelerations[i] = angular_acceleration;
            origin_accelerations[i] = acceleration;
            let center = link.center_of_mass();
            link_accelerations[i] = acceleration
                + angular_acceleration.cross(center)
                + omega.cross(&omega.cross(center));
            transforms[i] = transform;
        }

        let mut joint_force = JointVector::zeros();
        let mut loads: [Wrench; N] = std::array::from_fn(|_| Wrench::zeros());
        let mut base_load = Wrench::zeros();
        for external in external_wrenches {
            self.validate_link(external.link)?;
            if external.link == self.root_link() {
                base_load = add_wrench(base_load, external.wrench);
            } else {
                let index = external.link.0 - 1;
                loads[index] = add_wrench(loads[index], external.wrench);
            }
        }

        for i in (0..N).rev() {
            let joint = &self.joints[i];
            let link = &self.links[i + 1];
            let inertial_force = link.mass() * link_accelerations[i];
            let angular_momentum = link.inertia() * angular_velocities[i];
            let inertial_load = Wrench::new(
                link.center_of_mass().cross(&inertial_force)
                    + link.inertia() * angular_accelerations[i]
                    + angular_velocities[i].cross(&angular_momentum),
                inertial_force,
            );
            loads[i] = add_wrench(loads[i], inertial_load);
            joint_force[i] = joint.active_force(loads[i]);

            let parent_load = wrench_to_parent(&transforms[i], loads[i]);
            let parent = self.joint_parents[i].0;
            if parent == 0 {
                base_load = add_wrench(base_load, parent_load);
            } else {
                loads[parent - 1] = add_wrench(loads[parent - 1], parent_load);
            }
        }
        Ok((joint_force, base_load))
    }

    /// Gravity joint forces and the resulting base wrench.
    pub fn gravity<const N: usize>(
        &self,
        q: &JointVector<N>,
        base_frame: &Frame,
        external_wrenches: &[ExternalWrench],
    ) -> Result<(JointVector<N>, Wrench)> {
        self.validate_joint_count::<N>()?;
        let base_gravity = base_frame.rotation.inverse() * Vector3::new(0.0, 0.0, GRAVITY);
        let mut transforms: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        let mut gravity_at_link: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        for i in 0..N {
            transforms[i] = self.joints[i].frame(q[i]);
            let parent = self.joint_parents[i].0;
            let parent_gravity = if parent == 0 {
                base_gravity
            } else {
                gravity_at_link[parent - 1]
            };
            gravity_at_link[i] = transforms[i].rotation.inverse() * parent_gravity;
        }

        let mut torque = JointVector::zeros();
        let mut loads: [Wrench; N] = std::array::from_fn(|_| Wrench::zeros());
        let mut base_load = Wrench::zeros();
        for external in external_wrenches {
            self.validate_link(external.link)?;
            if external.link == self.root_link() {
                base_load = add_wrench(base_load, external.wrench);
            } else {
                let index = external.link.0 - 1;
                loads[index] = add_wrench(loads[index], external.wrench);
            }
        }

        for i in (0..N).rev() {
            let joint = &self.joints[i];
            let link = &self.links[i + 1];
            let force = link.mass() * gravity_at_link[i];
            let gravity_load = Wrench::new(link.center_of_mass().cross(&force), force);
            loads[i] = add_wrench(loads[i], gravity_load);
            torque[i] = joint.active_force(loads[i]);

            let parent_load = wrench_to_parent(&transforms[i], loads[i]);
            let parent = self.joint_parents[i].0;
            if parent == 0 {
                base_load = add_wrench(base_load, parent_load);
            } else {
                loads[parent - 1] = add_wrench(loads[parent - 1], parent_load);
            }
        }
        Ok((torque, base_load))
    }

    fn link_acceleration<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        target: LinkId,
    ) -> Motion {
        if target == self.root_link() {
            return Motion::zeros();
        }
        let mut frames: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        let mut angular_velocities: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut angular_accelerations: [Vector3<f64>; N] =
            std::array::from_fn(|_| Vector3::zeros());
        let mut linear_accelerations: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());

        for i in 0..N {
            let parent = self.joint_parents[i].0;
            let (parent_frame, angular_velocity, angular_acceleration, linear_acceleration) =
                if parent == 0 {
                    (
                        Frame::identity(),
                        Vector3::zeros(),
                        Vector3::zeros(),
                        Vector3::zeros(),
                    )
                } else {
                    (
                        frames[parent - 1],
                        angular_velocities[parent - 1],
                        angular_accelerations[parent - 1],
                        linear_accelerations[parent - 1],
                    )
                };
            let frame = parent_frame * self.joints[i].frame(q[i]);
            let offset = frame.translation.vector - parent_frame.translation.vector;
            let axis = frame.rotation * self.joints[i].axis().as_ref();
            let mut child_angular_velocity = angular_velocity;
            let mut child_angular_acceleration = angular_acceleration;
            let mut child_linear_acceleration = linear_acceleration
                + angular_acceleration.cross(&offset)
                + angular_velocity.cross(&angular_velocity.cross(&offset));

            match self.joints[i].kind() {
                JointKind::Revolute => {
                    child_angular_acceleration +=
                        axis * qdd[i] + angular_velocity.cross(&axis) * qd[i];
                    child_angular_velocity += axis * qd[i];
                }
                JointKind::Prismatic => {
                    child_linear_acceleration +=
                        axis * qdd[i] + 2.0 * qd[i] * angular_velocity.cross(&axis);
                }
                JointKind::Fixed => {}
            }

            frames[i] = frame;
            angular_velocities[i] = child_angular_velocity;
            angular_accelerations[i] = child_angular_acceleration;
            linear_accelerations[i] = child_linear_acceleration;
        }

        let index = target.0 - 1;
        Motion::new(angular_accelerations[index], linear_accelerations[index])
    }
}

fn wrench_to_parent(transform: &Frame, wrench: Wrench) -> Wrench {
    let force = transform.rotation * wrench.force;
    Wrench::new(
        transform.rotation * wrench.torque + transform.translation.vector.cross(&force),
        force,
    )
}

fn add_wrench(lhs: Wrench, rhs: Wrench) -> Wrench {
    Wrench::new(lhs.torque + rhs.torque, lhs.force + rhs.force)
}
