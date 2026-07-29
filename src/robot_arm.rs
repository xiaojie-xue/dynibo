use std::path::Path;

use nalgebra::{DVector, Isometry3, Translation3, UnitQuaternion, Vector3};

use crate::{
    Error, Frame, Jacobian, JointKind, JointVector, Motion, Result, RobotLink, Wrench,
    urdf::serial_links,
};

pub const GRAVITY: f64 = 9.80665;

/// Runtime-sized serial robot model with fixed-size calculation inputs and outputs.
#[derive(Clone, Debug)]
pub struct RobotArm {
    name: String,
    links: Box<[RobotLink]>,
    home_offset: Box<[f64]>,
    home_end_frame: Frame,
}

impl RobotArm {
    pub fn from_links<const N: usize>(name: impl Into<String>, links: [RobotLink; N]) -> Self {
        Self::from_link_vec(name.into(), Vec::from(links))
    }

    fn from_link_vec(name: String, links: Vec<RobotLink>) -> Self {
        let home_end_frame = links
            .iter()
            .fold(Frame::identity(), |frame, link| frame * link.frame(0.0));
        let home_offset = vec![0.0; links.len()].into_boxed_slice();
        Self {
            name,
            links: links.into_boxed_slice(),
            home_offset,
            home_end_frame,
        }
    }

    pub fn from_urdf_str(source: &str) -> Result<Self> {
        let robot = urdf_rs::read_from_string(source)?;
        let links = serial_links(&robot)?;
        Ok(Self::from_link_vec(robot.name, links))
    }

    pub fn from_urdf_file(path: impl AsRef<Path>) -> Result<Self> {
        let robot = urdf_rs::read_file(path)?;
        let links = serial_links(&robot)?;
        Ok(Self::from_link_vec(robot.name, links))
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn links(&self) -> &[RobotLink] {
        &self.links
    }

    pub fn link_mut(&mut self, index: usize) -> Option<&mut RobotLink> {
        self.links.get_mut(index)
    }

    pub fn replace_link(&mut self, index: usize, link: RobotLink) -> Option<RobotLink> {
        if index >= self.links.len() {
            return None;
        }
        let old = std::mem::replace(&mut self.links[index], link);
        self.update_home_end_frame();
        Some(old)
    }

    pub fn home_offset(&self) -> &[f64] {
        &self.home_offset
    }

    pub fn home_end_frame(&self) -> Frame {
        self.home_end_frame
    }

    pub fn joint_count(&self) -> usize {
        self.links.len()
    }

    pub fn validate_joint_count<const N: usize>(&self) -> Result<()> {
        if self.links.len() == N {
            Ok(())
        } else {
            Err(Error::WrongJointCount {
                expected: self.links.len(),
                actual: N,
            })
        }
    }

    pub fn movable_joint_count(&self) -> usize {
        self.links
            .iter()
            .filter(|link| link.kind() != JointKind::Fixed)
            .count()
    }

    pub fn forward_kinematics<const N: usize>(&self, q: &JointVector<N>) -> Result<Frame> {
        self.validate_joint_count::<N>()?;
        Ok(self
            .links
            .iter()
            .zip(q.iter())
            .fold(Frame::identity(), |frame, (link, &position)| {
                frame * link.frame(position)
            }))
    }

    /// Computes the end frame and base-frame geometric Jacobian in one chain traversal.
    pub fn forward_kinematics_and_jacobian<const N: usize>(
        &self,
        q: &JointVector<N>,
    ) -> Result<(Frame, Jacobian<N>)> {
        self.validate_joint_count::<N>()?;
        let mut transform = Frame::identity();
        let mut origins: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut jacobian: Jacobian<N> = Jacobian::zeros();

        for i in 0..N {
            transform *= self.links[i].frame(q[i]);
            let axis = transform.rotation * self.links[i].axis().as_ref();

            match self.links[i].kind() {
                JointKind::Revolute => {
                    origins[i] = transform.translation.vector;
                    jacobian.fixed_view_mut::<3, 1>(0, i).copy_from(&axis);
                }
                JointKind::Prismatic => {
                    jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&axis);
                }
                JointKind::Fixed => {}
            }
        }

        let end_origin = transform.translation.vector;
        for (i, origin) in origins.iter().enumerate() {
            if self.links[i].kind() == JointKind::Revolute {
                let axis = jacobian.fixed_view::<3, 1>(0, i).into_owned();
                let linear = axis.cross(&(end_origin - origin));
                jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&linear);
            }
        }

        Ok((transform, jacobian))
    }

    /// Geometric Jacobian in the base frame, angular rows first.
    pub fn jacobian<const N: usize>(&self, q: &JointVector<N>) -> Result<Jacobian<N>> {
        Ok(self.forward_kinematics_and_jacobian(q)?.1)
    }

    pub fn jacobian_with_tool<const N: usize>(
        &self,
        q: &JointVector<N>,
        tool: &Frame,
    ) -> Result<Jacobian<N>> {
        let (end, mut jacobian) = self.forward_kinematics_and_jacobian(q)?;
        let offset_world = end.rotation * tool.translation.vector;
        for i in 0..N {
            let angular = jacobian.fixed_view::<3, 1>(0, i).into_owned();
            let shifted =
                jacobian.fixed_view::<3, 1>(3, i).into_owned() + angular.cross(&offset_world);
            jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&shifted);
        }
        Ok(jacobian)
    }

    pub fn jacobian_with_base<const N: usize>(
        &self,
        q: &JointVector<N>,
        base: &Frame,
    ) -> Result<Jacobian<N>> {
        let mut jacobian = self.jacobian(q)?;
        for i in 0..N {
            let angular = base.rotation * jacobian.fixed_view::<3, 1>(0, i).into_owned();
            let linear = base.rotation * jacobian.fixed_view::<3, 1>(3, i).into_owned();
            jacobian.fixed_view_mut::<3, 1>(0, i).copy_from(&angular);
            jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&linear);
        }
        Ok(jacobian)
    }

    pub fn forward_velocity_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        base: &Frame,
        tool: &Frame,
    ) -> Result<Motion> {
        let vector = self.jacobian_with_tool(q, tool)? * qd;
        Ok(Motion::new(
            base.rotation * vector.fixed_rows::<3>(0).into_owned(),
            base.rotation * vector.fixed_rows::<3>(3).into_owned(),
        ))
    }

    /// Time derivative of the base-frame geometric Jacobian, angular rows first.
    pub fn jacobian_dot<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
    ) -> Result<Jacobian<N>> {
        self.validate_joint_count::<N>()?;
        let mut transform = Frame::identity();
        let mut angular_velocity = Vector3::zeros();
        let mut origin_velocity = Vector3::zeros();
        let mut origins: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut origin_velocities: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut axes: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        let mut axis_derivatives: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());

        for i in 0..N {
            let parent_origin = transform.translation.vector;
            let current = transform * self.links[i].frame(q[i]);
            let offset = current.translation.vector - parent_origin;
            let axis = current.rotation * self.links[i].axis().as_ref();

            origin_velocity += angular_velocity.cross(&offset);
            match self.links[i].kind() {
                JointKind::Revolute => angular_velocity += axis * qd[i],
                JointKind::Prismatic => origin_velocity += axis * qd[i],
                JointKind::Fixed => {}
            }

            origins[i] = current.translation.vector;
            origin_velocities[i] = origin_velocity;
            axes[i] = axis;
            axis_derivatives[i] = angular_velocity.cross(&axis);
            transform = current;
        }

        let end_origin = transform.translation.vector;
        let end_velocity = origin_velocity;
        let mut jacobian_dot: Jacobian<N> = Jacobian::zeros();
        for i in 0..N {
            match self.links[i].kind() {
                JointKind::Revolute => {
                    jacobian_dot
                        .fixed_view_mut::<3, 1>(0, i)
                        .copy_from(&axis_derivatives[i]);
                    let linear = axis_derivatives[i].cross(&(end_origin - origins[i]))
                        + axes[i].cross(&(end_velocity - origin_velocities[i]));
                    jacobian_dot.fixed_view_mut::<3, 1>(3, i).copy_from(&linear);
                }
                JointKind::Prismatic => {
                    jacobian_dot
                        .fixed_view_mut::<3, 1>(3, i)
                        .copy_from(&axis_derivatives[i]);
                }
                JointKind::Fixed => {}
            }
        }
        Ok(jacobian_dot)
    }

    /// Convective end-effector acceleration `J_dot(q, qd) * qd`.
    pub fn jacobian_dot_times_velocity<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
    ) -> Result<Motion> {
        self.validate_joint_count::<N>()?;
        Ok(self.end_acceleration(q, qd, |_| 0.0))
    }

    pub fn forward_acceleration_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
    ) -> Result<Motion> {
        self.validate_joint_count::<N>()?;
        Ok(self.end_acceleration(q, qd, |i| qdd[i]))
    }

    /// Newton-Euler inverse dynamics compatible with the original C++ code.
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
        end_load: Wrench,
    ) -> Result<(JointVector<N>, Wrench)> {
        self.validate_joint_count::<N>()?;
        let mut transforms: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        let mut angular_accelerations: [Vector3<f64>; N] =
            std::array::from_fn(|_| Vector3::zeros());
        let mut link_accelerations: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());

        let base_rotation_inverse = base_frame.rotation.inverse();
        let mut omega = base_rotation_inverse * base_velocity.angular;
        let mut angular_acceleration = base_rotation_inverse * base_acceleration.angular;
        let mut acceleration =
            base_rotation_inverse * (Vector3::new(0.0, 0.0, GRAVITY) + base_acceleration.linear);

        for i in 0..N {
            let link = &self.links[i];
            let transform = link.frame(q[i]);
            let rotation_inverse = transform.rotation.inverse();
            let translation = transform.translation.vector;
            let axis = link.axis().as_ref();
            let rotated_omega = rotation_inverse * omega;
            let rotated_angular_acceleration = rotation_inverse * angular_acceleration;
            let translated_acceleration = rotation_inverse
                * (acceleration
                    + angular_acceleration.cross(&translation)
                    + omega.cross(&omega.cross(&translation)));
            match link.kind() {
                JointKind::Revolute => {
                    acceleration = translated_acceleration;
                    angular_acceleration = rotated_angular_acceleration
                        + qdd[i] * axis
                        + rotated_omega.cross(&(qd[i] * axis));
                    omega = rotated_omega + qd[i] * axis;
                }
                JointKind::Prismatic => {
                    acceleration = translated_acceleration
                        + qdd[i] * axis
                        + 2.0 * qd[i] * omega.cross(&(transform.rotation * axis));
                    angular_acceleration = rotated_angular_acceleration;
                    omega = rotated_omega;
                }
                JointKind::Fixed => {
                    acceleration = translated_acceleration;
                    angular_acceleration = rotated_angular_acceleration;
                    omega = rotated_omega;
                }
            }
            angular_accelerations[i] = angular_acceleration;
            let center = link.center_of_mass();
            link_accelerations[i] = acceleration
                + angular_acceleration.cross(center)
                + omega.cross(&omega.cross(center));
            transforms[i] = transform;
        }

        let mut joint_force = JointVector::zeros();
        let mut load = end_load;
        for i in (0..N).rev() {
            let link = &self.links[i];
            let (next_rotation, next_translation) = if i + 1 < N {
                (
                    transforms[i + 1].rotation,
                    transforms[i + 1].translation.vector,
                )
            } else {
                (UnitQuaternion::identity(), Vector3::zeros())
            };
            let child_force = next_rotation * load.force;
            let force = child_force + link.mass() * link_accelerations[i];
            let torque = next_rotation * load.torque
                + next_translation.cross(&child_force)
                + link
                    .center_of_mass()
                    .cross(&(link.mass() * link_accelerations[i]))
                + link.inertia() * angular_accelerations[i];
            load = Wrench::new(torque, force);
            joint_force[i] = link.active_force(load);
        }
        Ok((joint_force, wrench_to_parent(&transforms[0], load)))
    }

    /// Gravity joint forces and the resulting base wrench.
    pub fn gravity_torque<const N: usize>(
        &self,
        q: &JointVector<N>,
        base_frame: &Frame,
        end_load: Wrench,
    ) -> Result<(JointVector<N>, Wrench)> {
        self.validate_joint_count::<N>()?;
        let mut gravity = base_frame.rotation.inverse() * Vector3::new(0.0, 0.0, GRAVITY);
        let mut transforms: [Frame; N] = std::array::from_fn(|_| Frame::identity());
        let mut gravity_at_link: [Vector3<f64>; N] = std::array::from_fn(|_| Vector3::zeros());
        for i in 0..N {
            transforms[i] = self.links[i].frame(q[i]);
            gravity = transforms[i].rotation.inverse() * gravity;
            gravity_at_link[i] = gravity;
        }

        let mut torque = JointVector::zeros();
        let mut load = end_load;
        for i in (0..N).rev() {
            let force = self.links[i].mass() * gravity_at_link[i];
            let gravity_load = Wrench::new(self.links[i].center_of_mass().cross(&force), force);
            if i + 1 < N {
                load = add_wrench(wrench_to_parent(&transforms[i + 1], load), gravity_load);
            } else {
                load = add_wrench(load, gravity_load);
            }
            torque[i] = self.links[i].active_force(load);
        }
        Ok((torque, wrench_to_parent(&transforms[0], load)))
    }

    pub fn joint_position_limits(&self) -> (DVector<f64>, DVector<f64>) {
        let lower = DVector::from_iterator(
            self.links.len(),
            self.links.iter().map(|link| link.limit().lower),
        );
        let upper = DVector::from_iterator(
            self.links.len(),
            self.links.iter().map(|link| link.limit().upper),
        );
        (lower, upper)
    }

    pub fn saturate_joint_position<const N: usize>(
        lower: &JointVector<N>,
        upper: &JointVector<N>,
        position: &JointVector<N>,
    ) -> JointVector<N> {
        JointVector::from_fn(|i, _| position[i].clamp(lower[i], upper[i]))
    }

    fn update_home_end_frame(&mut self) {
        self.home_end_frame = self
            .links
            .iter()
            .fold(Frame::identity(), |frame, link| frame * link.frame(0.0));
    }

    fn end_acceleration<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        mut joint_acceleration_at: impl FnMut(usize) -> f64,
    ) -> Motion {
        let mut transform = Frame::identity();
        let mut angular_velocity = Vector3::zeros();
        let mut angular_acceleration = Vector3::zeros();
        let mut linear_acceleration = Vector3::zeros();

        for i in 0..N {
            let parent_origin = transform.translation.vector;
            transform *= self.links[i].frame(q[i]);
            let offset = transform.translation.vector - parent_origin;
            let axis = transform.rotation * self.links[i].axis().as_ref();
            let joint_acceleration = joint_acceleration_at(i);

            linear_acceleration += angular_acceleration.cross(&offset)
                + angular_velocity.cross(&angular_velocity.cross(&offset));

            match self.links[i].kind() {
                JointKind::Revolute => {
                    angular_acceleration +=
                        axis * joint_acceleration + angular_velocity.cross(&axis) * qd[i];
                    angular_velocity += axis * qd[i];
                }
                JointKind::Prismatic => {
                    linear_acceleration +=
                        axis * joint_acceleration + 2.0 * qd[i] * angular_velocity.cross(&axis);
                }
                JointKind::Fixed => {}
            }
        }

        Motion::new(angular_acceleration, linear_acceleration)
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

#[allow(dead_code)]
fn translation_frame(translation: Vector3<f64>) -> Frame {
    Isometry3::from_parts(Translation3::from(translation), UnitQuaternion::identity())
}
