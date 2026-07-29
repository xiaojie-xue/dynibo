use std::path::Path;

use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    Error, Frame, Jacobian, JointKind, JointVector, Motion, Result, RobotLink, Wrench,
    urdf::serial_links,
};

const GRAVITY: f64 = 9.80665;

/// Runtime-sized serial robot model with fixed-size calculation inputs and outputs.
#[derive(Clone, Debug)]
pub struct RobotArm {
    name: String,
    links: Box<[RobotLink]>,
}

impl RobotArm {
    pub fn from_urdf(path: impl AsRef<Path>) -> Result<Self> {
        let robot = urdf_rs::read_file(path)?;
        let links = serial_links(&robot)?;
        Ok(Self {
            name: robot.name,
            links: links.into_boxed_slice(),
        })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn links(&self) -> &[RobotLink] {
        &self.links
    }

    pub fn joint_count(&self) -> usize {
        self.links.len()
    }

    fn validate_joint_count<const N: usize>(&self) -> Result<()> {
        if self.links.len() == N {
            Ok(())
        } else {
            Err(Error::WrongJointCount {
                expected: self.links.len(),
                actual: N,
            })
        }
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
    fn forward_kinematics_and_jacobian<const N: usize>(
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

    pub fn forward_velocity_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        base: &Frame,
        tool: &Frame,
    ) -> Result<Motion> {
        let (end, mut jacobian) = self.forward_kinematics_and_jacobian(q)?;
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
    pub fn gravity<const N: usize>(
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
