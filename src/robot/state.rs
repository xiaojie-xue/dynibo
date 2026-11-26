use std::marker::PhantomData;

use nalgebra::Vector3;

use crate::{BaseState, Error, Frame, JointType, Result, Twist};

use super::topology::{child_link_index, incoming_joint_index};
use super::{
    BaseKind, BaseMotion, Fixed, Floating, FloatingRobot, IndexedLoad, LinkId, Model, Robot,
    StateCache, Workspace, generalized_count,
};

impl StateCache {
    fn populate(&mut self, model: &Model, q: &[f64], qd: &[f64]) -> Result<()> {
        model.validate_slice("q", q)?;
        model.validate_slice("qd", qd)?;
        self.qd.copy_from_slice(qd);
        for joint_index in 0..model.model_joint_count() {
            let joint = model.joint_kinematics[joint_index];
            let local_transform = joint.frame(model.joint_value(q, joint_index));
            self.parent_from_child[joint_index] = local_transform;

            let parent_link_index = model.parent_link_indices[joint_index];
            let (parent_frame, parent_angular, parent_linear) = if parent_link_index == 0 {
                (Frame::identity(), Vector3::zeros(), Vector3::zeros())
            } else {
                let parent_joint = incoming_joint_index(parent_link_index);
                (
                    self.root_from_link[parent_joint],
                    self.root_angular_velocities[parent_joint],
                    self.root_origin_velocities[parent_joint],
                )
            };

            let frame = parent_frame * local_transform;
            self.root_from_link[joint_index] = frame;

            let offset = frame.translation.vector - parent_frame.translation.vector;
            let axis: Vector3<f64> = frame.rotation * joint.axis.as_ref();
            let mut child_angular = parent_angular;
            let mut child_linear = parent_linear + parent_angular.cross(&offset);
            let velocity = model.joint_value(qd, joint_index);
            match joint.joint_type {
                JointType::Revolute => child_angular += axis * velocity,
                JointType::Prismatic => child_linear += axis * velocity,
                JointType::Fixed => {}
            }
            self.root_angular_velocities[joint_index] = child_angular;
            self.root_origin_velocities[joint_index] = child_linear;
        }
        Ok(())
    }
}

#[derive(Debug)]
struct StateView<'a, B: BaseKind> {
    model: &'a Model,
    workspace: &'a mut Workspace,
    cache: &'a StateCache,
    base: BaseState,
    world_gravity: Vector3<f64>,
    _kind: PhantomData<B>,
}

impl<'a, B: BaseKind> StateView<'a, B> {
    fn forward_kinematics(&self, target: LinkId) -> Result<Frame> {
        let target_index = self.model.validate_link_id(target)?;
        let local = if target_index == 0 {
            Frame::identity()
        } else {
            self.cache.root_from_link[incoming_joint_index(target_index)]
        };
        let result = *self.base.frame() * local;
        Model::validate_numerical_result(
            "forward kinematics",
            result
                .translation
                .vector
                .iter()
                .chain(result.rotation.coords.iter()),
        )?;
        Ok(result)
    }

    fn forward_kinematics_all(&self, output: &mut [Frame]) -> Result<()> {
        self.model.validate_slice_length(
            "all link poses output",
            output.len(),
            self.model.link_count(),
        )?;
        let base = *self.base.frame();
        output[0] = base;
        for joint_index in 0..self.model.model_joint_count() {
            output[child_link_index(joint_index)] = base * self.cache.root_from_link[joint_index];
        }
        Model::validate_numerical_result(
            "forward kinematics",
            output.iter().flat_map(|frame| {
                frame
                    .translation
                    .vector
                    .iter()
                    .chain(frame.rotation.coords.iter())
            }),
        )
    }

    fn forward_velocity_kinematics(&self, target: LinkId, tool: &Frame) -> Result<Twist> {
        if !tool
            .translation
            .vector
            .iter()
            .chain(tool.rotation.coords.iter())
            .all(|v| v.is_finite())
        {
            return Err(Error::NonFiniteInput {
                input: "tool frame",
            });
        }
        let target_index = self.model.validate_link_id(target)?;
        let (local_target, local_angular, local_origin_linear) = if target_index == 0 {
            (Frame::identity(), Vector3::zeros(), Vector3::zeros())
        } else {
            let joint_index = incoming_joint_index(target_index);
            (
                self.cache.root_from_link[joint_index],
                self.cache.root_angular_velocities[joint_index],
                self.cache.root_origin_velocities[joint_index],
            )
        };
        let local_linear = local_origin_linear
            + local_angular.cross(&(local_target.rotation * tool.translation.vector));
        let base_rotation = self.base.frame().rotation;
        let base_velocity = self.base.velocity();
        let offset = base_rotation
            * (local_target.translation.vector + local_target.rotation * tool.translation.vector);
        let relative_angular = base_rotation * local_angular;
        let relative_linear = base_rotation * local_linear;
        let result = Twist::new(
            base_velocity.angular + relative_angular,
            base_velocity.linear + base_velocity.angular.cross(&offset) + relative_linear,
        );
        Model::validate_numerical_result(
            "forward velocity kinematics",
            result.angular.iter().chain(result.linear.iter()),
        )?;
        Ok(result)
    }

    fn forward_acceleration_kinematics(&mut self, qdd: &[f64], target: LinkId) -> Result<Twist> {
        self.model.validate_slice("qdd", qdd)?;
        let target_index = self.model.validate_link_id(target)?;
        let depth = self
            .model
            .prepare_ancestor_path(target_index, &mut self.workspace.ancestor_path);
        let path = &self.workspace.ancestor_path[..depth];

        let mut parent_frame = Frame::identity();
        let mut omega = Vector3::zeros();
        let mut velocity = Vector3::zeros();
        let mut alpha = Vector3::zeros();
        let mut acceleration = Vector3::zeros();
        for &joint_index in path.iter().rev() {
            let joint = self.model.joint_kinematics[joint_index];
            let frame = self.cache.root_from_link[joint_index];
            let offset = frame.translation.vector - parent_frame.translation.vector;
            let mut child_omega = omega;
            let mut child_velocity = velocity + omega.cross(&offset);
            let mut child_alpha = alpha;
            let mut child_acceleration =
                acceleration + alpha.cross(&offset) + omega.cross(&omega.cross(&offset));
            let axis = frame.rotation * joint.axis.as_ref();
            let joint_velocity = self.model.joint_value(&self.cache.qd, joint_index);
            let acceleration_value = self.model.joint_value(qdd, joint_index);
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
            parent_frame = frame;
            omega = child_omega;
            velocity = child_velocity;
            alpha = child_alpha;
            acceleration = child_acceleration;
        }

        let rotation = self.base.frame().rotation;
        let base_velocity = self.base.velocity();
        let base_acceleration = self.base.acceleration();
        let offset = rotation * parent_frame.translation.vector;
        let relative_angular = rotation * omega;
        let relative_linear = rotation * velocity;
        let result = Twist::new(
            base_acceleration.angular
                + base_velocity.angular.cross(&relative_angular)
                + rotation * alpha,
            base_acceleration.linear
                + base_acceleration.angular.cross(&offset)
                + base_velocity
                    .angular
                    .cross(&base_velocity.angular.cross(&offset))
                + 2.0 * base_velocity.angular.cross(&relative_linear)
                + rotation * acceleration,
        );
        Model::validate_numerical_result(
            "forward acceleration kinematics",
            result.angular.iter().chain(result.linear.iter()),
        )?;
        Ok(result)
    }

    fn jacobian(&self, target: LinkId, output: &mut [f64]) -> Result<()> {
        self.model.validate_slice_length(
            "jacobian output",
            output.len(),
            6 * generalized_count::<B>(self.model),
        )?;
        let target_index = self.model.validate_link_id(target)?;
        let base_rotation = self.base.frame().rotation;
        let local_target_pos = if target_index == 0 {
            Vector3::zeros()
        } else {
            self.cache.root_from_link[incoming_joint_index(target_index)]
                .translation
                .vector
        };
        output.fill(0.0);
        if B::IS_FLOATING {
            let world_target_pos = base_rotation * local_target_pos;
            for i in 0..3 {
                output[6 * i + i] = 1.0;
                let linear = Vector3::ith(i, 1.0).cross(&world_target_pos);
                output[6 * i + 3..6 * i + 6].copy_from_slice(linear.as_slice());
                output[6 * (i + 3) + i + 3] = 1.0;
            }
        }
        let mut current_link = target_index;
        while current_link != 0 {
            let joint_index = incoming_joint_index(current_link);
            current_link = self.model.parent_link_indices[joint_index];
            let Some(dof_index) = self.model.joint_dof_indices[joint_index] else {
                continue;
            };
            let joint = self.model.joint_kinematics[joint_index];
            let joint_frame = self.cache.root_from_link[joint_index];
            let local_axis: Vector3<f64> = joint_frame.rotation * joint.axis.as_ref();
            let world_axis = base_rotation * local_axis;
            let column = &mut output[6 * (B::DOF + dof_index)..6 * (B::DOF + dof_index + 1)];
            match joint.joint_type {
                JointType::Revolute => {
                    let local_linear =
                        local_axis.cross(&(local_target_pos - joint_frame.translation.vector));
                    column[..3].copy_from_slice(world_axis.as_slice());
                    column[3..].copy_from_slice((base_rotation * local_linear).as_slice());
                }
                JointType::Prismatic => column[3..].copy_from_slice(world_axis.as_slice()),
                JointType::Fixed => unreachable!("fixed joints have no DOF index"),
            }
        }
        Model::validate_numerical_result("jacobian", output.iter())
    }

    fn jacobian_derivative(&self, target: LinkId, output: &mut [f64]) -> Result<()> {
        self.model.validate_slice_length(
            "jacobian derivative output",
            output.len(),
            6 * generalized_count::<B>(self.model),
        )?;
        let target_index = self.model.validate_link_id(target)?;
        output.fill(0.0);
        if target_index == 0 {
            return Ok(());
        }
        let rotation = self.base.frame().rotation;
        let base_omega = self.base.velocity().angular;
        let target_joint = incoming_joint_index(target_index);
        let end_position = self.cache.root_from_link[target_joint].translation.vector;
        let end_velocity = self.cache.root_origin_velocities[target_joint];

        if B::IS_FLOATING {
            let offset = rotation * end_position;
            let offset_rate = base_omega.cross(&offset) + rotation * end_velocity;
            for axis_index in 0..3 {
                let axis = Vector3::ith(axis_index, 1.0);
                output[6 * axis_index + 3..6 * axis_index + 6]
                    .copy_from_slice(axis.cross(&offset_rate).as_slice());
            }
        }

        let mut current_link = target_index;
        while current_link != 0 {
            let joint_index = incoming_joint_index(current_link);
            current_link = self.model.parent_link_indices[joint_index];
            let Some(dof_index) = self.model.joint_dof_indices[joint_index] else {
                continue;
            };
            let joint = self.model.joint_kinematics[joint_index];
            let frame = self.cache.root_from_link[joint_index];
            let axis: Vector3<f64> = frame.rotation * joint.axis.as_ref();
            let axis_rate = self.cache.root_angular_velocities[joint_index].cross(&axis);
            let (local_j_ang, local_j_lin, local_jd_ang, local_jd_lin) = match joint.joint_type {
                JointType::Revolute => {
                    let moment_arm = end_position - frame.translation.vector;
                    let origin_velocity = self.cache.root_origin_velocities[joint_index];
                    let linear_rate = axis_rate.cross(&moment_arm)
                        + axis.cross(&(end_velocity - origin_velocity));
                    (axis, axis.cross(&moment_arm), axis_rate, linear_rate)
                }
                JointType::Prismatic => (Vector3::zeros(), axis, Vector3::zeros(), axis_rate),
                JointType::Fixed => unreachable!("fixed joints have no DOF index"),
            };
            let mut angular = rotation * local_jd_ang;
            let mut linear = rotation * local_jd_lin;
            if B::IS_FLOATING {
                angular += base_omega.cross(&(rotation * local_j_ang));
                linear += base_omega.cross(&(rotation * local_j_lin));
            }
            let column_index = B::DOF + dof_index;
            let column = &mut output[6 * column_index..6 * column_index + 6];
            column[..3].copy_from_slice(angular.as_slice());
            column[3..].copy_from_slice(linear.as_slice());
        }
        Model::validate_numerical_result("jacobian derivative", output.iter())
    }

    fn mass_matrix(&mut self, output: &mut [f64]) -> Result<()> {
        self.model.mass_matrix_from_transforms::<B>(
            self.base.frame(),
            &self.cache.parent_from_child,
            &mut self.workspace.composite_inertias,
            output,
        )
    }

    fn gravity(&mut self, loads: &[IndexedLoad], output: &mut [f64]) -> Result<()> {
        self.model.gravity_from_transforms::<B>(
            self.base.frame(),
            self.world_gravity,
            &self.cache.parent_from_child,
            loads,
            self.workspace,
            output,
        )
    }

    fn velocity_product_forces(&mut self, output: &mut [f64]) -> Result<()> {
        self.model.velocity_product_forces_from_transforms::<B>(
            BaseMotion::from_state(&self.base),
            &self.cache.parent_from_child,
            &self.cache.qd,
            self.workspace,
            output,
        )
    }

    fn inverse_dynamics(
        &mut self,
        qdd: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> Result<()> {
        self.model.inverse_dynamics_from_transforms::<B>(
            BaseMotion::from_state(&self.base),
            self.world_gravity,
            &self.cache.parent_from_child,
            &self.cache.qd,
            qdd,
            loads,
            self.workspace,
            output,
        )
    }

    fn forward_dynamics(
        &mut self,
        generalized_forces: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> Result<()> {
        self.model.forward_dynamics_from_transforms::<B>(
            self.base.frame(),
            self.base.velocity(),
            self.world_gravity,
            &self.cache.parent_from_child,
            &self.cache.qd,
            generalized_forces,
            loads,
            self.workspace,
            output,
        )
    }
}

/// Precomputed kinematic snapshot for a fixed-base [`Robot`] at one `(q, qd)` state.
///
/// Created by [`Robot::state`], this view caches joint transforms, link poses,
/// and link velocities in the robot's reusable workspace so multiple kinematics
/// and dynamics queries at the same state avoid redundant trigonometric and
/// forward-kinematics passes without allocating.
#[derive(Debug)]
pub struct RobotState<'a> {
    inner: StateView<'a, Fixed>,
}

/// Precomputed kinematic snapshot for a floating-base [`FloatingRobot`] at one
/// `(base, q, qd)` state.
///
/// Created by [`FloatingRobot::state`], this view caches joint transforms, link
/// poses, and link velocities in the robot's reusable workspace so multiple
/// kinematics and dynamics queries at the same state avoid redundant
/// trigonometric and forward-kinematics passes without allocating.
#[derive(Debug)]
pub struct FloatingRobotState<'a> {
    inner: StateView<'a, Floating>,
}

impl Robot {
    /// Precomputes link transforms, poses, and velocities at `(q, qd)` for
    /// repeated zero-allocation queries at this state.
    ///
    /// # Errors
    ///
    /// Returns an error if `q` or `qd` has the wrong length or contains a non-finite value.
    pub fn state<'a>(&'a mut self, q: &[f64], qd: &[f64]) -> Result<RobotState<'a>> {
        self.state_cache.populate(self.model.as_ref(), q, qd)?;
        Ok(RobotState {
            inner: StateView {
                model: self.model.as_ref(),
                workspace: &mut self.workspace,
                cache: &self.state_cache,
                base: BaseState::stationary_unchecked(self.world_from_root),
                world_gravity: self.world_gravity,
                _kind: PhantomData,
            },
        })
    }
}

impl FloatingRobot {
    /// Precomputes link transforms, poses, and velocities at `(base, q, qd)` for
    /// repeated zero-allocation queries at this state.
    ///
    /// # Errors
    ///
    /// Returns an error if `q` or `qd` has the wrong length or contains a non-finite value.
    pub fn state<'a>(
        &'a mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
    ) -> Result<FloatingRobotState<'a>> {
        self.state_cache.populate(self.model.as_ref(), q, qd)?;
        Ok(FloatingRobotState {
            inner: StateView {
                model: self.model.as_ref(),
                workspace: &mut self.workspace,
                cache: &self.state_cache,
                base: *base,
                world_gravity: self.world_gravity,
                _kind: PhantomData,
            },
        })
    }
}

macro_rules! impl_state_methods {
    ($ty:ident) => {
        impl $ty<'_> {
            /// Returns the cached world frame of a target link in `O(1)`.
            pub fn forward_kinematics(&self, target: LinkId) -> Result<Frame> {
                self.inner.forward_kinematics(target)
            }

            /// Writes all world link poses, including the root, in link-ID order.
            pub fn forward_kinematics_all(&self, output: &mut [Frame]) -> Result<()> {
                self.inner.forward_kinematics_all(output)
            }

            /// Computes the world-expressed spatial velocity of a target-link point in `O(1)`.
            pub fn forward_velocity_kinematics(
                &self,
                target: LinkId,
                tool: &Frame,
            ) -> Result<Twist> {
                self.inner.forward_velocity_kinematics(target, tool)
            }

            /// Computes the world-expressed spatial acceleration of a target-link origin.
            pub fn forward_acceleration_kinematics(
                &mut self,
                qdd: &[f64],
                target: LinkId,
            ) -> Result<Twist> {
                self.inner.forward_acceleration_kinematics(qdd, target)
            }

            /// Writes a world-expressed `6 x G` geometric Jacobian in column-major order.
            pub fn jacobian(&self, target: LinkId, output: &mut [f64]) -> Result<()> {
                self.inner.jacobian(target, output)
            }

            /// Writes the time derivative of the geometric Jacobian in column-major order.
            pub fn jacobian_derivative(&self, target: LinkId, output: &mut [f64]) -> Result<()> {
                self.inner.jacobian_derivative(target, output)
            }

            /// Writes the `G x G` mass matrix in column-major order using cached transforms.
            pub fn mass_matrix(&mut self, output: &mut [f64]) -> Result<()> {
                self.inner.mass_matrix(output)
            }

            /// Writes gravity and external-load generalized forces using cached transforms.
            pub fn gravity(&mut self, loads: &[IndexedLoad], output: &mut [f64]) -> Result<()> {
                self.inner.gravity(loads, output)
            }

            /// Writes velocity-product generalized forces `C(q, qd) * qd` using cached transforms.
            pub fn velocity_product_forces(&mut self, output: &mut [f64]) -> Result<()> {
                self.inner.velocity_product_forces(output)
            }

            /// Writes Newton-Euler generalized forces using cached transforms.
            pub fn inverse_dynamics(
                &mut self,
                qdd: &[f64],
                loads: &[IndexedLoad],
                output: &mut [f64],
            ) -> Result<()> {
                self.inner.inverse_dynamics(qdd, loads, output)
            }

            /// Writes articulated-body generalized accelerations using cached transforms.
            pub fn forward_dynamics(
                &mut self,
                generalized_forces: &[f64],
                loads: &[IndexedLoad],
                output: &mut [f64],
            ) -> Result<()> {
                self.inner
                    .forward_dynamics(generalized_forces, loads, output)
            }
        }
    };
}

impl_state_methods!(RobotState);
impl_state_methods!(FloatingRobotState);
