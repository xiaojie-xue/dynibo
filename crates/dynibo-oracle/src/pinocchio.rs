#![allow(dead_code)]

use std::{ffi::CString, ptr::NonNull};

use dynibo::{FloatingRobot, Frame, Robot, Twist, Wrench};
use nalgebra::{Matrix3, Vector3};

#[path = "pinocchio/dynamics.rs"]
mod dynamics;
#[path = "pinocchio/ffi.rs"]
mod ffi;
#[path = "pinocchio/kinematics.rs"]
mod kinematics;

use ffi::*;

#[derive(Clone, Copy, Debug)]
struct JointMapping {
    configuration_index: usize,
    configuration_dimension: usize,
    velocity_index: Option<usize>,
}

#[derive(Clone, Copy, Debug)]
pub struct PinocchioLoad {
    frame_index: usize,
    wrench: Wrench,
}

pub struct PinocchioContext {
    pointer: NonNull<std::ffi::c_void>,
    configuration_size: usize,
    velocity_size: usize,
    joint_mappings: Vec<JointMapping>,
}

trait RobotMetadata {
    fn joint_count(&self) -> usize;
    fn joint_name(&self, dof_index: usize) -> dynibo::Result<&str>;
}

impl RobotMetadata for Robot {
    fn joint_count(&self) -> usize {
        self.joint_count()
    }

    fn joint_name(&self, dof_index: usize) -> dynibo::Result<&str> {
        self.joint_name(dof_index)
    }
}

impl RobotMetadata for FloatingRobot {
    fn joint_count(&self) -> usize {
        self.joint_count()
    }

    fn joint_name(&self, dof_index: usize) -> dynibo::Result<&str> {
        self.joint_name(dof_index)
    }
}

impl PinocchioContext {
    pub fn new(robot: &Robot, path: &std::path::Path, frame_name: &str) -> Self {
        let path = CString::new(path.to_string_lossy().as_bytes()).unwrap();
        let frame_name = CString::new(frame_name).unwrap();
        // SAFETY: both C strings remain alive for the duration of the call.
        let pointer =
            unsafe { dynibo_pinocchio_create_for_frame(path.as_ptr(), frame_name.as_ptr()) };
        let pointer = NonNull::new(pointer).expect("Pinocchio must load the oracle fixture");
        Self::from_pointer(robot, pointer)
    }

    pub fn new_floating(robot: &FloatingRobot, path: &std::path::Path, frame_name: &str) -> Self {
        let path = CString::new(path.to_string_lossy().as_bytes()).unwrap();
        let frame_name = CString::new(frame_name).unwrap();
        // SAFETY: both C strings remain alive for the duration of the call.
        let pointer = unsafe {
            dynibo_pinocchio_create_floating_for_frame(path.as_ptr(), frame_name.as_ptr())
        };
        let pointer = NonNull::new(pointer).expect("Pinocchio must load the floating fixture");
        Self::from_pointer(robot, pointer)
    }

    fn from_pointer(robot: &impl RobotMetadata, pointer: NonNull<std::ffi::c_void>) -> Self {
        // SAFETY: `pointer` owns a live Pinocchio context.
        let configuration_size = unsafe { dynibo_pinocchio_configuration_size(pointer.as_ptr()) };
        // SAFETY: `pointer` owns a live Pinocchio context.
        let velocity_size = unsafe { dynibo_pinocchio_dof(pointer.as_ptr()) };
        let joint_mappings = (0..robot.joint_count())
            .filter_map(|dof_index| {
                let name = CString::new(robot.joint_name(dof_index).unwrap()).unwrap();
                // SAFETY: the context and name are valid for each query.
                let configuration_index = unsafe {
                    dynibo_pinocchio_joint_configuration_index(pointer.as_ptr(), name.as_ptr())
                };
                // SAFETY: the context and name are valid for each query.
                let configuration_dimension = unsafe {
                    dynibo_pinocchio_joint_configuration_dimension(pointer.as_ptr(), name.as_ptr())
                };
                // SAFETY: the context and name are valid for each query.
                let velocity_index = unsafe {
                    dynibo_pinocchio_joint_velocity_index(pointer.as_ptr(), name.as_ptr())
                };
                let mapping = JointMapping {
                    configuration_index,
                    configuration_dimension,
                    velocity_index: (velocity_index < velocity_size).then_some(velocity_index),
                };
                mapping.velocity_index.map(|_| mapping)
            })
            .collect();
        Self {
            pointer,
            configuration_size,
            velocity_size,
            joint_mappings,
        }
    }

    pub fn load(&self, frame_name: &str, wrench: Wrench) -> PinocchioLoad {
        let frame_name = CString::new(frame_name).unwrap();
        // SAFETY: the context and frame name are valid for this query.
        let frame_index =
            unsafe { dynibo_pinocchio_frame_index(self.pointer.as_ptr(), frame_name.as_ptr()) };
        assert_ne!(
            frame_index,
            usize::MAX,
            "Pinocchio frame {frame_name:?} must exist"
        );
        PinocchioLoad {
            frame_index,
            wrench,
        }
    }

    fn load_buffers(loads: &[PinocchioLoad]) -> (Vec<usize>, Vec<f64>) {
        let frame_indices = loads.iter().map(|load| load.frame_index).collect();
        let mut values = Vec::with_capacity(6 * loads.len());
        for load in loads {
            values.extend_from_slice(&[
                load.wrench.torque.x,
                load.wrench.torque.y,
                load.wrench.torque.z,
                load.wrench.force.x,
                load.wrench.force.y,
                load.wrench.force.z,
            ]);
        }
        (frame_indices, values)
    }

    fn pinocchio_joint_forces(&self, forces: &[f64]) -> Vec<f64> {
        assert_eq!(forces.len(), self.joint_mappings.len());
        let mut pinocchio = vec![0.0; self.velocity_size];
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            pinocchio[mapping.velocity_index.expect("active joint")] = forces[joint];
        }
        pinocchio
    }

    pub fn state(&self, q: &[f64], qd: &[f64], qdd: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        assert_eq!(q.len(), self.joint_mappings.len());
        assert_eq!(qd.len(), q.len());
        assert_eq!(qdd.len(), q.len());
        let mut configuration = vec![0.0; self.configuration_size];
        // SAFETY: the output contains exactly `model.nq` scalars.
        unsafe {
            dynibo_pinocchio_neutral_configuration(
                self.pointer.as_ptr(),
                configuration.as_mut_ptr(),
            )
        };
        let mut velocity = vec![0.0; self.velocity_size];
        let mut acceleration = vec![0.0; self.velocity_size];
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            match mapping.configuration_dimension {
                0 => {}
                1 => configuration[mapping.configuration_index] = q[joint],
                2 => {
                    configuration[mapping.configuration_index] = q[joint].cos();
                    configuration[mapping.configuration_index + 1] = q[joint].sin();
                }
                dimension => panic!("unsupported Pinocchio joint configuration size {dimension}"),
            }
            if let Some(index) = mapping.velocity_index {
                velocity[index] = qd[joint];
                acceleration[index] = qdd[joint];
            }
        }
        (configuration, velocity, acceleration)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn floating_state(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        base: &Frame,
        base_velocity: Twist,
        base_acceleration: Twist,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let (mut configuration, mut velocity, mut acceleration) = self.state(q, qd, qdd);
        configuration[..3].copy_from_slice(base.translation.vector.as_slice());
        configuration[3..7].copy_from_slice(base.rotation.coords.as_slice());
        let world_to_base = base.rotation.inverse();
        let local_angular_velocity = world_to_base * base_velocity.angular;
        let local_linear_velocity = world_to_base * base_velocity.linear;
        velocity[..3].copy_from_slice(local_linear_velocity.as_slice());
        velocity[3..6].copy_from_slice(local_angular_velocity.as_slice());
        let local_linear_acceleration = world_to_base * base_acceleration.linear
            - local_angular_velocity.cross(&local_linear_velocity);
        let local_angular_acceleration = world_to_base * base_acceleration.angular;
        acceleration[..3].copy_from_slice(local_linear_acceleration.as_slice());
        acceleration[3..6].copy_from_slice(local_angular_acceleration.as_slice());
        (configuration, velocity, acceleration)
    }

    pub fn floating_generalized_order(&self, pinocchio: &[f64], base: &Frame) -> Vec<f64> {
        let mut output = vec![0.0; 6 + self.joint_mappings.len()];
        let world_torque = base.rotation * Vector3::from_column_slice(&pinocchio[3..6]);
        let world_force = base.rotation * Vector3::from_column_slice(&pinocchio[..3]);
        output[..3].copy_from_slice(world_torque.as_slice());
        output[3..6].copy_from_slice(world_force.as_slice());
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            output[6 + joint] = pinocchio[mapping.velocity_index.expect("active joint")];
        }
        output
    }

    pub fn floating_velocity_transform(&self, base: &Frame) -> Vec<Vec<(usize, f64)>> {
        let mut columns = vec![Vec::new(); 6 + self.joint_mappings.len()];
        let inverse = base.rotation.inverse();
        for axis_index in 0..3 {
            let local_axis = inverse * Vector3::ith(axis_index, 1.0);
            for local_index in 0..3 {
                columns[axis_index].push((3 + local_index, local_axis[local_index]));
                columns[3 + axis_index].push((local_index, local_axis[local_index]));
            }
        }
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(index) = mapping.velocity_index {
                columns[6 + joint].push((index, 1.0));
            }
        }
        columns
    }

    pub fn transform_floating_spatial_matrix(
        &self,
        primary: &[f64],
        transform_derivative_source: Option<&[f64]>,
        base: &Frame,
        base_angular_velocity: Vector3<f64>,
    ) -> Vec<f64> {
        let transformations = self.floating_velocity_transform(base);
        let size = transformations.len();
        let mut output = vec![0.0; 6 * size];
        for column in 0..size {
            for output_row in 0..6 {
                let pin_row = if output_row < 3 {
                    output_row + 3
                } else {
                    output_row - 3
                };
                let mut value = transformations[column]
                    .iter()
                    .map(|&(pin_column, scale)| primary[6 * pin_column + pin_row] * scale)
                    .sum::<f64>();
                if let Some(jacobian) = transform_derivative_source
                    && column < 6
                {
                    let local_axis = base.rotation.inverse() * Vector3::ith(column % 3, 1.0);
                    let local_omega = base.rotation.inverse() * base_angular_velocity;
                    let derivative = -local_omega.cross(&local_axis);
                    let pin_offset = if column < 3 { 3 } else { 0 };
                    for local_index in 0..3 {
                        value += jacobian[6 * (pin_offset + local_index) + pin_row]
                            * derivative[local_index];
                    }
                }
                output[6 * column + output_row] = value;
            }
        }
        output
    }

    pub fn dynibo_joint_order(&self, pinocchio: &[f64]) -> Vec<f64> {
        self.joint_mappings
            .iter()
            .map(|mapping| pinocchio[mapping.velocity_index.expect("active joint")])
            .collect()
    }

    /// Reorders a column-major `6 x nv` Pinocchio matrix into dynibo's joint
    /// order, swapping the linear-first Pinocchio rows into dynibo's
    /// angular-first layout.
    pub fn dynibo_spatial_matrix_order(&self, pinocchio: &[f64]) -> Vec<f64> {
        let mut dynibo_order = vec![0.0; 6 * self.joint_mappings.len()];
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            let column = mapping.velocity_index.expect("active joint");
            for row in 0..6 {
                let pinocchio_row = if row < 3 { row + 3 } else { row - 3 };
                dynibo_order[6 * joint + row] = pinocchio[6 * column + pinocchio_row];
            }
        }
        dynibo_order
    }

    /// Reorders a column-major `nv x nv` Pinocchio matrix into dynibo's joint order.
    pub fn dynibo_square_order(&self, pinocchio: &[f64]) -> Vec<f64> {
        let joint_count = self.joint_mappings.len();
        let mut dynibo_order = vec![0.0; joint_count * joint_count];
        for (row, row_mapping) in self.joint_mappings.iter().enumerate() {
            for (column, column_mapping) in self.joint_mappings.iter().enumerate() {
                let row_index = row_mapping.velocity_index.expect("active joint");
                let column_index = column_mapping.velocity_index.expect("active joint");
                dynibo_order[column * joint_count + row] =
                    pinocchio[column_index * self.velocity_size + row_index];
            }
        }
        dynibo_order
    }
}

impl Drop for PinocchioContext {
    fn drop(&mut self) {
        // SAFETY: this context is owned here and destroyed exactly once.
        unsafe { dynibo_pinocchio_destroy(self.pointer.as_ptr()) };
    }
}
