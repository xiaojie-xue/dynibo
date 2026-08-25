#![cfg(feature = "pinocchio-tests")]

use std::{ffi::CString, path::PathBuf, ptr::NonNull};

use dynibo::{
    BaseMode, BaseState, Frame, IndexedLoad, InverseKinematicsOptions, Robot, Twist, Wrench,
};
use nalgebra::{Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};

unsafe extern "C" {
    fn dynibo_pinocchio_create_for_frame(
        urdf_path: *const std::ffi::c_char,
        frame_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
    fn dynibo_pinocchio_create_floating_for_frame(
        urdf_path: *const std::ffi::c_char,
        frame_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
    fn dynibo_pinocchio_destroy(context: *mut std::ffi::c_void);
    fn dynibo_pinocchio_dof(context: *const std::ffi::c_void) -> usize;
    fn dynibo_pinocchio_configuration_size(context: *const std::ffi::c_void) -> usize;
    fn dynibo_pinocchio_neutral_configuration(context: *const std::ffi::c_void, q: *mut f64);
    fn dynibo_pinocchio_joint_configuration_index(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    fn dynibo_pinocchio_joint_configuration_dimension(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    fn dynibo_pinocchio_joint_velocity_index(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    fn dynibo_pinocchio_link_frame_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        rotation: *mut f64,
        translation: *mut f64,
    );
    fn dynibo_pinocchio_link_jacobian_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        jacobian: *mut f64,
    );
    fn dynibo_pinocchio_link_jacobian_derivative_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        derivative: *mut f64,
    );
    fn dynibo_pinocchio_link_velocity_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        velocity: *mut f64,
    );
    fn dynibo_pinocchio_link_acceleration_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        acceleration: *mut f64,
    );
    fn dynibo_pinocchio_gravity_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        gravity: *mut f64,
    );
    fn dynibo_pinocchio_rnea_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        torque: *mut f64,
    );
    fn dynibo_pinocchio_aba_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        torque: *const f64,
        acceleration: *mut f64,
    );
    fn dynibo_pinocchio_mass_matrix_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        mass: *mut f64,
    );
    fn dynibo_pinocchio_coriolis_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        coriolis: *mut f64,
    );
    fn dynibo_pinocchio_rnea_with_link_load_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        load: *const f64,
        torque: *mut f64,
    );
    fn dynibo_pinocchio_floating_rnea_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        base_translation: *const f64,
        base_rotation_xyzw: *const f64,
        base_velocity: *const f64,
        base_acceleration: *const f64,
        torque: *mut f64,
    );
}

#[derive(Clone, Copy, Debug)]
struct JointMapping {
    configuration_index: usize,
    configuration_dimension: usize,
    velocity_index: Option<usize>,
}

struct PinocchioContext {
    pointer: NonNull<std::ffi::c_void>,
    configuration_size: usize,
    velocity_size: usize,
    joint_mappings: Vec<JointMapping>,
}

impl PinocchioContext {
    fn new(robot: &Robot, path: &std::path::Path, frame_name: &str) -> Self {
        let path = CString::new(path.to_string_lossy().as_bytes()).unwrap();
        let frame_name = CString::new(frame_name).unwrap();
        // SAFETY: both C strings remain alive for the duration of the call.
        let pointer =
            unsafe { dynibo_pinocchio_create_for_frame(path.as_ptr(), frame_name.as_ptr()) };
        let pointer = NonNull::new(pointer).expect("Pinocchio must load the oracle fixture");
        Self::from_pointer(robot, pointer)
    }

    fn new_floating(robot: &Robot, path: &std::path::Path, frame_name: &str) -> Self {
        let path = CString::new(path.to_string_lossy().as_bytes()).unwrap();
        let frame_name = CString::new(frame_name).unwrap();
        // SAFETY: both C strings remain alive for the duration of the call.
        let pointer = unsafe {
            dynibo_pinocchio_create_floating_for_frame(path.as_ptr(), frame_name.as_ptr())
        };
        let pointer = NonNull::new(pointer).expect("Pinocchio must load the floating fixture");
        Self::from_pointer(robot, pointer)
    }

    fn from_pointer(robot: &Robot, pointer: NonNull<std::ffi::c_void>) -> Self {
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

    fn state(&self, q: &[f64], qd: &[f64], qdd: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
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
    fn floating_state(
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

    fn frame(&mut self, configuration: &[f64]) -> (Matrix3<f64>, Vector3<f64>) {
        let mut rotation = [0.0; 9];
        let mut translation = [0.0; 3];
        // SAFETY: all buffers have the dimensions required by this context.
        unsafe {
            dynibo_pinocchio_link_frame_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                rotation.as_mut_ptr(),
                translation.as_mut_ptr(),
            )
        };
        (
            Matrix3::from_column_slice(&rotation),
            Vector3::from_column_slice(&translation),
        )
    }

    fn jacobian(&mut self, configuration: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; 6 * self.velocity_size];
        // SAFETY: the output has `6 * model.nv` elements.
        unsafe {
            dynibo_pinocchio_link_jacobian_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_spatial_matrix_order(&pinocchio)
    }

    fn floating_jacobian(&mut self, configuration: &[f64], base: &Frame) -> Vec<f64> {
        let mut pinocchio = vec![0.0; 6 * self.velocity_size];
        unsafe {
            dynibo_pinocchio_link_jacobian_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.transform_floating_spatial_matrix(&pinocchio, None, base, Vector3::zeros())
    }

    fn jacobian_derivative(&mut self, configuration: &[f64], velocity: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; 6 * self.velocity_size];
        // SAFETY: the output has `6 * model.nv` elements.
        unsafe {
            dynibo_pinocchio_link_jacobian_derivative_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_spatial_matrix_order(&pinocchio)
    }

    fn floating_jacobian_derivative(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        base: &Frame,
        base_angular_velocity: Vector3<f64>,
    ) -> Vec<f64> {
        let mut jacobian = vec![0.0; 6 * self.velocity_size];
        let mut derivative = vec![0.0; 6 * self.velocity_size];
        unsafe {
            dynibo_pinocchio_link_jacobian_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                jacobian.as_mut_ptr(),
            );
            dynibo_pinocchio_link_jacobian_derivative_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                derivative.as_mut_ptr(),
            );
        }
        self.transform_floating_spatial_matrix(
            &derivative,
            Some(&jacobian),
            base,
            base_angular_velocity,
        )
    }

    fn velocity(&mut self, configuration: &[f64], velocity: &[f64]) -> [f64; 6] {
        let mut output = [0.0; 6];
        // SAFETY: input and output sizes match the context dimensions.
        unsafe {
            dynibo_pinocchio_link_velocity_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                output.as_mut_ptr(),
            )
        };
        output
    }

    fn acceleration(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
    ) -> [f64; 6] {
        let mut output = [0.0; 6];
        // SAFETY: input and output sizes match the context dimensions.
        unsafe {
            dynibo_pinocchio_link_acceleration_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                output.as_mut_ptr(),
            )
        };
        output
    }

    fn gravity(&mut self, configuration: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size];
        // SAFETY: the output has one value per Pinocchio velocity coordinate.
        unsafe {
            dynibo_pinocchio_gravity_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio)
    }

    fn rnea(&mut self, configuration: &[f64], velocity: &[f64], acceleration: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the context dimensions.
        unsafe {
            dynibo_pinocchio_rnea_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio)
    }

    fn aba(&mut self, configuration: &[f64], velocity: &[f64], torque: &[f64]) -> Vec<f64> {
        assert_eq!(torque.len(), self.joint_mappings.len());
        let mut pinocchio_torque = vec![0.0; self.velocity_size];
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(index) = mapping.velocity_index {
                pinocchio_torque[index] = torque[joint];
            }
        }
        let mut pinocchio_acceleration = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the context dimensions.
        unsafe {
            dynibo_pinocchio_aba_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio_torque.as_ptr(),
                pinocchio_acceleration.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio_acceleration)
    }

    fn mass_matrix(&mut self, configuration: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size * self.velocity_size];
        // SAFETY: the output has `model.nv * model.nv` elements, column-major.
        unsafe {
            dynibo_pinocchio_mass_matrix_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_square_order(&pinocchio)
    }

    fn floating_mass_matrix(&mut self, configuration: &[f64], base: &Frame) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size * self.velocity_size];
        unsafe {
            dynibo_pinocchio_mass_matrix_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        let transformations = self.floating_velocity_transform(base);
        let size = transformations.len();
        let mut output = vec![0.0; size * size];
        for column in 0..size {
            for row in 0..size {
                let mut value = 0.0;
                for &(pin_row, row_scale) in &transformations[row] {
                    for &(pin_column, column_scale) in &transformations[column] {
                        value += row_scale
                            * pinocchio[pin_column * self.velocity_size + pin_row]
                            * column_scale;
                    }
                }
                output[column * size + row] = value;
            }
        }
        output
    }

    fn floating_gravity(&mut self, configuration: &[f64], base: &Frame) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size];
        unsafe {
            dynibo_pinocchio_gravity_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.floating_generalized_order(&pinocchio, base)
    }

    fn floating_generalized_order(&self, pinocchio: &[f64], base: &Frame) -> Vec<f64> {
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

    fn floating_velocity_transform(&self, base: &Frame) -> Vec<Vec<(usize, f64)>> {
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

    fn transform_floating_spatial_matrix(
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

    fn coriolis_matrix(&mut self, configuration: &[f64], velocity: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size * self.velocity_size];
        // SAFETY: the output has `model.nv * model.nv` elements, column-major.
        unsafe {
            dynibo_pinocchio_coriolis_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_square_order(&pinocchio)
    }

    fn rnea_with_link_load(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        load: Wrench,
    ) -> Vec<f64> {
        let load = [
            load.torque.x,
            load.torque.y,
            load.torque.z,
            load.force.x,
            load.force.y,
            load.force.z,
        ];
        let mut pinocchio = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the context dimensions, and the load has six elements.
        unsafe {
            dynibo_pinocchio_rnea_with_link_load_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                load.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio)
    }

    fn floating_rnea(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        base: &Frame,
        base_velocity: Twist,
        base_acceleration: Twist,
    ) -> Vec<f64> {
        let base_velocity = [
            base_velocity.angular.x,
            base_velocity.angular.y,
            base_velocity.angular.z,
            base_velocity.linear.x,
            base_velocity.linear.y,
            base_velocity.linear.z,
        ];
        let base_acceleration = [
            base_acceleration.angular.x,
            base_acceleration.angular.y,
            base_acceleration.angular.z,
            base_acceleration.linear.x,
            base_acceleration.linear.y,
            base_acceleration.linear.z,
        ];
        let rotation = base.rotation.coords;
        let mut pinocchio = vec![0.0; self.velocity_size];
        // SAFETY: all state and pose buffers match the free-flyer context dimensions.
        unsafe {
            dynibo_pinocchio_floating_rnea_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                base.translation.vector.as_ptr(),
                rotation.as_ptr(),
                base_velocity.as_ptr(),
                base_acceleration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        let mut output = vec![0.0; 6 + self.joint_mappings.len()];
        let local_force = Vector3::new(pinocchio[0], pinocchio[1], pinocchio[2]);
        let local_torque = Vector3::new(pinocchio[3], pinocchio[4], pinocchio[5]);
        let world_torque = base.rotation * local_torque;
        let world_force = base.rotation * local_force;
        output[..3].copy_from_slice(world_torque.as_slice());
        output[3..6].copy_from_slice(world_force.as_slice());
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(index) = mapping.velocity_index {
                output[6 + joint] = pinocchio[index];
            }
        }
        output
    }

    fn floating_coriolis_from_rnea(
        &mut self,
        q: &[f64],
        qd: &[f64],
        base: &Frame,
        base_velocity: Twist,
    ) -> Vec<f64> {
        let size = 6 + q.len();
        let mut output = vec![0.0; size * size];
        let zero = vec![0.0; q.len()];
        for column in 0..size {
            let mut plus_qd = qd.to_vec();
            let mut plus_base = base_velocity;
            if column < 3 {
                plus_base.angular[column] += 1.0;
            } else if column < 6 {
                plus_base.linear[column - 3] += 1.0;
            } else {
                plus_qd[column - 6] += 1.0;
            }
            let (plus_q, plus_v, plus_a) = self.state(q, &plus_qd, &zero);
            let plus =
                self.floating_rnea(&plus_q, &plus_v, &plus_a, base, plus_base, Twist::zeros());

            let mut minus_qd = qd.to_vec();
            let mut minus_base = base_velocity;
            if column < 3 {
                minus_base.angular[column] -= 1.0;
            } else if column < 6 {
                minus_base.linear[column - 3] -= 1.0;
            } else {
                minus_qd[column - 6] -= 1.0;
            }
            let (minus_q, minus_v, minus_a) = self.state(q, &minus_qd, &zero);
            let minus = self.floating_rnea(
                &minus_q,
                &minus_v,
                &minus_a,
                base,
                minus_base,
                Twist::zeros(),
            );
            for row in 0..size {
                output[column * size + row] = 0.25 * (plus[row] - minus[row]);
            }
        }
        output
    }

    fn dynibo_joint_order(&self, pinocchio: &[f64]) -> Vec<f64> {
        self.joint_mappings
            .iter()
            .map(|mapping| pinocchio[mapping.velocity_index.expect("active joint")])
            .collect()
    }

    /// Reorders a column-major `6 x nv` Pinocchio matrix into dynibo's joint
    /// order, swapping the linear-first Pinocchio rows into dynibo's
    /// angular-first layout.
    fn dynibo_spatial_matrix_order(&self, pinocchio: &[f64]) -> Vec<f64> {
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
    fn dynibo_square_order(&self, pinocchio: &[f64]) -> Vec<f64> {
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

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/oracle_mixed.urdf")
}

fn tree_fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf")
}

fn deterministic_state(sample: usize) -> ([f64; 4], [f64; 4], [f64; 4]) {
    let wave =
        |joint: usize, phase: f64| ((sample + 1) as f64 * (joint + 2) as f64 * 0.619 + phase).sin();
    (
        [
            1.4 * wave(0, 0.1),
            5.0 * wave(1, 0.2),
            0.3 * wave(2, 0.3),
            2.7 * wave(3, 0.4),
        ],
        std::array::from_fn(|joint| 0.8 * wave(joint, 0.9)),
        std::array::from_fn(|joint| 1.1 * wave(joint, 1.7)),
    )
}

fn deterministic_mixed_state(sample: usize) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let (q, qd, qdd) = deterministic_state(sample);
    (
        [q[0], q[2], q[3]],
        [qd[0], qd[2], qd[3]],
        [qdd[0], qdd[2], qdd[3]],
    )
}

fn deterministic_tree_state(sample: usize) -> ([f64; 7], [f64; 7], [f64; 7]) {
    let values = |phase: f64, amplitude: f64| {
        std::array::from_fn(|joint| {
            let argument = (sample + 1) as f64 * (joint + 3) as f64 * 0.731 + phase;
            amplitude * argument.sin()
        })
    };
    (values(0.0, 0.9), values(0.7, 0.8), values(1.3, 1.1))
}

fn serial_fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf")
}

fn assert_close(actual: &[f64], expected: &[f64], absolute: f64, relative: f64, context: &str) {
    assert_eq!(actual.len(), expected.len());
    for (index, (&actual, &expected)) in actual.iter().zip(expected).enumerate() {
        let tolerance = absolute + relative * actual.abs().max(expected.abs());
        assert!(
            (actual - expected).abs() <= tolerance,
            "{context}: element {index}: actual={actual:.16e}, expected={expected:.16e}, tolerance={tolerance:.3e}"
        );
    }
}

#[test]
fn serial_arm_calculations_match_pinocchio() {
    let path = serial_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let target = robot.link_id("test_link_4").unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let q = [0.2, 1.0, -0.7, 0.4];
    let qd = [-0.3, 0.5, -0.2, 0.8];
    let qdd = [0.7, -0.4, 0.1, 0.3];
    let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);

    let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
    let actual_frame = robot
        .forward_kinematics(&dynibo::BaseState::fixed(), &q, target)
        .unwrap();
    assert_close(
        actual_frame
            .rotation
            .to_rotation_matrix()
            .matrix()
            .as_slice(),
        expected_rotation.as_slice(),
        1.0e-11,
        1.0e-11,
        "serial FK rotation",
    );
    assert_close(
        actual_frame.translation.vector.as_slice(),
        expected_translation.as_slice(),
        1.0e-11,
        1.0e-11,
        "serial FK translation",
    );

    let mut actual_jacobian = [f64::NAN; 24];
    robot
        .jacobian(
            &dynibo::BaseState::fixed(),
            &q,
            target,
            &mut actual_jacobian,
        )
        .unwrap();
    assert_close(
        &actual_jacobian,
        &pinocchio.jacobian(&pin_q),
        1.0e-10,
        1.0e-10,
        "serial Jacobian",
    );
    assert_close(
        robot
            .forward_velocity_kinematics(
                &dynibo::BaseState::fixed(),
                &q,
                &qd,
                target,
                &Frame::identity(),
            )
            .unwrap()
            .to_vector()
            .as_slice(),
        &pinocchio.velocity(&pin_q, &pin_qd),
        1.0e-10,
        1.0e-10,
        "serial velocity",
    );
    assert_close(
        robot
            .forward_acceleration_kinematics(&dynibo::BaseState::fixed(), &q, &qd, &qdd, target)
            .unwrap()
            .to_vector()
            .as_slice(),
        &pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd),
        1.0e-9,
        1.0e-10,
        "serial acceleration",
    );

    let mut actual_gravity = [f64::NAN; 4];
    robot
        .gravity(&dynibo::BaseState::fixed(), &q, &[], &mut actual_gravity)
        .unwrap();
    assert_close(
        &actual_gravity,
        &pinocchio.gravity(&pin_q),
        1.0e-9,
        1.0e-10,
        "serial gravity",
    );
    let mut actual_torque = [f64::NAN; 4];
    robot
        .inverse_dynamics(
            &dynibo::BaseState::fixed(),
            &q,
            &qd,
            &qdd,
            &[],
            &mut actual_torque,
        )
        .unwrap();
    assert_close(
        &actual_torque,
        &pinocchio.rnea(&pin_q, &pin_qd, &pin_qdd),
        1.0e-9,
        1.0e-10,
        "serial RNEA",
    );
}

#[test]
fn mixed_link_kinematics_match_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    assert_eq!(robot.joint_count(), 3);

    for link_name in ["base", "link_a", "mounted_link", "slider_link", "tool"] {
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..32 {
            let (q, qd, qdd) = deterministic_mixed_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
            let actual_frame = robot
                .forward_kinematics(&dynibo::BaseState::fixed(), &q, target)
                .unwrap();
            assert_close(
                actual_frame
                    .rotation
                    .to_rotation_matrix()
                    .matrix()
                    .as_slice(),
                expected_rotation.as_slice(),
                1.0e-11,
                1.0e-11,
                &format!("FK rotation for {link_name}, sample {sample}"),
            );
            assert_close(
                actual_frame.translation.vector.as_slice(),
                expected_translation.as_slice(),
                1.0e-11,
                1.0e-11,
                &format!("FK translation for {link_name}, sample {sample}"),
            );

            let mut actual_jacobian = vec![f64::NAN; 6 * robot.joint_count()];
            robot
                .jacobian(
                    &dynibo::BaseState::fixed(),
                    &q,
                    target,
                    &mut actual_jacobian,
                )
                .unwrap();
            let expected_jacobian = pinocchio.jacobian(&pin_q);
            assert_close(
                &actual_jacobian,
                &expected_jacobian,
                1.0e-10,
                1.0e-10,
                &format!("Jacobian for {link_name}, sample {sample}"),
            );

            let actual_velocity = robot
                .forward_velocity_kinematics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    target,
                    &Frame::identity(),
                )
                .unwrap();
            let expected_velocity = pinocchio.velocity(&pin_q, &pin_qd);
            assert_close(
                actual_velocity.to_vector().as_slice(),
                &expected_velocity,
                1.0e-10,
                1.0e-10,
                &format!("velocity for {link_name}, sample {sample}"),
            );

            let actual_acceleration = robot
                .forward_acceleration_kinematics(&dynibo::BaseState::fixed(), &q, &qd, &qdd, target)
                .unwrap();
            let expected_acceleration = pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd);
            assert_close(
                actual_acceleration.to_vector().as_slice(),
                &expected_acceleration,
                1.0e-9,
                1.0e-10,
                &format!("acceleration for {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn mass_matrices_match_pinocchio() {
    let path = serial_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let zero4 = [0.0; 4];
    for sample in 0..32 {
        let (q, _, _) = deterministic_state(sample);
        let (pin_q, _, _) = pinocchio.state(&q, &zero4, &zero4);
        let mut mass = vec![f64::NAN; 16];
        robot
            .mass_matrix(&dynibo::BaseState::fixed(), &q, &mut mass)
            .unwrap();
        assert_close(
            &mass,
            &pinocchio.mass_matrix(&pin_q),
            1.0e-9,
            1.0e-10,
            &format!("serial mass matrix sample {sample}"),
        );
    }

    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");
    let zero3 = [0.0; 3];
    for sample in 0..64 {
        let (q, _, _) = deterministic_mixed_state(sample);
        let (pin_q, _, _) = pinocchio.state(&q, &zero3, &zero3);
        let mut mass = vec![f64::NAN; 9];
        robot
            .mass_matrix(&dynibo::BaseState::fixed(), &q, &mut mass)
            .unwrap();
        assert_close(
            &mass,
            &pinocchio.mass_matrix(&pin_q),
            1.0e-9,
            1.0e-10,
            &format!("mixed mass matrix sample {sample}"),
        );
    }

    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "right_tool");
    let zero7 = [0.0; 7];
    for sample in 0..32 {
        let (q, _, _) = deterministic_tree_state(sample);
        let (pin_q, _, _) = pinocchio.state(&q, &zero7, &zero7);
        let mut mass = vec![f64::NAN; 49];
        robot
            .mass_matrix(&dynibo::BaseState::fixed(), &q, &mut mass)
            .unwrap();
        assert_close(
            &mass,
            &pinocchio.mass_matrix(&pin_q),
            1.0e-9,
            1.0e-10,
            &format!("tree mass matrix sample {sample}"),
        );
    }
}

#[test]
fn velocity_products_match_pinocchio() {
    let path = serial_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let zero4 = [0.0; 4];
    for sample in 0..32 {
        let (q, qd, _) = deterministic_state(sample);
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero4);
        let mut velocity_product = vec![f64::NAN; 4];
        robot
            .velocity_product_forces(&dynibo::BaseState::fixed(), &q, &qd, &mut velocity_product)
            .unwrap();
        let coriolis = pinocchio.coriolis_matrix(&pin_q, &pin_qd);
        let expected: Vec<f64> = (0..4)
            .map(|row| {
                (0..4)
                    .map(|column| coriolis[column * 4 + row] * qd[column])
                    .sum()
            })
            .collect();
        assert_close(
            &velocity_product,
            &expected,
            1.0e-9,
            1.0e-10,
            &format!("serial velocity product sample {sample}"),
        );
    }

    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");
    for sample in 0..64 {
        let (q, qd, _) = deterministic_mixed_state(sample);
        let zero = [0.0; 3];
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero);
        let mut velocity_product = vec![f64::NAN; 3];
        robot
            .velocity_product_forces(&dynibo::BaseState::fixed(), &q, &qd, &mut velocity_product)
            .unwrap();
        let coriolis = pinocchio.coriolis_matrix(&pin_q, &pin_qd);
        let expected: Vec<f64> = (0..3)
            .map(|row| {
                (0..3)
                    .map(|column| coriolis[column * 3 + row] * qd[column])
                    .sum()
            })
            .collect();
        assert_close(
            &velocity_product,
            &expected,
            1.0e-9,
            1.0e-10,
            &format!("mixed velocity product sample {sample}"),
        );
    }

    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "right_tool");
    let zero7 = [0.0; 7];
    for sample in 0..32 {
        let (q, qd, _) = deterministic_tree_state(sample);
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero7);
        let mut velocity_product = vec![f64::NAN; 7];
        robot
            .velocity_product_forces(&dynibo::BaseState::fixed(), &q, &qd, &mut velocity_product)
            .unwrap();
        let coriolis = pinocchio.coriolis_matrix(&pin_q, &pin_qd);
        let expected: Vec<f64> = (0..7)
            .map(|row| {
                (0..7)
                    .map(|column| coriolis[column * 7 + row] * qd[column])
                    .sum()
            })
            .collect();
        assert_close(
            &velocity_product,
            &expected,
            1.0e-9,
            1.0e-10,
            &format!("tree velocity product sample {sample}"),
        );
    }
}

#[test]
fn jacobian_time_variations_match_pinocchio() {
    let path = serial_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let target = robot.link_id("test_link_4").unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let zero4 = [0.0; 4];
    for sample in 0..32 {
        let (q, qd, _) = deterministic_state(sample);
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero4);
        let mut derivative = vec![f64::NAN; 24];
        robot
            .jacobian_derivative(
                &dynibo::BaseState::fixed(),
                &q,
                &qd,
                target,
                &mut derivative,
            )
            .unwrap();
        assert_close(
            &derivative,
            &pinocchio.jacobian_derivative(&pin_q, &pin_qd),
            1.0e-9,
            1.0e-10,
            &format!("serial Jacobian derivative sample {sample}"),
        );
    }

    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    for link_name in ["link_a", "mounted_link", "slider_link", "tool"] {
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..32 {
            let (q, qd, _) = deterministic_mixed_state(sample);
            let zero = [0.0; 3];
            let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero);
            let mut derivative = vec![f64::NAN; 18];
            robot
                .jacobian_derivative(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    target,
                    &mut derivative,
                )
                .unwrap();
            assert_close(
                &derivative,
                &pinocchio.jacobian_derivative(&pin_q, &pin_qd),
                1.0e-9,
                1.0e-10,
                &format!("mixed Jacobian derivative for {link_name}, sample {sample}"),
            );
        }
    }

    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let zero7 = [0.0; 7];
    for link_name in [
        "trunk",
        "left_lower",
        "left_tool",
        "right_lower",
        "right_tool",
    ] {
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..32 {
            let (q, qd, _) = deterministic_tree_state(sample);
            let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero7);
            let mut derivative = vec![f64::NAN; 42];
            robot
                .jacobian_derivative(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    target,
                    &mut derivative,
                )
                .unwrap();
            assert_close(
                &derivative,
                &pinocchio.jacobian_derivative(&pin_q, &pin_qd),
                1.0e-9,
                1.0e-10,
                &format!("tree Jacobian derivative for {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn mixed_joint_gravity_and_rnea_match_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");

    for sample in 0..64 {
        let (q, qd, qdd) = deterministic_mixed_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let mut actual_gravity = [f64::NAN; 3];
        robot
            .gravity(&dynibo::BaseState::fixed(), &q, &[], &mut actual_gravity)
            .unwrap();
        let expected_gravity = pinocchio.gravity(&pin_q);
        assert_close(
            &actual_gravity,
            &expected_gravity,
            1.0e-9,
            1.0e-10,
            &format!("gravity sample {sample}"),
        );

        let mut actual_torque = [f64::NAN; 3];
        robot
            .inverse_dynamics(
                &dynibo::BaseState::fixed(),
                &q,
                &qd,
                &qdd,
                &[],
                &mut actual_torque,
            )
            .unwrap();
        let expected_torque = pinocchio.rnea(&pin_q, &pin_qd, &pin_qdd);
        assert_close(
            &actual_torque,
            &expected_torque,
            1.0e-9,
            1.0e-10,
            &format!("RNEA sample {sample}"),
        );
    }
}

#[test]
fn mixed_joint_aba_matches_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");

    for sample in 0..32 {
        let (q, qd, _) = deterministic_mixed_state(sample);
        let zero = [0.0; 3];
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero);
        let torque: [f64; 3] = std::array::from_fn(|joint| {
            let phase = (sample + 1) as f64 * (joint + 2) as f64 * 0.413;
            8.0 * phase.sin()
        });
        let mut actual = [f64::NAN; 3];
        robot
            .forward_dynamics(&BaseState::fixed(), &q, &qd, &torque, &[], &mut actual)
            .unwrap();
        let expected = pinocchio.aba(&pin_q, &pin_qd, &torque);
        assert_close(
            &actual,
            &expected,
            2.0e-9,
            2.0e-10,
            &format!("ABA sample {sample}"),
        );
    }
}

#[test]
fn mixed_joint_external_loads_match_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let load = Wrench::new(
        Vector3::new(0.31, -0.27, 0.19),
        Vector3::new(-0.8, 0.55, 0.42),
    );

    for link_name in ["link_a", "mounted_link", "slider_link", "tool"] {
        let target = robot.link_id(link_name).unwrap();
        let indexed_load = IndexedLoad {
            link: target,
            wrench: load,
        };
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..16 {
            let (q, qd, qdd) = deterministic_mixed_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let mut actual = [f64::NAN; 3];
            robot
                .inverse_dynamics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    &qdd,
                    &[indexed_load],
                    &mut actual,
                )
                .unwrap();
            let expected = pinocchio.rnea_with_link_load(&pin_q, &pin_qd, &pin_qdd, load);
            assert_close(
                &actual,
                &expected,
                1.0e-9,
                1.0e-10,
                &format!("external load on {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn every_branched_link_frame_and_jacobian_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();

    for link_index in 0..robot.link_count() {
        let target = robot.link_id_at(link_index).unwrap();
        let link_name = robot.link_name(target).unwrap().to_owned();
        let mut pinocchio = PinocchioContext::new(&robot, &path, &link_name);
        for sample in 0..16 {
            let (q, qd, qdd) = deterministic_tree_state(sample);
            let (pin_q, _, _) = pinocchio.state(&q, &qd, &qdd);
            let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
            let actual_frame = robot
                .forward_kinematics(&dynibo::BaseState::fixed(), &q, target)
                .unwrap();
            assert_close(
                actual_frame
                    .rotation
                    .to_rotation_matrix()
                    .matrix()
                    .as_slice(),
                expected_rotation.as_slice(),
                1.0e-11,
                1.0e-11,
                &format!("tree FK rotation for {link_name}, sample {sample}"),
            );
            assert_close(
                actual_frame.translation.vector.as_slice(),
                expected_translation.as_slice(),
                1.0e-11,
                1.0e-11,
                &format!("tree FK translation for {link_name}, sample {sample}"),
            );
            let mut actual_jacobian = vec![f64::NAN; 6 * robot.joint_count()];
            robot
                .jacobian(
                    &dynibo::BaseState::fixed(),
                    &q,
                    target,
                    &mut actual_jacobian,
                )
                .unwrap();
            let expected_jacobian = pinocchio.jacobian(&pin_q);
            assert_close(
                &actual_jacobian,
                &expected_jacobian,
                1.0e-10,
                1.0e-10,
                &format!("tree Jacobian for {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn branched_velocity_and_acceleration_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();

    for link_name in ["left_tool", "right_tool"] {
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..32 {
            let (q, qd, qdd) = deterministic_tree_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let velocity = robot
                .forward_velocity_kinematics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    target,
                    &Frame::identity(),
                )
                .unwrap();
            assert_close(
                velocity.to_vector().as_slice(),
                &pinocchio.velocity(&pin_q, &pin_qd),
                1.0e-10,
                1.0e-10,
                &format!("tree velocity for {link_name}, sample {sample}"),
            );
            let acceleration = robot
                .forward_acceleration_kinematics(&dynibo::BaseState::fixed(), &q, &qd, &qdd, target)
                .unwrap();
            assert_close(
                acceleration.to_vector().as_slice(),
                &pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd),
                1.0e-9,
                1.0e-10,
                &format!("tree acceleration for {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn branched_gravity_and_rnea_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "right_tool");

    for sample in 0..32 {
        let (q, qd, qdd) = deterministic_tree_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let mut gravity = [f64::NAN; 7];
        robot
            .gravity(&dynibo::BaseState::fixed(), &q, &[], &mut gravity)
            .unwrap();
        assert_close(
            &gravity,
            &pinocchio.gravity(&pin_q),
            1.0e-9,
            1.0e-10,
            &format!("tree gravity sample {sample}"),
        );
        let mut torque = [f64::NAN; 7];
        robot
            .inverse_dynamics(&dynibo::BaseState::fixed(), &q, &qd, &qdd, &[], &mut torque)
            .unwrap();
        assert_close(
            &torque,
            &pinocchio.rnea(&pin_q, &pin_qd, &pin_qdd),
            1.0e-9,
            1.0e-10,
            &format!("tree RNEA sample {sample}"),
        );
    }
}

#[test]
fn branched_moving_external_loads_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let load = Wrench::new(
        Vector3::new(-0.22, 0.35, 0.41),
        Vector3::new(0.74, -0.63, 0.28),
    );

    for link_name in ["trunk", "left_lower", "left_tool", "right_tool"] {
        let target = robot.link_id(link_name).unwrap();
        let indexed_load = IndexedLoad {
            link: target,
            wrench: load,
        };
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..16 {
            let (q, qd, qdd) = deterministic_tree_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let mut actual = [f64::NAN; 7];
            robot
                .inverse_dynamics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    &qdd,
                    &[indexed_load],
                    &mut actual,
                )
                .unwrap();
            let expected = pinocchio.rnea_with_link_load(&pin_q, &pin_qd, &pin_qdd, load);
            assert_close(
                &actual,
                &expected,
                1.0e-9,
                1.0e-10,
                &format!("tree external load on {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn mixed_joint_ik_reaches_pinocchio_generated_targets() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let target = robot.link_id("tool").unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");
    let options = InverseKinematicsOptions {
        max_iterations: 200,
        damping: 2.0e-3,
        max_step_norm: 0.25,
        ..InverseKinematicsOptions::default()
    };

    for sample in 0..16 {
        let phase = (sample + 1) as f64;
        let target_q = [
            1.0 * (phase * 0.47).sin(),
            0.22 * (phase * 0.31).sin(),
            1.7 * (phase * 0.59).sin(),
        ];
        let zero = [0.0; 3];
        let (pin_target_q, _, _) = pinocchio.state(&target_q, &zero, &zero);
        let (rotation, translation) = pinocchio.frame(&pin_target_q);
        let desired = Frame::from_parts(
            Translation3::from(translation),
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation)),
        );
        let initial = [
            target_q[0] + 0.12 * (phase * 0.73).sin(),
            target_q[1] + 0.05 * (phase * 0.41).cos(),
            target_q[2] - 0.15 * (phase * 0.37).sin(),
        ];
        let mut solution = [f64::NAN; 3];
        robot
            .inverse_kinematics(
                &dynibo::BaseState::fixed(),
                &initial,
                target,
                &desired,
                options,
                &mut solution,
            )
            .unwrap_or_else(|error| panic!("IK failed for sample {sample}: {error}"));

        let (pin_solution_q, _, _) = pinocchio.state(&solution, &zero, &zero);
        let (solved_rotation, solved_translation) = pinocchio.frame(&pin_solution_q);
        assert_close(
            solved_rotation.as_slice(),
            desired.rotation.to_rotation_matrix().matrix().as_slice(),
            2.0e-6,
            1.0e-10,
            &format!("IK rotation sample {sample}"),
        );
        assert_close(
            solved_translation.as_slice(),
            desired.translation.vector.as_slice(),
            2.0e-6,
            1.0e-10,
            &format!("IK translation sample {sample}"),
        );
    }
}

#[test]
fn floating_base_kinematics_and_dynamics_match_free_flyer_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf_with_base(&path, BaseMode::Floating).unwrap();
    let target = robot.link_id("tool").unwrap();
    let mut pinocchio = PinocchioContext::new_floating(&robot, &path, "tool");

    for sample in 0..12 {
        let (q, qd, qdd) = deterministic_mixed_state(sample);
        let phase = sample as f64 + 1.0;
        let base = Frame::from_parts(
            Translation3::new(0.2, -0.3, 0.4),
            UnitQuaternion::from_euler_angles(
                0.3 * (phase * 0.23).sin(),
                -0.25 * (phase * 0.31).cos(),
                0.2 * (phase * 0.17).sin(),
            ),
        );
        let base_velocity = Twist::new(
            Vector3::new(0.21, -0.17, 0.13),
            Vector3::new(-0.3, 0.2, 0.1),
        );
        let base_acceleration = Twist::new(
            Vector3::new(-0.11, 0.14, 0.09),
            Vector3::new(0.35, -0.22, 0.18),
        );
        let base_state = dynibo::BaseState::new(base, base_velocity, base_acceleration).unwrap();
        let (pin_q, pin_qd, pin_qdd) =
            pinocchio.floating_state(&q, &qd, &qdd, &base, base_velocity, base_acceleration);

        let actual_frame = robot.forward_kinematics(&base_state, &q, target).unwrap();
        let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
        assert_close(
            actual_frame
                .rotation
                .to_rotation_matrix()
                .matrix()
                .as_slice(),
            expected_rotation.as_slice(),
            2.0e-11,
            1.0e-11,
            &format!("floating FK rotation sample {sample}"),
        );
        assert_close(
            actual_frame.translation.vector.as_slice(),
            expected_translation.as_slice(),
            2.0e-11,
            1.0e-11,
            &format!("floating FK translation sample {sample}"),
        );

        let mut actual_jacobian = vec![0.0; 6 * robot.generalized_count()];
        robot
            .jacobian(&base_state, &q, target, &mut actual_jacobian)
            .unwrap();
        assert_close(
            &actual_jacobian,
            &pinocchio.floating_jacobian(&pin_q, &base),
            2.0e-10,
            1.0e-10,
            &format!("floating Jacobian sample {sample}"),
        );
        let mut actual_derivative = vec![0.0; actual_jacobian.len()];
        robot
            .jacobian_derivative(&base_state, &q, &qd, target, &mut actual_derivative)
            .unwrap();
        assert_close(
            &actual_derivative,
            &pinocchio.floating_jacobian_derivative(&pin_q, &pin_qd, &base, base_velocity.angular),
            3.0e-9,
            1.0e-9,
            &format!("floating Jacobian derivative sample {sample}"),
        );

        assert_close(
            robot
                .forward_velocity_kinematics(&base_state, &q, &qd, target, &Frame::identity())
                .unwrap()
                .to_vector()
                .as_slice(),
            &pinocchio.velocity(&pin_q, &pin_qd),
            2.0e-10,
            1.0e-10,
            &format!("floating velocity sample {sample}"),
        );
        assert_close(
            robot
                .forward_acceleration_kinematics(&base_state, &q, &qd, &qdd, target)
                .unwrap()
                .to_vector()
                .as_slice(),
            &pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd),
            3.0e-9,
            1.0e-9,
            &format!("floating acceleration sample {sample}"),
        );

        let n = robot.generalized_count();
        let mut actual_mass = vec![0.0; n * n];
        robot
            .mass_matrix(&base_state, &q, &mut actual_mass)
            .unwrap();
        assert_close(
            &actual_mass,
            &pinocchio.floating_mass_matrix(&pin_q, &base),
            2.0e-9,
            1.0e-9,
            &format!("floating mass matrix sample {sample}"),
        );
        let mut actual_gravity = vec![0.0; n];
        robot
            .gravity(&base_state, &q, &[], &mut actual_gravity)
            .unwrap();
        assert_close(
            &actual_gravity,
            &pinocchio.floating_gravity(&pin_q, &base),
            2.0e-9,
            1.0e-9,
            &format!("floating gravity sample {sample}"),
        );
        let mut actual_velocity_product = vec![0.0; n];
        robot
            .velocity_product_forces(&base_state, &q, &qd, &mut actual_velocity_product)
            .unwrap();
        let coriolis = pinocchio.floating_coriolis_from_rnea(&q, &qd, &base, base_velocity);
        let generalized_velocity = [
            base_velocity.angular[0],
            base_velocity.angular[1],
            base_velocity.angular[2],
            base_velocity.linear[0],
            base_velocity.linear[1],
            base_velocity.linear[2],
            qd[0],
            qd[1],
            qd[2],
        ];
        let expected: Vec<f64> = (0..n)
            .map(|row| {
                (0..n)
                    .map(|column| coriolis[column * n + row] * generalized_velocity[column])
                    .sum()
            })
            .collect();
        assert_close(
            &actual_velocity_product,
            &expected,
            3.0e-9,
            1.0e-9,
            &format!("floating velocity product sample {sample}"),
        );
    }
}

#[test]
fn mixed_joint_moving_base_rnea_matches_free_flyer_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf_with_base(&path, BaseMode::Floating).unwrap();
    let mut pinocchio = PinocchioContext::new_floating(&robot, &path, "tool");

    for sample in 0..16 {
        let (q, qd, qdd) = deterministic_mixed_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let phase = sample as f64 + 1.0;
        let base = Frame::from_parts(
            Translation3::new(0.2, -0.3, 0.4),
            UnitQuaternion::from_euler_angles(
                0.3 * (phase * 0.23).sin(),
                -0.25 * (phase * 0.31).cos(),
                0.2 * (phase * 0.17).sin(),
            ),
        );
        let base_velocity = Twist::new(
            Vector3::new(0.21, -0.17, 0.13),
            Vector3::new(-0.3, 0.2, 0.1),
        );
        let base_acceleration = Twist::new(
            Vector3::new(-0.11, 0.14, 0.09),
            Vector3::new(0.35, -0.22, 0.18),
        );
        let base_state = dynibo::BaseState::new(base, base_velocity, base_acceleration).unwrap();
        let mut actual = [f64::NAN; 9];
        robot
            .inverse_dynamics(&base_state, &q, &qd, &qdd, &[], &mut actual)
            .unwrap();
        let expected = pinocchio.floating_rnea(
            &pin_q,
            &pin_qd,
            &pin_qdd,
            &base,
            base_velocity,
            base_acceleration,
        );
        assert_close(
            &actual,
            &expected,
            1.0e-9,
            1.0e-10,
            &format!("moving-base RNEA sample {sample}"),
        );
    }
}
