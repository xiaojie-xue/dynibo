#![allow(dead_code)]

use std::{ffi::CString, ptr::NonNull};

use dynibo::{FloatingRobot, Frame, Robot, Twist, Wrench};
use nalgebra::{Matrix3, Vector3};

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
    fn dynibo_pinocchio_frame_index(
        context: *const std::ffi::c_void,
        frame_name: *const std::ffi::c_char,
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
    fn dynibo_pinocchio_aba_with_link_load_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        torque: *const f64,
        load: *const f64,
        acceleration: *mut f64,
    );
    fn dynibo_pinocchio_aba_with_loads_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        torque: *const f64,
        frame_indices: *const usize,
        loads: *const f64,
        load_count: usize,
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
    fn dynibo_pinocchio_rnea_with_loads_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        frame_indices: *const usize,
        loads: *const f64,
        load_count: usize,
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

    fn rnea_with_loads_raw(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let (frame_indices, load_values) = Self::load_buffers(loads);
        let mut output = vec![0.0; self.velocity_size];
        // SAFETY: state and output buffers match the model dimensions; each
        // frame index has one six-element wrench in `load_values`.
        unsafe {
            dynibo_pinocchio_rnea_with_loads_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                frame_indices.as_ptr(),
                load_values.as_ptr(),
                loads.len(),
                output.as_mut_ptr(),
            )
        };
        output
    }

    fn aba_with_loads_raw(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        generalized_forces: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        assert_eq!(generalized_forces.len(), self.velocity_size);
        let (frame_indices, load_values) = Self::load_buffers(loads);
        let mut output = vec![0.0; self.velocity_size];
        // SAFETY: state, force, and output buffers match the model dimensions;
        // each frame index has one six-element wrench in `load_values`.
        unsafe {
            dynibo_pinocchio_aba_with_loads_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                generalized_forces.as_ptr(),
                frame_indices.as_ptr(),
                load_values.as_ptr(),
                loads.len(),
                output.as_mut_ptr(),
            )
        };
        output
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

    pub fn frame(&mut self, configuration: &[f64]) -> (Matrix3<f64>, Vector3<f64>) {
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

    pub fn jacobian(&mut self, configuration: &[f64]) -> Vec<f64> {
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

    pub fn floating_jacobian(&mut self, configuration: &[f64], base: &Frame) -> Vec<f64> {
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

    pub fn jacobian_derivative(&mut self, configuration: &[f64], velocity: &[f64]) -> Vec<f64> {
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

    pub fn floating_jacobian_derivative(
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

    pub fn velocity(&mut self, configuration: &[f64], velocity: &[f64]) -> [f64; 6] {
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

    pub fn acceleration(
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

    pub fn gravity(&mut self, configuration: &[f64]) -> Vec<f64> {
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

    pub fn rnea(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
    ) -> Vec<f64> {
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

    pub fn rnea_with_loads(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let pinocchio = self.rnea_with_loads_raw(configuration, velocity, acceleration, loads);
        self.dynibo_joint_order(&pinocchio)
    }

    pub fn gravity_with_loads(
        &mut self,
        configuration: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let zero = vec![0.0; self.velocity_size];
        self.rnea_with_loads(configuration, &zero, &zero, loads)
    }

    pub fn aba(&mut self, configuration: &[f64], velocity: &[f64], torque: &[f64]) -> Vec<f64> {
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

    pub fn aba_with_loads(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        torque: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let pinocchio_torque = self.pinocchio_joint_forces(torque);
        let acceleration =
            self.aba_with_loads_raw(configuration, velocity, &pinocchio_torque, loads);
        self.dynibo_joint_order(&acceleration)
    }

    pub fn aba_with_link_load(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        torque: &[f64],
        load: Wrench,
    ) -> Vec<f64> {
        assert_eq!(torque.len(), self.joint_mappings.len());
        let mut pinocchio_torque = vec![0.0; self.velocity_size];
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(index) = mapping.velocity_index {
                pinocchio_torque[index] = torque[joint];
            }
        }
        let load = [
            load.torque.x,
            load.torque.y,
            load.torque.z,
            load.force.x,
            load.force.y,
            load.force.z,
        ];
        let mut pinocchio_acceleration = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the context dimensions.
        unsafe {
            dynibo_pinocchio_aba_with_link_load_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio_torque.as_ptr(),
                load.as_ptr(),
                pinocchio_acceleration.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio_acceleration)
    }

    pub fn floating_aba(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        generalized_forces: &[f64],
        base: &Frame,
        base_velocity: Twist,
    ) -> Vec<f64> {
        assert_eq!(generalized_forces.len(), 6 + self.joint_mappings.len());
        let mut pinocchio_force = vec![0.0; self.velocity_size];
        let world_to_base = base.rotation.inverse();
        let local_torque = world_to_base * Vector3::from_column_slice(&generalized_forces[..3]);
        let local_force = world_to_base * Vector3::from_column_slice(&generalized_forces[3..6]);
        pinocchio_force[..3].copy_from_slice(local_force.as_slice());
        pinocchio_force[3..6].copy_from_slice(local_torque.as_slice());
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(index) = mapping.velocity_index {
                pinocchio_force[index] = generalized_forces[6 + joint];
            }
        }
        let mut pinocchio_acceleration = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the free-flyer context dimensions.
        unsafe {
            dynibo_pinocchio_aba_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio_force.as_ptr(),
                pinocchio_acceleration.as_mut_ptr(),
            )
        };

        self.floating_acceleration_order(&pinocchio_acceleration, base, base_velocity)
    }

    pub fn floating_aba_with_loads(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        generalized_forces: &[f64],
        base: &Frame,
        base_velocity: Twist,
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        assert_eq!(generalized_forces.len(), 6 + self.joint_mappings.len());
        let mut pinocchio_force = vec![0.0; self.velocity_size];
        let world_to_base = base.rotation.inverse();
        let local_torque = world_to_base * Vector3::from_column_slice(&generalized_forces[..3]);
        let local_force = world_to_base * Vector3::from_column_slice(&generalized_forces[3..6]);
        pinocchio_force[..3].copy_from_slice(local_force.as_slice());
        pinocchio_force[3..6].copy_from_slice(local_torque.as_slice());
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            pinocchio_force[mapping.velocity_index.expect("active joint")] =
                generalized_forces[6 + joint];
        }
        let acceleration =
            self.aba_with_loads_raw(configuration, velocity, &pinocchio_force, loads);
        self.floating_acceleration_order(&acceleration, base, base_velocity)
    }

    fn floating_acceleration_order(
        &self,
        pinocchio_acceleration: &[f64],
        base: &Frame,
        base_velocity: Twist,
    ) -> Vec<f64> {
        let world_to_base = base.rotation.inverse();
        let local_linear_velocity = world_to_base * base_velocity.linear;
        let local_angular_velocity = world_to_base * base_velocity.angular;
        let local_linear_acceleration = Vector3::from_column_slice(&pinocchio_acceleration[..3]);
        let local_angular_acceleration = Vector3::from_column_slice(&pinocchio_acceleration[3..6]);
        let world_angular_acceleration = base.rotation * local_angular_acceleration;
        let world_linear_acceleration = base.rotation
            * (local_linear_acceleration + local_angular_velocity.cross(&local_linear_velocity));
        let mut output = vec![0.0; 6 + self.joint_mappings.len()];
        output[..3].copy_from_slice(world_angular_acceleration.as_slice());
        output[3..6].copy_from_slice(world_linear_acceleration.as_slice());
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            output[6 + joint] =
                pinocchio_acceleration[mapping.velocity_index.expect("active joint")];
        }
        output
    }

    pub fn mass_matrix(&mut self, configuration: &[f64]) -> Vec<f64> {
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

    pub fn floating_mass_matrix(&mut self, configuration: &[f64], base: &Frame) -> Vec<f64> {
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

    pub fn floating_gravity(&mut self, configuration: &[f64], base: &Frame) -> Vec<f64> {
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

    pub fn coriolis_matrix(&mut self, configuration: &[f64], velocity: &[f64]) -> Vec<f64> {
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

    pub fn rnea_with_link_load(
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

    pub fn floating_rnea(
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

    pub fn floating_rnea_with_loads(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        base: &Frame,
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let pinocchio = self.rnea_with_loads_raw(configuration, velocity, acceleration, loads);
        self.floating_generalized_order(&pinocchio, base)
    }

    pub fn floating_gravity_with_loads(
        &mut self,
        configuration: &[f64],
        base: &Frame,
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let zero = vec![0.0; self.velocity_size];
        self.floating_rnea_with_loads(configuration, &zero, &zero, base, loads)
    }

    pub fn floating_coriolis_from_rnea(
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
