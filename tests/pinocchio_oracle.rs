#![cfg(feature = "pinocchio-tests")]

use std::{ffi::CString, path::PathBuf, ptr::NonNull};

use dyno::{Frame, IndexedLoad, InverseKinematicsOptions, Robot, Twist, Wrench};
use nalgebra::{Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};

unsafe extern "C" {
    fn dyno_pinocchio_create_for_frame(
        urdf_path: *const std::ffi::c_char,
        frame_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
    fn dyno_pinocchio_create_floating_for_frame(
        urdf_path: *const std::ffi::c_char,
        frame_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
    fn dyno_pinocchio_destroy(context: *mut std::ffi::c_void);
    fn dyno_pinocchio_dof(context: *const std::ffi::c_void) -> usize;
    fn dyno_pinocchio_configuration_size(context: *const std::ffi::c_void) -> usize;
    fn dyno_pinocchio_neutral_configuration(context: *const std::ffi::c_void, q: *mut f64);
    fn dyno_pinocchio_joint_configuration_index(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    fn dyno_pinocchio_joint_configuration_dimension(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    fn dyno_pinocchio_joint_velocity_index(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    fn dyno_pinocchio_link_frame_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        rotation: *mut f64,
        translation: *mut f64,
    );
    fn dyno_pinocchio_link_jacobian_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        jacobian: *mut f64,
    );
    fn dyno_pinocchio_link_velocity_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        velocity: *mut f64,
    );
    fn dyno_pinocchio_link_acceleration_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        acceleration: *mut f64,
    );
    fn dyno_pinocchio_gravity_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        gravity: *mut f64,
    );
    fn dyno_pinocchio_rnea_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        torque: *mut f64,
    );
    fn dyno_pinocchio_rnea_with_link_load_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        load: *const f64,
        torque: *mut f64,
    );
    fn dyno_pinocchio_floating_rnea_values(
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
            unsafe { dyno_pinocchio_create_for_frame(path.as_ptr(), frame_name.as_ptr()) };
        let pointer = NonNull::new(pointer).expect("Pinocchio must load the oracle fixture");
        Self::from_pointer(robot, pointer)
    }

    fn new_floating(robot: &Robot, path: &std::path::Path, frame_name: &str) -> Self {
        let path = CString::new(path.to_string_lossy().as_bytes()).unwrap();
        let frame_name = CString::new(frame_name).unwrap();
        // SAFETY: both C strings remain alive for the duration of the call.
        let pointer =
            unsafe { dyno_pinocchio_create_floating_for_frame(path.as_ptr(), frame_name.as_ptr()) };
        let pointer = NonNull::new(pointer).expect("Pinocchio must load the floating fixture");
        Self::from_pointer(robot, pointer)
    }

    fn from_pointer(robot: &Robot, pointer: NonNull<std::ffi::c_void>) -> Self {
        // SAFETY: `pointer` owns a live Pinocchio context.
        let configuration_size = unsafe { dyno_pinocchio_configuration_size(pointer.as_ptr()) };
        // SAFETY: `pointer` owns a live Pinocchio context.
        let velocity_size = unsafe { dyno_pinocchio_dof(pointer.as_ptr()) };
        let joint_mappings = robot
            .joints()
            .iter()
            .map(|joint| {
                let name = CString::new(joint.name()).unwrap();
                // SAFETY: the context and name are valid for each query.
                let configuration_index = unsafe {
                    dyno_pinocchio_joint_configuration_index(pointer.as_ptr(), name.as_ptr())
                };
                // SAFETY: the context and name are valid for each query.
                let configuration_dimension = unsafe {
                    dyno_pinocchio_joint_configuration_dimension(pointer.as_ptr(), name.as_ptr())
                };
                // SAFETY: the context and name are valid for each query.
                let velocity_index =
                    unsafe { dyno_pinocchio_joint_velocity_index(pointer.as_ptr(), name.as_ptr()) };
                JointMapping {
                    configuration_index,
                    configuration_dimension,
                    velocity_index: (velocity_index < velocity_size).then_some(velocity_index),
                }
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
            dyno_pinocchio_neutral_configuration(self.pointer.as_ptr(), configuration.as_mut_ptr())
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

    fn frame(&mut self, configuration: &[f64]) -> (Matrix3<f64>, Vector3<f64>) {
        let mut rotation = [0.0; 9];
        let mut translation = [0.0; 3];
        // SAFETY: all buffers have the dimensions required by this context.
        unsafe {
            dyno_pinocchio_link_frame_values(
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
            dyno_pinocchio_link_jacobian_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        let mut dyno_order = vec![0.0; 6 * self.joint_mappings.len()];
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(column) = mapping.velocity_index {
                for row in 0..6 {
                    let pinocchio_row = if row < 3 { row + 3 } else { row - 3 };
                    dyno_order[6 * joint + row] = pinocchio[6 * column + pinocchio_row];
                }
            }
        }
        dyno_order
    }

    fn velocity(&mut self, configuration: &[f64], velocity: &[f64]) -> [f64; 6] {
        let mut output = [0.0; 6];
        // SAFETY: input and output sizes match the context dimensions.
        unsafe {
            dyno_pinocchio_link_velocity_values(
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
            dyno_pinocchio_link_acceleration_values(
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
            dyno_pinocchio_gravity_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dyno_joint_order(&pinocchio)
    }

    fn rnea(&mut self, configuration: &[f64], velocity: &[f64], acceleration: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the context dimensions.
        unsafe {
            dyno_pinocchio_rnea_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dyno_joint_order(&pinocchio)
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
            dyno_pinocchio_rnea_with_link_load_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                load.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dyno_joint_order(&pinocchio)
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
            dyno_pinocchio_floating_rnea_values(
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
        self.dyno_joint_order(&pinocchio)
    }

    fn dyno_joint_order(&self, pinocchio: &[f64]) -> Vec<f64> {
        self.joint_mappings
            .iter()
            .map(|mapping| mapping.velocity_index.map_or(0.0, |index| pinocchio[index]))
            .collect()
    }
}

impl Drop for PinocchioContext {
    fn drop(&mut self) {
        // SAFETY: this context is owned here and destroyed exactly once.
        unsafe { dyno_pinocchio_destroy(self.pointer.as_ptr()) };
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
    let robot = Robot::from_urdf(&path).unwrap();
    let target = robot.link_id("test_link_4").unwrap();
    let mut workspace = robot.workspace();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let q = [0.2, 1.0, -0.7, 0.4];
    let qd = [-0.3, 0.5, -0.2, 0.8];
    let qdd = [0.7, -0.4, 0.1, 0.3];
    let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);

    let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
    let actual_frame = robot
        .forward_kinematics(&q, target, &mut workspace)
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
        .jacobian(&q, target, &mut workspace, &mut actual_jacobian)
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
                &q,
                &qd,
                target,
                &Frame::identity(),
                &Frame::identity(),
                &mut workspace,
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
            .forward_acceleration_kinematics(&q, &qd, &qdd, target, &mut workspace)
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
        .gravity(
            &q,
            &Frame::identity(),
            &[],
            &mut workspace,
            &mut actual_gravity,
        )
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
            &q,
            &qd,
            &qdd,
            &Frame::identity(),
            Twist::zeros(),
            Twist::zeros(),
            &[],
            &mut workspace,
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
    let robot = Robot::from_urdf(&path).unwrap();
    assert_eq!(robot.joint_count(), 4);
    let mut workspace = robot.workspace();

    for link_name in ["base", "link_a", "mounted_link", "slider_link", "tool"] {
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..32 {
            let (q, qd, qdd) = deterministic_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
            let actual_frame = robot
                .forward_kinematics(&q, target, &mut workspace)
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
                .jacobian(&q, target, &mut workspace, &mut actual_jacobian)
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
                    &q,
                    &qd,
                    target,
                    &Frame::identity(),
                    &Frame::identity(),
                    &mut workspace,
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
                .forward_acceleration_kinematics(&q, &qd, &qdd, target, &mut workspace)
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
fn mixed_joint_gravity_and_rnea_match_pinocchio() {
    let path = fixture();
    let robot = Robot::from_urdf(&path).unwrap();
    let mut workspace = robot.workspace();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");

    for sample in 0..64 {
        let (q, qd, qdd) = deterministic_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let mut actual_gravity = [f64::NAN; 4];
        robot
            .gravity(
                &q,
                &Frame::identity(),
                &[],
                &mut workspace,
                &mut actual_gravity,
            )
            .unwrap();
        let expected_gravity = pinocchio.gravity(&pin_q);
        assert_close(
            &actual_gravity,
            &expected_gravity,
            1.0e-9,
            1.0e-10,
            &format!("gravity sample {sample}"),
        );

        let mut actual_torque = [f64::NAN; 4];
        robot
            .inverse_dynamics(
                &q,
                &qd,
                &qdd,
                &Frame::identity(),
                Twist::zeros(),
                Twist::zeros(),
                &[],
                &mut workspace,
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
fn mixed_joint_external_loads_match_pinocchio() {
    let path = fixture();
    let robot = Robot::from_urdf(&path).unwrap();
    let mut workspace = robot.workspace();
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
            let (q, qd, qdd) = deterministic_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let mut actual = [f64::NAN; 4];
            robot
                .inverse_dynamics(
                    &q,
                    &qd,
                    &qdd,
                    &Frame::identity(),
                    Twist::zeros(),
                    Twist::zeros(),
                    &[indexed_load],
                    &mut workspace,
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
    let robot = Robot::from_urdf(&path).unwrap();
    let mut workspace = robot.workspace();

    for link in robot.links() {
        let link_name = link.name();
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..16 {
            let (q, qd, qdd) = deterministic_tree_state(sample);
            let (pin_q, _, _) = pinocchio.state(&q, &qd, &qdd);
            let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
            let actual_frame = robot
                .forward_kinematics(&q, target, &mut workspace)
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
                .jacobian(&q, target, &mut workspace, &mut actual_jacobian)
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
    let robot = Robot::from_urdf(&path).unwrap();
    let mut workspace = robot.workspace();

    for link_name in ["left_tool", "right_tool"] {
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..32 {
            let (q, qd, qdd) = deterministic_tree_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let velocity = robot
                .forward_velocity_kinematics(
                    &q,
                    &qd,
                    target,
                    &Frame::identity(),
                    &Frame::identity(),
                    &mut workspace,
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
                .forward_acceleration_kinematics(&q, &qd, &qdd, target, &mut workspace)
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
    let robot = Robot::from_urdf(&path).unwrap();
    let mut workspace = robot.workspace();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "right_tool");

    for sample in 0..32 {
        let (q, qd, qdd) = deterministic_tree_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let mut gravity = [f64::NAN; 7];
        robot
            .gravity(&q, &Frame::identity(), &[], &mut workspace, &mut gravity)
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
            .inverse_dynamics(
                &q,
                &qd,
                &qdd,
                &Frame::identity(),
                Twist::zeros(),
                Twist::zeros(),
                &[],
                &mut workspace,
                &mut torque,
            )
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
    let robot = Robot::from_urdf(&path).unwrap();
    let mut workspace = robot.workspace();
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
                    &q,
                    &qd,
                    &qdd,
                    &Frame::identity(),
                    Twist::zeros(),
                    Twist::zeros(),
                    &[indexed_load],
                    &mut workspace,
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
    let robot = Robot::from_urdf(&path).unwrap();
    let target = robot.link_id("tool").unwrap();
    let mut workspace = robot.workspace();
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
            0.0,
            0.22 * (phase * 0.31).sin(),
            1.7 * (phase * 0.59).sin(),
        ];
        let zero = [0.0; 4];
        let (pin_target_q, _, _) = pinocchio.state(&target_q, &zero, &zero);
        let (rotation, translation) = pinocchio.frame(&pin_target_q);
        let desired = Frame::from_parts(
            Translation3::from(translation),
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation)),
        );
        let initial = [
            target_q[0] + 0.12 * (phase * 0.73).sin(),
            0.0,
            target_q[2] + 0.05 * (phase * 0.41).cos(),
            target_q[3] - 0.15 * (phase * 0.37).sin(),
        ];
        let mut solution = [f64::NAN; 4];
        robot
            .inverse_kinematics(
                &initial,
                target,
                &desired,
                options,
                &mut workspace,
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
        assert_eq!(solution[1], 0.0, "fixed coordinate changed");
    }
}

#[test]
fn mixed_joint_moving_base_rnea_matches_free_flyer_pinocchio() {
    let path = fixture();
    let robot = Robot::from_urdf(&path).unwrap();
    let mut workspace = robot.workspace();
    let mut pinocchio = PinocchioContext::new_floating(&robot, &path, "tool");

    for sample in 0..16 {
        let (q, qd, qdd) = deterministic_state(sample);
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
        let mut actual = [f64::NAN; 4];
        robot
            .inverse_dynamics(
                &q,
                &qd,
                &qdd,
                &base,
                base_velocity,
                base_acceleration,
                &[],
                &mut workspace,
                &mut actual,
            )
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
