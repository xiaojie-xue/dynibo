#![cfg(feature = "pinocchio-bench")]

use std::{ffi::CString, path::PathBuf, ptr::NonNull};

use approx::assert_relative_eq;
use dyno::{Frame, Robot, Twist};
use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

unsafe extern "C" {
    fn dyno_pinocchio_create_for_joint(
        urdf_path: *const std::ffi::c_char,
        end_joint_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
    fn dyno_pinocchio_destroy(context: *mut std::ffi::c_void);
    fn dyno_pinocchio_dof(context: *const std::ffi::c_void) -> usize;
    fn dyno_pinocchio_joint_velocity_index(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    fn dyno_pinocchio_frame_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        rotation: *mut f64,
        translation: *mut f64,
    );
    fn dyno_pinocchio_jacobian_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        jacobian: *mut f64,
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
}

struct PinocchioContext(NonNull<std::ffi::c_void>);

impl PinocchioContext {
    fn new(urdf_path: &std::path::Path, end_joint_name: &str) -> Self {
        let path = CString::new(urdf_path.to_string_lossy().as_bytes()).unwrap();
        let joint_name = CString::new(end_joint_name).unwrap();
        // SAFETY: both C strings remain valid for the duration of the call.
        let context =
            unsafe { dyno_pinocchio_create_for_joint(path.as_ptr(), joint_name.as_ptr()) };
        Self(NonNull::new(context).expect("Pinocchio must load the tree fixture"))
    }

    fn dof(&self) -> usize {
        // SAFETY: the context remains alive while owned by `self`.
        unsafe { dyno_pinocchio_dof(self.0.as_ptr()) }
    }

    fn joint_index(&self, name: &str) -> usize {
        let name = CString::new(name).unwrap();
        // SAFETY: the context and C string are valid for the duration of the call.
        unsafe { dyno_pinocchio_joint_velocity_index(self.0.as_ptr(), name.as_ptr()) }
    }
}

impl Drop for PinocchioContext {
    fn drop(&mut self) {
        // SAFETY: the context is owned here and destroyed exactly once.
        unsafe { dyno_pinocchio_destroy(self.0.as_ptr()) };
    }
}

fn tree_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("benches/data/test_tree_7.urdf")
}

fn joint_mapping(arm: &Robot, pinocchio: &PinocchioContext) -> [usize; 7] {
    assert_eq!(arm.joint_count(), 7);
    assert_eq!(pinocchio.dof(), 7);
    std::array::from_fn(|index| pinocchio.joint_index(arm.joints()[index].name()))
}

fn pinocchio_order(values: &[f64; 7], mapping: &[usize; 7]) -> [f64; 7] {
    let mut result = [0.0; 7];
    for dyno_index in 0..7 {
        result[mapping[dyno_index]] = values[dyno_index];
    }
    result
}

fn dyno_order(values: &[f64; 7], mapping: &[usize; 7]) -> SVector<f64, 7> {
    SVector::from_fn(|index, _| values[mapping[index]])
}

fn deterministic_state(sample: usize, phase: f64, amplitude: f64) -> [f64; 7] {
    std::array::from_fn(|joint| {
        let argument = (sample + 1) as f64 * (joint + 3) as f64 * 0.731 + phase;
        amplitude * argument.sin()
    })
}

#[test]
fn branched_fk_and_jacobian_match_pinocchio() {
    let path = tree_path();
    let arm = Robot::from_urdf(&path).unwrap();
    for (link_name, joint_name) in [("left_tool", "left_wrist"), ("right_tool", "right_wrist")] {
        let target = arm.link_id(link_name).unwrap();
        let mut workspace = arm.workspace();
        let pinocchio = PinocchioContext::new(&path, joint_name);
        let mapping = joint_mapping(&arm, &pinocchio);

        for sample in 0..32 {
            let q = deterministic_state(sample, 0.0, 0.9);
            let pin_q = pinocchio_order(&q, &mapping);
            let mut rotation = [0.0; 9];
            let mut translation = [0.0; 3];
            // SAFETY: all buffers have the dimensions required by the seven-DoF context.
            unsafe {
                dyno_pinocchio_frame_values(
                    pinocchio.0.as_ptr(),
                    pin_q.as_ptr(),
                    rotation.as_mut_ptr(),
                    translation.as_mut_ptr(),
                )
            };

            let frame = arm.forward_kinematics(&q, target, &mut workspace).unwrap();
            assert_relative_eq!(
                frame.rotation.to_rotation_matrix().matrix(),
                &Matrix3::from_column_slice(&rotation),
                epsilon = 2.0e-12
            );
            assert_relative_eq!(
                frame.translation.vector,
                Vector3::from_column_slice(&translation),
                epsilon = 2.0e-12
            );

            let mut pin_jacobian = [0.0; 42];
            // SAFETY: the output buffer contains 6 * 7 elements.
            unsafe {
                dyno_pinocchio_jacobian_values(
                    pinocchio.0.as_ptr(),
                    pin_q.as_ptr(),
                    pin_jacobian.as_mut_ptr(),
                )
            };
            let pin_jacobian = SMatrix::<f64, 6, 7>::from_column_slice(&pin_jacobian);
            let expected = SMatrix::<f64, 6, 7>::from_fn(|row, dyno_column| {
                let pin_row = if row < 3 { row + 3 } else { row - 3 };
                pin_jacobian[(pin_row, mapping[dyno_column])]
            });
            let mut jacobian = [0.0; 42];
            arm.jacobian(&q, target, &mut workspace, &mut jacobian)
                .unwrap();
            assert_relative_eq!(
                SMatrix::<f64, 6, 7>::from_column_slice(&jacobian),
                expected,
                epsilon = 2.0e-12
            );
        }
    }
}

#[test]
fn branched_gravity_and_rnea_match_pinocchio() {
    let path = tree_path();
    let arm = Robot::from_urdf(&path).unwrap();
    let pinocchio = PinocchioContext::new(&path, "right_wrist");
    let mapping = joint_mapping(&arm, &pinocchio);
    let mut workspace = arm.workspace();

    for sample in 0..32 {
        let q = deterministic_state(sample, 0.0, 0.9);
        let pin_q = pinocchio_order(&q, &mapping);
        let mut pin_gravity = [0.0; 7];
        // SAFETY: the output buffer has one element per velocity coordinate.
        unsafe {
            dyno_pinocchio_gravity_values(
                pinocchio.0.as_ptr(),
                pin_q.as_ptr(),
                pin_gravity.as_mut_ptr(),
            )
        };
        let mut gravity = [0.0; 7];
        arm.gravity(&q, &Frame::identity(), &[], &mut workspace, &mut gravity)
            .unwrap();
        assert_relative_eq!(
            SVector::<f64, 7>::from(gravity),
            dyno_order(&pin_gravity, &mapping),
            epsilon = 2.0e-11
        );

        let qd = deterministic_state(sample, 0.7, 0.8);
        let qdd = deterministic_state(sample, 1.3, 1.1);
        let pin_qd = pinocchio_order(&qd, &mapping);
        let pin_qdd = pinocchio_order(&qdd, &mapping);
        let mut pin_torque = [0.0; 7];
        // SAFETY: all input and output buffers contain seven scalar coordinates.
        unsafe {
            dyno_pinocchio_rnea_values(
                pinocchio.0.as_ptr(),
                pin_q.as_ptr(),
                pin_qd.as_ptr(),
                pin_qdd.as_ptr(),
                pin_torque.as_mut_ptr(),
            )
        };
        let mut torque = [0.0; 7];
        arm.inverse_dynamics(
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
        assert_relative_eq!(
            SVector::<f64, 7>::from(torque),
            dyno_order(&pin_torque, &mapping),
            epsilon = 2.0e-11
        );
    }
}
