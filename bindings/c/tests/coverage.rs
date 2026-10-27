use std::{ffi::CString, path::PathBuf, ptr};

use dynibo_c::*;

struct Handles {
    fixed: *mut DyniboRobot,
    fixed_workspace: *mut DyniboWorkspace,
    floating: *mut DyniboFloatingRobot,
    floating_workspace: *mut DyniboFloatingWorkspace,
    target: usize,
    floating_target: usize,
}

impl Handles {
    fn new() -> Self {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("tests/data/test_arm.urdf");
        let path = CString::new(path.to_string_lossy().as_bytes()).unwrap();
        let mut handles = Self {
            fixed: ptr::null_mut(),
            fixed_workspace: ptr::null_mut(),
            floating: ptr::null_mut(),
            floating_workspace: ptr::null_mut(),
            target: 0,
            floating_target: 0,
        };
        unsafe {
            assert_eq!(
                dynibo_robot_from_urdf(path.as_ptr(), &mut handles.fixed),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_workspace_create(handles.fixed, &mut handles.fixed_workspace),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_robot_link_id(handles.fixed, c"test_link_4".as_ptr(), &mut handles.target,),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_floating_robot_from_urdf(path.as_ptr(), &mut handles.floating),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_floating_workspace_create(handles.floating, &mut handles.floating_workspace,),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_floating_robot_link_id(
                    handles.floating,
                    c"test_link_4".as_ptr(),
                    &mut handles.floating_target,
                ),
                DyniboStatus::Ok
            );
        }
        handles
    }
}

impl Drop for Handles {
    fn drop(&mut self) {
        unsafe {
            dynibo_workspace_destroy(self.fixed_workspace);
            dynibo_robot_destroy(self.fixed);
            dynibo_floating_workspace_destroy(self.floating_workspace);
            dynibo_floating_robot_destroy(self.floating);
        }
    }
}

#[test]
fn finite_overflow_reports_solver_error_and_workspace_recovers() {
    let handles = Handles::new();
    let base = DyniboBaseState::default();
    let q = [0.0; 4];
    let huge_velocity = [1.0e308; 4];
    for floating in [false, true] {
        let count = unsafe {
            if floating {
                dynibo_floating_robot_generalized_count(handles.floating)
            } else {
                dynibo_robot_generalized_count(handles.fixed)
            }
        };
        let mut output = vec![0.0; count];
        let calculate = |qd: &[f64], output: &mut [f64]| unsafe {
            if floating {
                dynibo_floating_velocity_product_forces(
                    handles.floating,
                    handles.floating_workspace,
                    &base,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    output.as_mut_ptr(),
                    output.len(),
                )
            } else {
                dynibo_velocity_product_forces(
                    handles.fixed,
                    handles.fixed_workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    output.as_mut_ptr(),
                    output.len(),
                )
            }
        };
        assert_eq!(
            calculate(&huge_velocity, &mut output),
            DyniboStatus::SolverError
        );
        assert!(last_error().contains("numerical"));
        assert_eq!(calculate(&q, &mut output), DyniboStatus::Ok);
        assert!(output.iter().all(|value| *value == 0.0));
        assert!(last_error().is_empty());
    }
}

#[test]
fn metadata_and_factory_error_paths_are_reported() {
    let handles = Handles::new();
    unsafe {
        assert!(dynibo_robot_name(ptr::null()).is_null());
        assert!(dynibo_floating_robot_name(ptr::null()).is_null());
        assert_eq!(dynibo_robot_joint_count(ptr::null()), 0);
        assert_eq!(dynibo_robot_generalized_count(ptr::null()), 0);
        assert_eq!(dynibo_robot_link_count(ptr::null()), 0);
        assert_eq!(dynibo_floating_robot_joint_count(ptr::null()), 0);
        assert_eq!(dynibo_floating_robot_generalized_count(ptr::null()), 0);
        assert_eq!(dynibo_floating_robot_link_count(ptr::null()), 0);

        let mut link = 0;
        assert_eq!(
            dynibo_robot_link_id(handles.fixed, ptr::null(), &mut link),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_robot_link_id(handles.fixed, c"missing".as_ptr(), &mut link),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_robot_link_id(handles.fixed, c"test_link_4".as_ptr(), ptr::null_mut()),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_robot_link_id(handles.floating, ptr::null(), &mut link),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_robot_link_id(handles.floating, c"missing".as_ptr(), &mut link),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_robot_link_id(
                handles.floating,
                c"test_link_4".as_ptr(),
                ptr::null_mut(),
            ),
            DyniboStatus::InvalidArgument
        );

        assert_eq!(
            dynibo_workspace_create(ptr::null(), ptr::null_mut()),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_workspace_create(handles.fixed, ptr::null_mut()),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_workspace_create(ptr::null(), ptr::null_mut()),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_workspace_create(handles.floating, ptr::null_mut()),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_robot_set_base_frame(handles.fixed, ptr::null()),
            DyniboStatus::InvalidArgument
        );
    }
}

#[test]
fn calculation_pointer_and_length_errors_are_reported() {
    let handles = Handles::new();
    let q = [0.0; 4];
    let qd = [0.0; 4];
    let qdd = [0.0; 4];
    let forces = [0.0; 4];
    let floating_forces = [0.0; 10];
    let base = DyniboBaseState::default();
    let tool = DyniboPose::default();
    let mut pose = DyniboPose::default();
    let mut twist = DyniboTwist::default();
    let mut fixed = [0.0; 24];
    let mut floating = [0.0; 100];

    unsafe {
        assert_eq!(
            dynibo_forward_kinematics(
                handles.fixed,
                handles.fixed_workspace,
                ptr::null(),
                q.len(),
                handles.target,
                &mut pose,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_forward_kinematics(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                handles.floating_target,
                ptr::null_mut(),
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_jacobian(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.len(),
                handles.target,
                ptr::null_mut(),
                fixed.len(),
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_jacobian(
                handles.floating,
                handles.floating_workspace,
                ptr::null(),
                q.as_ptr(),
                q.len(),
                handles.floating_target,
                floating.as_mut_ptr(),
                60,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_jacobian_derivative(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                ptr::null(),
                q.len(),
                handles.target,
                fixed.as_mut_ptr(),
                fixed.len(),
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_jacobian_derivative(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                ptr::null(),
                q.len(),
                handles.floating_target,
                floating.as_mut_ptr(),
                60,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_mass_matrix(
                handles.fixed,
                handles.fixed_workspace,
                ptr::null(),
                q.len(),
                fixed.as_mut_ptr(),
                16,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_mass_matrix(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                ptr::null_mut(),
                100,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_velocity_product_forces(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                ptr::null(),
                q.len(),
                fixed.as_mut_ptr(),
                q.len(),
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_velocity_product_forces(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                ptr::null(),
                q.len(),
                floating.as_mut_ptr(),
                10,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_forward_velocity_kinematics(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                handles.target,
                ptr::null(),
                &mut twist,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_forward_velocity_kinematics(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                ptr::null(),
                q.len(),
                handles.floating_target,
                &tool,
                &mut twist,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_forward_acceleration_kinematics(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                qd.as_ptr(),
                ptr::null(),
                q.len(),
                handles.target,
                &mut twist,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_forward_acceleration_kinematics(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                ptr::null(),
                q.len(),
                handles.floating_target,
                &mut twist,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_gravity(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.len(),
                ptr::null(),
                1,
                fixed.as_mut_ptr(),
                q.len(),
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_gravity(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                ptr::null(),
                1,
                floating.as_mut_ptr(),
                10,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_inverse_dynamics(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                ptr::null(),
                qdd.as_ptr(),
                q.len(),
                ptr::null(),
                0,
                fixed.as_mut_ptr(),
                q.len(),
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_inverse_dynamics(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                ptr::null(),
                q.len(),
                ptr::null(),
                0,
                floating.as_mut_ptr(),
                10,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_forward_dynamics(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                ptr::null(),
                forces.len(),
                ptr::null(),
                0,
                fixed.as_mut_ptr(),
                q.len(),
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_forward_dynamics(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                ptr::null(),
                floating_forces.len(),
                ptr::null(),
                0,
                floating.as_mut_ptr(),
                10,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_inverse_kinematics(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.len(),
                handles.target,
                ptr::null(),
                DyniboIkOptions::default(),
                fixed.as_mut_ptr(),
                q.len(),
            ),
            DyniboStatus::InvalidArgument
        );
    }
}

#[test]
fn non_finite_joint_inputs_are_rejected() {
    let handles = Handles::new();
    let mut q = [0.0; 4];
    let mut pose = DyniboPose::default();

    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        q[0] = invalid;
        assert_eq!(
            unsafe {
                dynibo_forward_kinematics(
                    handles.fixed,
                    handles.fixed_workspace,
                    q.as_ptr(),
                    q.len(),
                    handles.target,
                    &mut pose,
                )
            },
            DyniboStatus::InvalidArgument
        );
    }
}

#[test]
fn zero_length_buffers_are_rejected_by_model_validation_without_dereferencing() {
    let handles = Handles::new();
    let base = DyniboBaseState::default();

    unsafe {
        assert_eq!(
            dynibo_mass_matrix(
                handles.fixed,
                handles.fixed_workspace,
                ptr::null(),
                0,
                ptr::null_mut(),
                0,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_mass_matrix(
                handles.floating,
                handles.floating_workspace,
                &base,
                ptr::null(),
                0,
                ptr::null_mut(),
                0,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_velocity_product_forces(
                handles.fixed,
                handles.fixed_workspace,
                ptr::null(),
                ptr::null(),
                0,
                ptr::null_mut(),
                0,
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_velocity_product_forces(
                handles.floating,
                handles.floating_workspace,
                &base,
                ptr::null(),
                ptr::null(),
                0,
                ptr::null_mut(),
                0,
            ),
            DyniboStatus::InvalidArgument
        );
    }
}

#[test]
fn oversized_buffer_ranges_are_rejected_before_slice_construction() {
    let handles = Handles::new();
    let q = [0.0; 4];
    let mut output = [0.0; 16];

    unsafe {
        assert_eq!(
            dynibo_mass_matrix(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                usize::MAX,
                output.as_mut_ptr(),
                output.len(),
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_mass_matrix(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.len(),
                output.as_mut_ptr(),
                usize::MAX,
            ),
            DyniboStatus::InvalidArgument
        );

        // The fabricated address is never dereferenced: range overflow is
        // detected by reject_byte_overlap before any slice is constructed.
        let overflowing = ptr::without_provenance::<f64>(usize::MAX - 3);
        assert_eq!(
            dynibo_mass_matrix(
                handles.fixed,
                handles.fixed_workspace,
                overflowing,
                1,
                output.as_mut_ptr(),
                output.len(),
            ),
            DyniboStatus::InvalidArgument
        );
    }
}

#[test]
fn invalid_external_load_ids_are_rejected_and_do_not_poison_the_workspace() {
    let handles = Handles::new();
    let q = [0.2, -0.3, 0.4, -0.1];
    let base = DyniboBaseState::default();
    let invalid = DyniboLoad {
        link_id: usize::MAX,
        torque: [0.1, 0.2, 0.3],
        force: [-0.4, 0.5, -0.6],
    };
    let mut fixed_output = [0.0; 4];
    let mut floating_output = [0.0; 10];

    unsafe {
        assert_eq!(
            dynibo_gravity(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.len(),
                &invalid,
                1,
                fixed_output.as_mut_ptr(),
                fixed_output.len(),
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_gravity(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                &invalid,
                1,
                floating_output.as_mut_ptr(),
                floating_output.len(),
            ),
            DyniboStatus::InvalidArgument
        );

        assert_eq!(
            dynibo_gravity(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.len(),
                ptr::null(),
                0,
                fixed_output.as_mut_ptr(),
                fixed_output.len(),
            ),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_floating_gravity(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                ptr::null(),
                0,
                floating_output.as_mut_ptr(),
                floating_output.len(),
            ),
            DyniboStatus::Ok
        );
        assert!(fixed_output.iter().all(|value| value.is_finite()));
        assert!(floating_output.iter().all(|value| value.is_finite()));
    }
}

fn last_error() -> String {
    unsafe { std::ffi::CStr::from_ptr(dynibo_last_error_message()) }
        .to_str()
        .unwrap()
        .to_owned()
}

#[test]
fn invalid_utf8_paths_and_link_names_report_errors_without_writing_results() {
    let handles = Handles::new();
    // A valid, NUL-terminated C string whose content is not UTF-8.
    let invalid = c"\xff";
    unsafe {
        let mut fixed = handles.fixed;
        let mut floating = handles.floating;
        assert_eq!(
            dynibo_robot_from_urdf(invalid.as_ptr(), &mut fixed),
            DyniboStatus::InvalidArgument
        );
        assert!(fixed.is_null());
        assert_eq!(last_error(), "path must be valid UTF-8");
        assert_eq!(
            dynibo_floating_robot_from_urdf(invalid.as_ptr(), &mut floating),
            DyniboStatus::InvalidArgument
        );
        assert!(floating.is_null());
        assert_eq!(last_error(), "path must be valid UTF-8");

        for is_floating in [false, true] {
            let mut target = usize::MAX;
            let status = if is_floating {
                dynibo_floating_robot_link_id(handles.floating, invalid.as_ptr(), &mut target)
            } else {
                dynibo_robot_link_id(handles.fixed, invalid.as_ptr(), &mut target)
            };
            assert_eq!(status, DyniboStatus::InvalidArgument);
            assert_eq!(last_error(), "name must be valid UTF-8");
            assert_eq!(target, usize::MAX);
            let status = if is_floating {
                dynibo_floating_robot_link_id(
                    handles.floating,
                    c"test_link_4".as_ptr(),
                    &mut target,
                )
            } else {
                dynibo_robot_link_id(handles.fixed, c"test_link_4".as_ptr(), &mut target)
            };
            assert_eq!(status, DyniboStatus::Ok);
            assert!(last_error().is_empty());
            assert_eq!(
                target,
                if is_floating {
                    handles.floating_target
                } else {
                    handles.target
                }
            );
        }
    }
}

#[test]
fn output_address_and_struct_input_length_overflow_are_rejected_before_slicing() {
    let handles = Handles::new();
    let q = [0.0; 4];
    let mut pose = DyniboPose::default();
    let mut twist = DyniboTwist::default();
    unsafe {
        // The fabricated output is rejected by address arithmetic, before it
        // could be dereferenced or turned into a mutable slice.
        let output = ptr::without_provenance_mut::<f64>(usize::MAX - 3);
        assert_eq!(
            dynibo_mass_matrix(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.len(),
                output,
                1
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(last_error(), "output address range overflows");
        assert_eq!(
            dynibo_forward_kinematics(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                usize::MAX,
                handles.target,
                &mut pose
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(last_error(), "q length is too large");
        assert_eq!(pose.translation, [0.0; 3]);
        assert_eq!(pose.rotation_xyzw, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(
            dynibo_forward_velocity_kinematics(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.as_ptr(),
                usize::MAX,
                handles.target,
                &DyniboPose::default(),
                &mut twist
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(last_error(), "q length is too large");
        assert_eq!(twist.angular, [0.0; 3]);
        assert_eq!(twist.linear, [0.0; 3]);
        assert_eq!(
            dynibo_forward_kinematics(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.len(),
                handles.target,
                &mut pose
            ),
            DyniboStatus::Ok
        );
        assert!(last_error().is_empty());
        assert!(pose.translation.iter().all(|v| v.is_finite()));
    }
}

#[test]
fn inverse_and_forward_dynamics_load_errors_preserve_outputs_and_recover() {
    let handles = Handles::new();
    let base = DyniboBaseState::default();
    let q = [0.2, -0.3, 0.4, -0.1];
    let qd = [0.1, 0.2, -0.1, -0.2];
    let qdd = [-0.2, 0.1, 0.3, -0.1];
    for floating in [false, true] {
        for inverse in [false, true] {
            let forces = vec![0.1; if floating { 10 } else { 4 }];
            let target = if floating {
                handles.floating_target
            } else {
                handles.target
            };
            let load = DyniboLoad {
                link_id: target,
                torque: [0.1, 0.2, -0.3],
                force: [-0.1, 0.3, 0.2],
            };
            let calculate = |loads: &[DyniboLoad], output: &mut [f64]| unsafe {
                match (floating, inverse) {
                    (false, false) => dynibo_forward_dynamics(
                        handles.fixed,
                        handles.fixed_workspace,
                        q.as_ptr(),
                        qd.as_ptr(),
                        q.len(),
                        forces.as_ptr(),
                        forces.len(),
                        loads.as_ptr(),
                        loads.len(),
                        output.as_mut_ptr(),
                        output.len(),
                    ),
                    (false, true) => dynibo_inverse_dynamics(
                        handles.fixed,
                        handles.fixed_workspace,
                        q.as_ptr(),
                        qd.as_ptr(),
                        qdd.as_ptr(),
                        q.len(),
                        loads.as_ptr(),
                        loads.len(),
                        output.as_mut_ptr(),
                        output.len(),
                    ),
                    (true, false) => dynibo_floating_forward_dynamics(
                        handles.floating,
                        handles.floating_workspace,
                        &base,
                        q.as_ptr(),
                        qd.as_ptr(),
                        q.len(),
                        forces.as_ptr(),
                        forces.len(),
                        loads.as_ptr(),
                        loads.len(),
                        output.as_mut_ptr(),
                        output.len(),
                    ),
                    (true, true) => dynibo_floating_inverse_dynamics(
                        handles.floating,
                        handles.floating_workspace,
                        &base,
                        q.as_ptr(),
                        qd.as_ptr(),
                        qdd.as_ptr(),
                        q.len(),
                        loads.as_ptr(),
                        loads.len(),
                        output.as_mut_ptr(),
                        output.len(),
                    ),
                }
            };
            let mut expected = vec![0.0; if floating { 10 } else { 4 }];
            assert_eq!(calculate(&[load], &mut expected), DyniboStatus::Ok);
            let invalid_id = DyniboLoad {
                link_id: usize::MAX,
                ..load
            };
            let overflow = DyniboLoad {
                torque: [f64::MAX; 3],
                ..load
            };
            let mut cases = vec![
                (
                    vec![load, invalid_id],
                    DyniboStatus::InvalidArgument,
                    "invalid link id",
                ),
                (
                    vec![overflow, overflow],
                    DyniboStatus::SolverError,
                    "load aggregation",
                ),
            ];
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                for torque in [false, true] {
                    let mut invalid = load;
                    if torque {
                        invalid.torque[1] = value;
                    } else {
                        invalid.force[2] = value;
                    }
                    cases.push((
                        vec![load, invalid],
                        DyniboStatus::InvalidArgument,
                        "load contains a non-finite value",
                    ));
                }
            }
            for (loads, status, message) in cases {
                let mut output = vec![123.0; expected.len()];
                assert_eq!(
                    calculate(&loads, &mut output),
                    status,
                    "floating={floating}, inverse={inverse}, {message}"
                );
                assert!(last_error().contains(message), "{}", last_error());
                assert!(output.iter().all(|v| *v == 123.0));
                assert_eq!(calculate(&[load], &mut output), DyniboStatus::Ok);
                assert!(last_error().is_empty());
                assert_eq!(output, expected);
            }
        }
    }
}

#[test]
fn factory_diagnostics_version_and_null_destruction_follow_the_c_contract() {
    unsafe {
        assert_eq!(
            std::ffi::CStr::from_ptr(dynibo_version()).to_str().unwrap(),
            env!("CARGO_PKG_VERSION")
        );
        dynibo_robot_destroy(ptr::null_mut());
        dynibo_workspace_destroy(ptr::null_mut());
        dynibo_floating_robot_destroy(ptr::null_mut());
        dynibo_floating_workspace_destroy(ptr::null_mut());
        let mut robot = ptr::null_mut();
        assert_eq!(
            dynibo_floating_robot_from_urdf(ptr::null(), &mut robot),
            DyniboStatus::InvalidArgument
        );
        assert!(robot.is_null());
        assert_eq!(last_error(), "path must not be null");
        // An existing non-URDF file exercises model parsing without a temporary fixture.
        let invalid_path =
            CString::new(concat!(env!("CARGO_MANIFEST_DIR"), "/Cargo.toml")).unwrap();
        assert_eq!(
            dynibo_floating_robot_from_urdf(invalid_path.as_ptr(), &mut robot),
            DyniboStatus::ModelError
        );
        assert!(robot.is_null());
        assert!(!last_error().is_empty());
        let handles = Handles::new();
        assert!(last_error().is_empty());
        assert!(dynibo_floating_robot_link_count(handles.floating) > 0);
    }
}

#[test]
fn floating_workspace_from_another_model_is_rejected_without_writing_output() {
    let handles = Handles::new();
    let other = Handles::new();
    let q = [0.0; 4];
    let base = DyniboBaseState::default();
    let mut output = [123.0; 100];
    unsafe {
        assert_eq!(
            dynibo_floating_mass_matrix(
                handles.floating,
                other.floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                output.as_mut_ptr(),
                output.len()
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(output, [123.0; 100]);
        assert_eq!(
            last_error(),
            "workspace does not belong to this robot model"
        );
        assert_eq!(
            dynibo_floating_mass_matrix(
                handles.floating,
                handles.floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                output.as_mut_ptr(),
                output.len()
            ),
            DyniboStatus::Ok
        );
        assert!(last_error().is_empty());
        let mut expected = [0.0; 100];
        assert_eq!(
            dynibo_floating_mass_matrix(
                other.floating,
                other.floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                expected.as_mut_ptr(),
                expected.len()
            ),
            DyniboStatus::Ok
        );
        assert_eq!(output, expected);
    }
}

#[test]
fn invalid_c_poses_preserve_the_base_frame_and_allow_recovery() {
    let handles = Handles::new();
    let q = [0.0; 4];
    let mut expected = DyniboPose::default();
    unsafe {
        let original = DyniboPose {
            translation: [0.3, -0.2, 0.5],
            ..Default::default()
        };
        assert_eq!(
            dynibo_robot_set_base_frame(handles.fixed, &original),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_forward_kinematics(
                handles.fixed,
                handles.fixed_workspace,
                q.as_ptr(),
                q.len(),
                handles.target,
                &mut expected
            ),
            DyniboStatus::Ok
        );
        let mut invalid = vec![DyniboPose {
            rotation_xyzw: [0.0; 4],
            ..Default::default()
        }];
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for index in 0..3 {
                let mut pose = DyniboPose::default();
                pose.translation[index] = value;
                invalid.push(pose);
            }
            for index in 0..4 {
                let mut pose = DyniboPose::default();
                pose.rotation_xyzw[index] = value;
                invalid.push(pose);
            }
        }
        for pose in invalid {
            assert_eq!(
                dynibo_robot_set_base_frame(handles.fixed, &pose),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                last_error(),
                "pose contains non-finite values or a zero quaternion"
            );
            let mut actual = DyniboPose::default();
            assert_eq!(
                dynibo_forward_kinematics(
                    handles.fixed,
                    handles.fixed_workspace,
                    q.as_ptr(),
                    q.len(),
                    handles.target,
                    &mut actual
                ),
                DyniboStatus::Ok
            );
            assert!(last_error().is_empty());
            assert_eq!(actual.translation, expected.translation);
            assert_eq!(actual.rotation_xyzw, expected.rotation_xyzw);
        }
    }
}
