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
