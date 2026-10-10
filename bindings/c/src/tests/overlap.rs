use super::*;

#[test]
fn fixed_vector_calculations_reject_overlapping_output_before_slicing() {
    // SAFETY: Every pointer comes from a live allocation and the calls are
    // deliberately rejected before the declared output range is accessed.
    unsafe {
        let (robot, workspace, target) = fixed_handles();
        let mut q = [0.0; 4];
        let qd = [0.0; 4];
        let qdd = [0.0; 4];
        let forces = [0.0; 4];
        let overlap = q.as_mut_ptr();
        assert_eq!(
            dynibo_jacobian(robot, workspace, overlap, 4, target, overlap, 24),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_jacobian_derivative(
                robot,
                workspace,
                overlap,
                qd.as_ptr(),
                4,
                target,
                overlap,
                24
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_mass_matrix(robot, workspace, overlap, 4, overlap, 16),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_velocity_product_forces(robot, workspace, overlap, qd.as_ptr(), 4, overlap, 4),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_gravity(robot, workspace, overlap, 4, ptr::null(), 0, overlap, 4),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_inverse_dynamics(
                robot,
                workspace,
                overlap,
                qd.as_ptr(),
                qdd.as_ptr(),
                4,
                ptr::null(),
                0,
                overlap,
                4
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_forward_dynamics(
                robot,
                workspace,
                overlap,
                qd.as_ptr(),
                4,
                forces.as_ptr(),
                4,
                ptr::null(),
                0,
                overlap,
                4
            ),
            DyniboStatus::InvalidArgument
        );
        let desired = DyniboPose::default();
        assert_eq!(
            dynibo_inverse_kinematics(
                robot,
                workspace,
                overlap,
                4,
                target,
                &desired,
                DyniboIkOptions::default(),
                overlap,
                4
            ),
            DyniboStatus::InvalidArgument
        );
        dynibo_workspace_destroy(workspace);
        dynibo_robot_destroy(robot);
    }
}

#[test]
fn floating_vector_calculations_reject_overlapping_output_and_invalid_base() {
    // SAFETY: Handles are valid; overlap calls return before constructing slices.
    unsafe {
        let (robot, workspace, target) = floating_handles();
        let mut q = [0.0; 4];
        let qd = [0.0; 4];
        let qdd = [0.0; 4];
        let forces = [0.0; 10];
        let base = DyniboBaseState::default();
        let overlap = q.as_mut_ptr();
        assert_eq!(
            dynibo_floating_jacobian(robot, workspace, &base, overlap, 4, target, overlap, 60),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_jacobian_derivative(
                robot,
                workspace,
                &base,
                overlap,
                qd.as_ptr(),
                4,
                target,
                overlap,
                60
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_mass_matrix(robot, workspace, &base, overlap, 4, overlap, 100),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_velocity_product_forces(
                robot,
                workspace,
                &base,
                overlap,
                qd.as_ptr(),
                4,
                overlap,
                10
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_gravity(
                robot,
                workspace,
                &base,
                overlap,
                4,
                ptr::null(),
                0,
                overlap,
                10
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_inverse_dynamics(
                robot,
                workspace,
                &base,
                overlap,
                qd.as_ptr(),
                qdd.as_ptr(),
                4,
                ptr::null(),
                0,
                overlap,
                10
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_forward_dynamics(
                robot,
                workspace,
                &base,
                overlap,
                qd.as_ptr(),
                4,
                forces.as_ptr(),
                10,
                ptr::null(),
                0,
                overlap,
                10
            ),
            DyniboStatus::InvalidArgument
        );
        let invalid_base = DyniboBaseState {
            velocity: DyniboTwist {
                angular: [f64::NAN, 0.0, 0.0],
                ..DyniboTwist::default()
            },
            ..DyniboBaseState::default()
        };
        let mut pose = DyniboPose::default();
        assert_eq!(
            dynibo_floating_forward_kinematics(
                robot,
                workspace,
                &invalid_base,
                q.as_ptr(),
                4,
                target,
                &mut pose
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_forward_kinematics(
                robot,
                workspace,
                ptr::null(),
                q.as_ptr(),
                4,
                target,
                &mut pose
            ),
            DyniboStatus::InvalidArgument
        );
        dynibo_floating_workspace_destroy(workspace);
        dynibo_floating_robot_destroy(robot);
    }
}

#[test]
fn pose_and_twist_outputs_reject_overlapping_joint_buffers() {
    // SAFETY: The invalid calls return from raw range validation before a
    // typed output reference or the declared oversized output is accessed.
    unsafe {
        let (robot, workspace, target) = fixed_handles();
        let (floating, floating_workspace, floating_target) = floating_handles();
        let mut q = [0.0; 4];
        let qd = [0.0; 4];
        let qdd = [0.0; 4];
        let pose = q.as_mut_ptr().cast::<DyniboPose>();
        let twist = q.as_mut_ptr().cast::<DyniboTwist>();
        let tool = DyniboPose::default();
        let base = DyniboBaseState::default();
        assert_eq!(
            dynibo_forward_kinematics(robot, workspace, q.as_ptr(), 4, target, pose),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_forward_velocity_kinematics(
                robot,
                workspace,
                q.as_ptr(),
                qd.as_ptr(),
                4,
                target,
                &tool,
                twist
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_forward_acceleration_kinematics(
                robot,
                workspace,
                q.as_ptr(),
                qd.as_ptr(),
                qdd.as_ptr(),
                4,
                target,
                twist
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_forward_kinematics(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                4,
                floating_target,
                pose
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_forward_velocity_kinematics(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                4,
                floating_target,
                &tool,
                twist
            ),
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            dynibo_floating_forward_acceleration_kinematics(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                qdd.as_ptr(),
                4,
                floating_target,
                twist
            ),
            DyniboStatus::InvalidArgument
        );
        dynibo_workspace_destroy(workspace);
        dynibo_robot_destroy(robot);
        dynibo_floating_workspace_destroy(floating_workspace);
        dynibo_floating_robot_destroy(floating);
    }
}
