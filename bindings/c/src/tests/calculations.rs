use super::*;

#[test]
fn fixed_and_floating_abi_success_paths_return_finite_results() {
    // SAFETY: Every opaque handle is created by this ABI, all slices remain
    // live for the duration of each call, and output buffers have the sizes
    // required by the fixed- and floating-base contracts.
    unsafe {
        let (robot, workspace, target) = fixed_handles();
        let (floating, floating_workspace, floating_target) = floating_handles();
        let q = [0.2, -0.3, 0.4, -0.1];
        let qd = [-0.1, 0.2, -0.3, 0.4];
        let qdd = [0.3, -0.2, 0.1, -0.4];
        let base = DyniboBaseState {
            frame: DyniboPose {
                translation: [0.1, -0.2, 0.3],
                ..DyniboPose::default()
            },
            velocity: DyniboTwist {
                angular: [0.1, -0.2, 0.3],
                linear: [-0.3, 0.2, 0.1],
            },
            acceleration: DyniboTwist {
                angular: [-0.2, 0.1, 0.3],
                linear: [0.4, -0.1, 0.2],
            },
        };
        let tool = DyniboPose {
            translation: [0.02, -0.01, 0.04],
            ..DyniboPose::default()
        };

        assert_eq!(
            CStr::from_ptr(dynibo_version()).to_bytes(),
            env!("CARGO_PKG_VERSION").as_bytes()
        );
        assert!(
            !CStr::from_ptr(dynibo_robot_name(robot))
                .to_bytes()
                .is_empty()
        );
        assert!(
            !CStr::from_ptr(dynibo_floating_robot_name(floating))
                .to_bytes()
                .is_empty()
        );
        assert_eq!(dynibo_robot_joint_count(robot), q.len());
        assert_eq!(dynibo_robot_generalized_count(robot), q.len());
        assert_eq!(dynibo_floating_robot_joint_count(floating), q.len());
        assert_eq!(
            dynibo_floating_robot_generalized_count(floating),
            q.len() + 6
        );
        assert_eq!(
            dynibo_robot_link_count(robot),
            dynibo_floating_robot_link_count(floating)
        );
        assert_eq!(
            dynibo_robot_set_base_frame(robot, &DyniboPose::default()),
            DyniboStatus::Ok
        );

        let mut fixed_pose = DyniboPose::default();
        assert_eq!(
            dynibo_forward_kinematics(
                robot,
                workspace,
                q.as_ptr(),
                q.len(),
                target,
                &mut fixed_pose,
            ),
            DyniboStatus::Ok
        );
        let mut floating_pose = DyniboPose::default();
        assert_eq!(
            dynibo_floating_forward_kinematics(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                floating_target,
                &mut floating_pose,
            ),
            DyniboStatus::Ok
        );

        let mut fixed_jacobian = [0.0; 24];
        assert_eq!(
            dynibo_jacobian(
                robot,
                workspace,
                q.as_ptr(),
                q.len(),
                target,
                fixed_jacobian.as_mut_ptr(),
                fixed_jacobian.len(),
            ),
            DyniboStatus::Ok
        );
        let mut floating_jacobian = [0.0; 60];
        assert_eq!(
            dynibo_floating_jacobian(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                floating_target,
                floating_jacobian.as_mut_ptr(),
                floating_jacobian.len(),
            ),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_jacobian_derivative(
                robot,
                workspace,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                target,
                fixed_jacobian.as_mut_ptr(),
                fixed_jacobian.len(),
            ),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_floating_jacobian_derivative(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                floating_target,
                floating_jacobian.as_mut_ptr(),
                floating_jacobian.len(),
            ),
            DyniboStatus::Ok
        );

        let mut fixed_matrix = [0.0; 16];
        assert_eq!(
            dynibo_mass_matrix(
                robot,
                workspace,
                q.as_ptr(),
                q.len(),
                fixed_matrix.as_mut_ptr(),
                fixed_matrix.len(),
            ),
            DyniboStatus::Ok
        );
        let mut floating_matrix = [0.0; 100];
        assert_eq!(
            dynibo_floating_mass_matrix(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                q.len(),
                floating_matrix.as_mut_ptr(),
                floating_matrix.len(),
            ),
            DyniboStatus::Ok
        );

        let mut fixed_output = [0.0; 4];
        assert_eq!(
            dynibo_velocity_product_forces(
                robot,
                workspace,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                fixed_output.as_mut_ptr(),
                fixed_output.len(),
            ),
            DyniboStatus::Ok
        );
        let mut floating_output = [0.0; 10];
        assert_eq!(
            dynibo_floating_velocity_product_forces(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                floating_output.as_mut_ptr(),
                floating_output.len(),
            ),
            DyniboStatus::Ok
        );

        let mut fixed_twist = DyniboTwist::default();
        assert_eq!(
            dynibo_forward_velocity_kinematics(
                robot,
                workspace,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                target,
                &tool,
                &mut fixed_twist,
            ),
            DyniboStatus::Ok
        );
        let mut floating_twist = DyniboTwist::default();
        assert_eq!(
            dynibo_floating_forward_velocity_kinematics(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                floating_target,
                &tool,
                &mut floating_twist,
            ),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_forward_acceleration_kinematics(
                robot,
                workspace,
                q.as_ptr(),
                qd.as_ptr(),
                qdd.as_ptr(),
                q.len(),
                target,
                &mut fixed_twist,
            ),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_floating_forward_acceleration_kinematics(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                qdd.as_ptr(),
                q.len(),
                floating_target,
                &mut floating_twist,
            ),
            DyniboStatus::Ok
        );

        assert_eq!(
            dynibo_gravity(
                robot,
                workspace,
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
                floating,
                floating_workspace,
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
        assert_eq!(
            dynibo_inverse_dynamics(
                robot,
                workspace,
                q.as_ptr(),
                qd.as_ptr(),
                qdd.as_ptr(),
                q.len(),
                ptr::null(),
                0,
                fixed_output.as_mut_ptr(),
                fixed_output.len(),
            ),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_floating_inverse_dynamics(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                qdd.as_ptr(),
                q.len(),
                ptr::null(),
                0,
                floating_output.as_mut_ptr(),
                floating_output.len(),
            ),
            DyniboStatus::Ok
        );
        let fixed_forces = fixed_output;
        assert_eq!(
            dynibo_forward_dynamics(
                robot,
                workspace,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                fixed_forces.as_ptr(),
                fixed_forces.len(),
                ptr::null(),
                0,
                fixed_output.as_mut_ptr(),
                fixed_output.len(),
            ),
            DyniboStatus::Ok
        );
        let floating_forces = floating_output;
        assert_eq!(
            dynibo_floating_forward_dynamics(
                floating,
                floating_workspace,
                &base,
                q.as_ptr(),
                qd.as_ptr(),
                q.len(),
                floating_forces.as_ptr(),
                floating_forces.len(),
                ptr::null(),
                0,
                floating_output.as_mut_ptr(),
                floating_output.len(),
            ),
            DyniboStatus::Ok
        );

        let mut ik_output = [0.0; 4];
        assert_eq!(
            dynibo_inverse_kinematics(
                robot,
                workspace,
                q.as_ptr(),
                q.len(),
                target,
                &fixed_pose,
                dynibo_ik_options_default(),
                ik_output.as_mut_ptr(),
                ik_output.len(),
            ),
            DyniboStatus::Ok
        );

        assert!(fixed_jacobian.iter().all(|x| x.is_finite()));
        assert!(floating_jacobian.iter().all(|x| x.is_finite()));
        assert!(fixed_matrix.iter().all(|x| x.is_finite()));
        assert!(floating_matrix.iter().all(|x| x.is_finite()));
        assert!(fixed_output.iter().all(|x| x.is_finite()));
        assert!(floating_output.iter().all(|x| x.is_finite()));
        assert!(ik_output.iter().all(|x| x.is_finite()));

        dynibo_workspace_destroy(workspace);
        dynibo_robot_destroy(robot);
        dynibo_floating_workspace_destroy(floating_workspace);
        dynibo_floating_robot_destroy(floating);
        dynibo_workspace_destroy(ptr::null_mut());
        dynibo_floating_workspace_destroy(ptr::null_mut());
        dynibo_robot_destroy(ptr::null_mut());
        dynibo_floating_robot_destroy(ptr::null_mut());
    }
}
