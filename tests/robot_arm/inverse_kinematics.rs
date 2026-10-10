use super::*;

#[test]
fn damped_inverse_kinematics_reaches_a_known_pose() {
    let mut arm = test_arm();
    let target = end_link(&arm);
    let expected_q = JointVector::<4>::new(0.2, 1.0, -1.2, 0.45);
    let desired = arm.test_forward_kinematics(&expected_q, target).unwrap();
    let initial_q = JointVector::<4>::zeros();

    let solved_q = arm
        .test_inverse_kinematics(
            &initial_q,
            target,
            &desired,
            InverseKinematicsOptions::default(),
        )
        .unwrap();
    let solved = arm.test_forward_kinematics(&solved_q, target).unwrap();

    assert!(
        (solved.translation.vector - desired.translation.vector).norm() <= 1.0e-6,
        "position did not converge: solved_q={solved_q:?}"
    );
    assert!(
        (desired.rotation * solved.rotation.inverse())
            .scaled_axis()
            .norm()
            <= 1.0e-6,
        "orientation did not converge: solved_q={solved_q:?}"
    );
}

#[test]
fn inverse_kinematics_reports_specific_solver_errors() {
    let mut arm = test_arm();
    let initial_q = JointVector::<4>::zeros();
    let unreachable =
        Frame::from_parts(Translation3::new(1.0, 0.0, 0.0), UnitQuaternion::identity());
    let options = InverseKinematicsOptions {
        max_iterations: 2,
        ..InverseKinematicsOptions::default()
    };
    let error = arm
        .test_inverse_kinematics(&initial_q, arm.root_link_id(), &unreachable, options)
        .unwrap_err();
    assert!(matches!(
        error,
        Error::IkNotConverged {
            iterations: 2,
            translation_error,
            rotation_error,
        } if (translation_error - 1.0).abs() <= 1.0e-12 && rotation_error <= 1.0e-12
    ));

    let invalid_options = InverseKinematicsOptions {
        damping: 0.0,
        ..InverseKinematicsOptions::default()
    };
    let error = arm
        .test_inverse_kinematics(
            &initial_q,
            end_link(&arm),
            &Frame::identity(),
            invalid_options,
        )
        .unwrap_err();
    assert!(matches!(
        error,
        Error::InvalidIkOptions {
            option: "damping",
            reason: "must be finite and greater than zero",
        }
    ));

    let mut non_finite_q = initial_q;
    non_finite_q[0] = f64::NAN;
    let error = arm
        .test_inverse_kinematics(
            &non_finite_q,
            end_link(&arm),
            &Frame::identity(),
            InverseKinematicsOptions::default(),
        )
        .unwrap_err();
    assert!(matches!(
        error,
        Error::NonFiniteInput { input: "initial_q" }
    ));

    let outside_q = JointVector::<4>::new(0.8, 0.0, 0.0, 0.0);
    let outside_target = arm
        .test_forward_kinematics(&outside_q, end_link(&arm))
        .unwrap();
    let error = arm
        .test_inverse_kinematics(
            &outside_q,
            end_link(&arm),
            &outside_target,
            InverseKinematicsOptions::default(),
        )
        .unwrap_err();
    assert!(matches!(
        error,
        Error::IkJointLimitViolation {
            joint_index: 0,
            ref joint,
            position,
            lower,
            upper,
        } if joint == "test_joint_1"
            && (position - 0.8).abs() <= 1.0e-12
            && (lower + 0.610865238198015).abs() <= 1.0e-12
            && (upper - 0.610865238198015).abs() <= 1.0e-12
    ));
}

#[test]
fn inverse_kinematics_validates_every_option_and_target_component() {
    let mut arm = test_arm();
    let target = end_link(&arm);
    let initial_q = JointVector::<4>::zeros();
    let desired = Frame::identity();
    let invalid_options = [
        InverseKinematicsOptions {
            max_iterations: 0,
            ..InverseKinematicsOptions::default()
        },
        InverseKinematicsOptions {
            translation_tolerance: 0.0,
            ..InverseKinematicsOptions::default()
        },
        InverseKinematicsOptions {
            rotation_tolerance: -1.0,
            ..InverseKinematicsOptions::default()
        },
        InverseKinematicsOptions {
            damping: f64::NAN,
            ..InverseKinematicsOptions::default()
        },
        InverseKinematicsOptions {
            max_step_norm: f64::INFINITY,
            ..InverseKinematicsOptions::default()
        },
    ];
    for options in invalid_options {
        assert!(matches!(
            arm.test_inverse_kinematics(&initial_q, target, &desired, options),
            Err(Error::InvalidIkOptions { .. })
        ));
    }

    let mut non_finite_target = desired;
    non_finite_target.translation.vector.y = f64::NAN;
    assert!(matches!(
        arm.test_inverse_kinematics(
            &initial_q,
            target,
            &non_finite_target,
            InverseKinematicsOptions::default(),
        ),
        Err(Error::NonFiniteIkInput {
            input: "target frame"
        })
    ));

    let unreachable = Frame::translation(1.0, 0.0, 0.0);
    let numerically_singular = InverseKinematicsOptions {
        damping: f64::MIN_POSITIVE,
        ..InverseKinematicsOptions::default()
    };
    assert!(matches!(
        arm.test_inverse_kinematics(
            &initial_q,
            arm.root_link_id(),
            &unreachable,
            numerically_singular,
        ),
        Err(Error::IkNumericalFailure { iteration: 1 })
    ));

    let rotation_only = Frame::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.25),
    );
    let one_iteration = InverseKinematicsOptions {
        max_iterations: 1,
        ..InverseKinematicsOptions::default()
    };
    assert!(matches!(
        arm.test_inverse_kinematics(
            &initial_q,
            arm.root_link_id(),
            &rotation_only,
            one_iteration,
        ),
        Err(Error::IkNotConverged {
            iterations: 1,
            translation_error,
            rotation_error,
        }) if translation_error <= 1.0e-12 && (rotation_error - 0.25).abs() <= 1.0e-12
    ));

    let overflowing_target = Frame::translation(f64::MAX, f64::MAX, f64::MAX);
    assert!(matches!(
        arm.test_inverse_kinematics(
            &initial_q,
            target,
            &overflowing_target,
            InverseKinematicsOptions::default(),
        ),
        Err(Error::IkNumericalFailure { iteration: 1 })
    ));
}
