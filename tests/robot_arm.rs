// Several regression inputs deliberately preserve the decimal constants used
// by the original C++ tests instead of replacing them with exact PI fractions.
#![allow(clippy::approx_constant)]

use std::{
    f64::consts::{FRAC_PI_2, PI},
    path::PathBuf,
};

use approx::{assert_abs_diff_eq, assert_relative_eq};
use dyno::{
    Error, Frame, InverseKinematicsOptions, Joint, JointType, JointVector, Link, Load, Robot,
    Twist, Wrench,
};
use nalgebra::{Isometry3, SVector, Translation3, UnitQuaternion, Vector3};

fn test_arm() -> Robot {
    Robot::from_urdf(urdf_path("test_arm.urdf")).expect("test URDF must load")
}

fn end_link(arm: &Robot) -> &Link {
    arm.leaf_links()
        .next()
        .expect("robot must have a leaf link")
}

fn urdf_path(file_name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(file_name)
}

fn tree_arm() -> Robot {
    Robot::from_urdf(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("benches/data/test_tree_7.urdf"),
    )
    .expect("tree URDF must load")
}

#[allow(clippy::too_many_arguments)]
fn joint(name: &str, joint_type: JointType, xyz: [f64; 3], rpy: [f64; 3], axis: [f64; 3]) -> Joint {
    Joint::new(
        name,
        joint_type,
        Isometry3::from_parts(
            Translation3::new(xyz[0], xyz[1], xyz[2]),
            UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
        ),
        Vector3::new(axis[0], axis[1], axis[2]),
        -10.0,
        10.0,
        100.0,
    )
    .unwrap()
}

#[test]
fn joint_and_loaded_link_preserve_their_parameters() {
    let mut joint = Joint::new(
        "joint_1",
        JointType::Revolute,
        Isometry3::from_parts(
            Translation3::new(2.0, 3.0, 4.0),
            UnitQuaternion::from_euler_angles(0.0, 2.0, 1.0),
        ),
        Vector3::z(),
        -3.14,
        3.14,
        100.0,
    )
    .unwrap();
    let arm = test_arm();
    let link = arm.link("test_link_2").unwrap();

    assert_eq!(joint.name(), "joint_1");
    assert_eq!(link.name(), "test_link_2");
    assert_eq!(joint.joint_type(), JointType::Revolute);
    assert_abs_diff_eq!(joint.lower_limit(), -3.14);
    assert_abs_diff_eq!(joint.upper_limit(), 3.14);
    assert_abs_diff_eq!(joint.velocity_limit(), 100.0);
    assert_eq!(link.mass(), 7.016);
    assert_abs_diff_eq!(joint.origin().translation.vector.x, 2.0);
    assert_abs_diff_eq!(link.center_of_mass().z, 0.129994);
    assert!(joint.is_over_limit(4.0));
    assert!(!joint.is_over_limit(0.0));
    assert_abs_diff_eq!(joint.set_position(10.0), 3.14);
    assert_abs_diff_eq!(joint.set_velocity(-200.0), -100.0);
    assert_abs_diff_eq!(joint.set_acceleration(12.0), 12.0);
}

#[test]
fn revolute_and_prismatic_joint_frames_match_urdf_semantics() {
    let revolute = joint(
        "revolute",
        JointType::Revolute,
        [0.0, 0.0, 0.226],
        [0.0, 0.0, FRAC_PI_2],
        [0.0, 0.0, 1.0],
    );
    let frame = revolute.frame(0.3 * PI);
    let expected = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.8 * PI);
    assert_relative_eq!(frame.rotation, expected, epsilon = 1.0e-12);
    assert_abs_diff_eq!(frame.translation.vector.z, 0.226);

    let prismatic = joint(
        "slide",
        JointType::Prismatic,
        [1.0, 0.0, 0.0],
        [0.0; 3],
        [0.0, 1.0, 0.0],
    );
    assert_relative_eq!(
        prismatic.frame(0.25).translation.vector,
        Vector3::new(1.0, 0.25, 0.0),
        epsilon = 1.0e-12
    );
}

#[test]
fn urdf_rs_loads_test_arm_and_checks_calculation_size() {
    let arm = test_arm();
    assert_eq!(arm.name(), "test_arm");
    assert_eq!(arm.links().len(), 5);
    assert_eq!(arm.links()[0].name(), "test_base_link");
    assert_eq!(arm.links()[1].name(), "test_link_1");
    assert_eq!(arm.joints().len(), 4);
    assert_eq!(arm.joints()[0].name(), "test_joint_1");
    assert_eq!(arm.link_count(), 5);
    assert_eq!(arm.joint_count(), 4);
    assert_eq!(arm.link("test_link_1").unwrap().name(), "test_link_1");
    assert!(matches!(
        arm.link("missing_link"),
        Err(Error::UnknownLink { name }) if name == "missing_link"
    ));
    let owned_link = arm.link("test_link_1").unwrap().clone();
    arm.forward_kinematics(&JointVector::<4>::zeros(), &owned_link)
        .expect("a cloned link remains valid for its model");
    let other_arm = test_arm();
    assert!(matches!(
        other_arm.forward_kinematics(&JointVector::<4>::zeros(), &owned_link),
        Err(Error::InvalidLink { name }) if name == "test_link_1"
    ));
    assert_abs_diff_eq!(arm.links()[2].mass(), 7.016);
    assert_abs_diff_eq!(arm.joints()[1].origin().translation.vector.z, 0.108);

    let wrong_size = arm
        .forward_kinematics(&JointVector::<3>::zeros(), end_link(&arm))
        .unwrap_err();
    assert!(matches!(
        wrong_size,
        Error::WrongJointCount {
            expected: 4,
            actual: 3
        }
    ));
}

#[test]
fn jacobian_agrees_with_forward_kinematics_finite_difference() {
    let arm = test_arm();
    let q = JointVector::<4>::new(0.147607, 1.014764, -1.840751, 0.825987);
    let jacobian = arm.jacobian(&q, end_link(&arm)).unwrap();
    let epsilon = 1.0e-7;

    for joint in 0..4 {
        let mut q_plus = q;
        let mut q_minus = q;
        q_plus[joint] += epsilon;
        q_minus[joint] -= epsilon;
        let plus = arm.forward_kinematics(&q_plus, end_link(&arm)).unwrap();
        let minus = arm.forward_kinematics(&q_minus, end_link(&arm)).unwrap();
        let linear = (plus.translation.vector - minus.translation.vector) / (2.0 * epsilon);
        let angular = (plus.rotation * minus.rotation.inverse()).scaled_axis() / (2.0 * epsilon);
        assert_relative_eq!(
            jacobian.fixed_view::<3, 1>(0, joint).into_owned(),
            angular,
            epsilon = 2.0e-8
        );
        assert_relative_eq!(
            jacobian.fixed_view::<3, 1>(3, joint).into_owned(),
            linear,
            epsilon = 2.0e-8
        );
    }
}

#[test]
fn test_arm_jacobian_matches_numeric_reference() {
    let arm = test_arm();
    let q = JointVector::<4>::new(0.205506, 1.443005, -2.645997, 1.202992);
    let expected = nalgebra::SMatrix::<f64, 6, 4>::from_row_slice(&[
        -0.0000, 0.2041, 0.2041, 0.2041, -0.0000, -0.9790, -0.9790, -0.9790, 1.0000, 0.0000,
        0.0000, 0.0000, -0.0303, -0.0367, 0.2740, 0.0, 0.1455, -0.0076, 0.0571, 0.0, 0.0, 0.1487,
        0.1079, 0.0,
    ]);
    assert_relative_eq!(
        arm.jacobian(&q, end_link(&arm)).unwrap(),
        expected,
        epsilon = 5.0e-4
    );

    let q = JointVector::<4>::new(0.147607, 1.014764, -1.840751, 0.825987);
    let expected = nalgebra::SMatrix::<f64, 6, 4>::from_row_slice(&[
        0.0000, 0.1471, 0.1471, 0.1471, -0.0000, -0.9891, -0.9891, -0.9891, 1.0000, 0.0000, 0.0000,
        0.0000, -0.0547, -0.0507, 0.2182, 0.0, 0.3682, -0.0075, 0.0324, 0.0, 0.0, 0.3723, 0.2033,
        0.0,
    ]);
    assert_relative_eq!(
        arm.jacobian(&q, end_link(&arm)).unwrap(),
        expected,
        epsilon = 5.0e-4
    );
}

#[test]
fn damped_inverse_kinematics_reaches_a_known_pose() {
    let arm = test_arm();
    let target = end_link(&arm);
    let expected_q = JointVector::<4>::new(0.2, 1.0, -1.2, 0.45);
    let desired = arm.forward_kinematics(&expected_q, target).unwrap();
    let initial_q = JointVector::<4>::zeros();

    let solved_q = arm
        .inverse_kinematics(&initial_q, target, &desired)
        .unwrap();
    let solved = arm.forward_kinematics(&solved_q, target).unwrap();

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
    let arm = test_arm();
    let initial_q = JointVector::<4>::zeros();
    let unreachable =
        Frame::from_parts(Translation3::new(1.0, 0.0, 0.0), UnitQuaternion::identity());
    let options = InverseKinematicsOptions {
        max_iterations: 2,
        ..InverseKinematicsOptions::default()
    };
    let error = arm
        .inverse_kinematics_with_options(&initial_q, arm.root_link(), &unreachable, options)
        .unwrap_err();
    assert!(matches!(
        error,
        Error::NotConverged {
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
        .inverse_kinematics_with_options(
            &initial_q,
            end_link(&arm),
            &Frame::identity(),
            invalid_options,
        )
        .unwrap_err();
    assert!(matches!(error, Error::InvalidOptions(_)));

    let mut non_finite_q = initial_q;
    non_finite_q[0] = f64::NAN;
    let error = arm
        .inverse_kinematics(&non_finite_q, end_link(&arm), &Frame::identity())
        .unwrap_err();
    assert!(matches!(
        error,
        Error::NonFiniteInput {
            input: "initial joint vector"
        }
    ));

    let outside_q = JointVector::<4>::new(0.8, 0.0, 0.0, 0.0);
    let outside_target = arm.forward_kinematics(&outside_q, end_link(&arm)).unwrap();
    let error = arm
        .inverse_kinematics(&outside_q, end_link(&arm), &outside_target)
        .unwrap_err();
    assert!(matches!(
        error,
        Error::JointLimitViolation {
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
fn velocity_is_jacobian_times_joint_velocity() {
    let arm = test_arm();
    let q = JointVector::<4>::new(PI / 12.0, PI / 3.0, -PI / 2.0, PI / 6.0);
    let qd = q;
    let velocity = arm
        .forward_velocity_kinematics(
            &q,
            &qd,
            end_link(&arm),
            &Frame::identity(),
            &Frame::identity(),
        )
        .unwrap();
    assert_relative_eq!(
        velocity.to_vector(),
        arm.jacobian(&q, end_link(&arm)).unwrap() * qd,
        epsilon = 1.0e-12
    );
    let expected = nalgebra::SVector::<f64, 6>::new(
        -2.88805923e-17,
        3.20702034e-17,
        2.61799388e-1,
        -3.84628546e-1,
        1.07215119e-2,
        3.15166559e-2,
    );
    assert_relative_eq!(velocity.to_vector(), expected, epsilon = 1.0e-8);

    let base = Isometry3::from_parts(
        Translation3::new(0.3, -0.2, 0.5),
        UnitQuaternion::from_euler_angles(0.2, -0.4, 0.1),
    );
    let tool = Isometry3::translation(0.1, -0.03, 0.2);
    let end = arm.forward_kinematics(&q, end_link(&arm)).unwrap();
    let mut tool_jacobian = arm.jacobian(&q, end_link(&arm)).unwrap();
    let offset_world = end.rotation * tool.translation.vector;
    for i in 0..4 {
        let angular = tool_jacobian.fixed_view::<3, 1>(0, i).into_owned();
        let shifted =
            tool_jacobian.fixed_view::<3, 1>(3, i).into_owned() + angular.cross(&offset_world);
        tool_jacobian
            .fixed_view_mut::<3, 1>(3, i)
            .copy_from(&shifted);
    }
    let tool_jacobian_velocity = tool_jacobian * qd;
    let expected_with_frames = Twist::new(
        base.rotation * tool_jacobian_velocity.fixed_rows::<3>(0).into_owned(),
        base.rotation * tool_jacobian_velocity.fixed_rows::<3>(3).into_owned(),
    );
    assert_relative_eq!(
        arm.forward_velocity_kinematics(&q, &qd, end_link(&arm), &base, &tool)
            .unwrap()
            .to_vector(),
        expected_with_frames.to_vector(),
        epsilon = 1.0e-12
    );
}

#[test]
fn forward_acceleration_matches_finite_difference() {
    let arm = test_arm();
    let q = JointVector::<4>::new(0.2, 1.1, -0.7, 0.4);
    let qd = JointVector::<4>::new(-0.3, 0.5, -0.2, 0.8);
    let epsilon = 1.0e-7;
    let numerical = (arm.jacobian(&(q + epsilon * qd), end_link(&arm)).unwrap()
        - arm.jacobian(&(q - epsilon * qd), end_link(&arm)).unwrap())
        / (2.0 * epsilon);
    let qdd = JointVector::<4>::new(0.7, -0.4, 0.1, 0.3);
    assert_relative_eq!(
        arm.forward_acceleration_kinematics(&q, &qd, &qdd, end_link(&arm))
            .unwrap()
            .to_vector(),
        arm.jacobian(&q, end_link(&arm)).unwrap() * qdd + numerical * qd,
        epsilon = 2.0e-8
    );

    let mixed_arm = Robot::from_urdf(urdf_path("mixed_arm.urdf")).unwrap();
    let mixed_q = JointVector::<2>::new(0.4, 0.2);
    let mixed_qd = JointVector::<2>::new(-0.3, 0.5);
    let mixed_numerical = (mixed_arm
        .jacobian(&(mixed_q + epsilon * mixed_qd), end_link(&mixed_arm))
        .unwrap()
        - mixed_arm
            .jacobian(&(mixed_q - epsilon * mixed_qd), end_link(&mixed_arm))
            .unwrap())
        / (2.0 * epsilon);
    let mixed_qdd = JointVector::<2>::new(0.7, -0.4);
    assert_relative_eq!(
        mixed_arm
            .forward_acceleration_kinematics(&mixed_q, &mixed_qd, &mixed_qdd, end_link(&mixed_arm),)
            .unwrap()
            .to_vector(),
        mixed_arm.jacobian(&mixed_q, end_link(&mixed_arm)).unwrap() * mixed_qdd
            + mixed_numerical * mixed_qd,
        epsilon = 2.0e-8
    );

    for q in [
        JointVector::<4>::new(0.0, FRAC_PI_2, 0.0, 0.0),
        JointVector::<4>::new(1.5708, 1.0472, -1.0472, 0.5236),
    ] {
        let numerical_jacobian_dot = (arm.jacobian(&(q + epsilon * q), end_link(&arm)).unwrap()
            - arm.jacobian(&(q - epsilon * q), end_link(&arm)).unwrap())
            / (2.0 * epsilon);
        let expected = arm.jacobian(&q, end_link(&arm)).unwrap() * q + numerical_jacobian_dot * q;
        let acceleration = arm
            .forward_acceleration_kinematics(&q, &q, &q, end_link(&arm))
            .unwrap();
        assert_relative_eq!(acceleration.to_vector(), expected, epsilon = 2.0e-8);
    }
}

#[test]
fn gravity_matches_original_two_link_cases() {
    let arm = Robot::from_urdf(urdf_path("gravity_arm.urdf")).unwrap();
    let q = JointVector::<2>::new(FRAC_PI_2, FRAC_PI_2);
    let (tau, _) = arm.gravity(&q, &Frame::identity(), &[]).unwrap();
    assert_abs_diff_eq!(tau[0], 0.0, epsilon = 0.1);
    assert_abs_diff_eq!(tau[1], -4.903325, epsilon = 0.1);

    let vertical_base = Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_euler_angles(FRAC_PI_2, 0.0, 0.0),
    );
    let (tau, _) = arm
        .gravity(&JointVector::<2>::zeros(), &vertical_base, &[])
        .unwrap();
    assert_abs_diff_eq!(tau[0], 9.80665, epsilon = 0.1);
    assert_abs_diff_eq!(tau[1], 0.0, epsilon = 0.1);
}

#[test]
fn inverse_dynamics_matches_test_arm_numeric_reference() {
    let arm = test_arm();
    let set_q = JointVector::<4>::new(1.5708, 1.0472, -1.0472, 0.5236);
    let zero = JointVector::<4>::zeros();
    let random = JointVector::<4>::new(-0.2, 0.5, -0.3, 0.8);
    let cases = [
        (
            set_q,
            zero,
            zero,
            JointVector::<4>::new(0.0, 38.8143, 18.4362, 0.0607),
            1.0e-4,
        ),
        (
            zero,
            random,
            zero,
            JointVector::<4>::new(-0.1404, 59.2065, 18.4470, 0.0716),
            1.0e-2,
        ),
        (
            zero,
            zero,
            random,
            JointVector::<4>::new(-0.6787, 60.3962, 18.8600, 0.0733),
            1.0e-2,
        ),
        (
            set_q,
            zero,
            random,
            JointVector::<4>::new(-0.5904, 39.8839, 18.6987, 0.0623),
            1.0e-2,
        ),
        (
            zero,
            random,
            random,
            JointVector::<4>::new(-0.8191, 60.3961, 18.8599, 0.0733),
            1.0e-2,
        ),
        (
            set_q,
            random,
            random,
            JointVector::<4>::new(-0.4478, 39.8180, 18.5676, 0.0621),
            1.0e-2,
        ),
    ];
    for (q, qd, qdd, expected, epsilon) in cases {
        let (tau, _) = arm
            .inverse_dynamics(
                &q,
                &qd,
                &qdd,
                &Frame::identity(),
                Twist::zeros(),
                Twist::zeros(),
                &[],
            )
            .unwrap();
        assert_relative_eq!(tau, expected, epsilon = epsilon);
    }
}

#[test]
fn tree_jacobians_match_finite_difference_on_both_branches() {
    let arm = tree_arm();
    let q = JointVector::<7>::from_row_slice(&[0.2, -0.3, 0.4, -0.5, 0.6, -0.7, 0.8]);
    let epsilon = 1.0e-7;

    for target_name in ["left_tool", "right_tool"] {
        let target = arm.link(target_name).unwrap();
        let jacobian = arm.jacobian(&q, target).unwrap();
        for joint in 0..7 {
            let mut q_plus = q;
            let mut q_minus = q;
            q_plus[joint] += epsilon;
            q_minus[joint] -= epsilon;
            let plus = arm.forward_kinematics(&q_plus, target).unwrap();
            let minus = arm.forward_kinematics(&q_minus, target).unwrap();
            let linear = (plus.translation.vector - minus.translation.vector) / (2.0 * epsilon);
            let angular =
                (plus.rotation * minus.rotation.inverse()).scaled_axis() / (2.0 * epsilon);
            assert_relative_eq!(
                jacobian.fixed_view::<3, 1>(0, joint).into_owned(),
                angular,
                epsilon = 2.0e-8
            );
            assert_relative_eq!(
                jacobian.fixed_view::<3, 1>(3, joint).into_owned(),
                linear,
                epsilon = 2.0e-8
            );
        }
    }
}

#[test]
fn tree_external_loads_are_isolated_and_add_linearly() {
    let arm = tree_arm();
    let q = JointVector::<7>::from_row_slice(&[0.2, -0.3, 0.4, -0.5, 0.6, -0.7, 0.8]);
    let left = Load {
        link: arm.link("left_tool").unwrap(),
        wrench: Wrench::new(Vector3::new(0.3, -0.2, 0.4), Vector3::new(1.0, 0.5, -0.7)),
    };
    let right = Load {
        link: arm.link("right_tool").unwrap(),
        wrench: Wrench::new(Vector3::new(-0.4, 0.1, 0.2), Vector3::new(-0.6, 0.8, 0.3)),
    };

    let (baseline, baseline_base) = arm.gravity(&q, &Frame::identity(), &[]).unwrap();
    let (left_only, left_base) = arm.gravity(&q, &Frame::identity(), &[left]).unwrap();
    let (right_only, right_base) = arm.gravity(&q, &Frame::identity(), &[right]).unwrap();
    let (both, both_base) = arm.gravity(&q, &Frame::identity(), &[left, right]).unwrap();

    for right_joint in [2, 4, 6] {
        assert_abs_diff_eq!(
            left_only[right_joint],
            baseline[right_joint],
            epsilon = 1.0e-12
        );
    }
    for left_joint in [1, 3, 5] {
        assert_abs_diff_eq!(
            right_only[left_joint],
            baseline[left_joint],
            epsilon = 1.0e-12
        );
    }
    assert!((left_only - baseline).norm() > 1.0e-6);
    assert!((right_only - baseline).norm() > 1.0e-6);
    for (external, with_load) in [(left, left_only), (right, right_only)] {
        let frame = arm.forward_kinematics(&q, external.link).unwrap();
        let torque = frame.rotation * external.wrench.torque;
        let force = frame.rotation * external.wrench.force;
        let wrench_in_base =
            SVector::<f64, 6>::from_iterator(torque.iter().chain(force.iter()).copied());
        let expected = arm.jacobian(&q, external.link).unwrap().transpose() * wrench_in_base;
        assert_relative_eq!(with_load - baseline, expected, epsilon = 2.0e-12);
    }
    assert_relative_eq!(both, left_only + right_only - baseline, epsilon = 2.0e-12);
    assert_relative_eq!(
        both_base.torque,
        left_base.torque + right_base.torque - baseline_base.torque,
        epsilon = 2.0e-12
    );
    assert_relative_eq!(
        both_base.force,
        left_base.force + right_base.force - baseline_base.force,
        epsilon = 2.0e-12
    );
}

#[test]
fn tree_gravity_equals_zero_motion_inverse_dynamics() {
    let arm = tree_arm();
    let q = JointVector::<7>::from_row_slice(&[-0.45, 0.12, -0.28, 0.63, -0.31, 0.22, -0.51]);
    let zero = JointVector::<7>::zeros();
    let (gravity, gravity_base) = arm.gravity(&q, &Frame::identity(), &[]).unwrap();
    let (inverse, inverse_base) = arm
        .inverse_dynamics(
            &q,
            &zero,
            &zero,
            &Frame::identity(),
            Twist::zeros(),
            Twist::zeros(),
            &[],
        )
        .unwrap();
    assert_relative_eq!(inverse, gravity, epsilon = 2.0e-12);
    assert_relative_eq!(inverse_base.torque, gravity_base.torque, epsilon = 2.0e-12);
    assert_relative_eq!(inverse_base.force, gravity_base.force, epsilon = 2.0e-12);
}
