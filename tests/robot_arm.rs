// Several regression inputs deliberately preserve the decimal constants used
// by the original C++ tests instead of replacing them with exact PI fractions.
#![allow(clippy::approx_constant)]

use std::f64::consts::{FRAC_PI_2, PI};

use approx::{assert_abs_diff_eq, assert_relative_eq};
use dyno::{
    Error, Frame, JointKind, JointLimit, JointVector, Motion, PassiveJointMap, RobotArm, RobotLink,
    RobotWithPassiveJoints, Wrench,
};
use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};

const TEST_ARM_URDF: &str = include_str!("data/test_arm.urdf");

fn test_arm() -> RobotArm<4> {
    RobotArm::from_urdf_str(TEST_ARM_URDF).expect("test URDF must load")
}

#[allow(clippy::too_many_arguments)]
fn link(
    name: &str,
    kind: JointKind,
    xyz: [f64; 3],
    rpy: [f64; 3],
    axis: [f64; 3],
    mass: f64,
    com: [f64; 3],
) -> RobotLink {
    RobotLink::new(
        name,
        kind,
        Isometry3::from_parts(
            Translation3::new(xyz[0], xyz[1], xyz[2]),
            UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
        ),
        Vector3::new(axis[0], axis[1], axis[2]),
        JointLimit {
            lower: -10.0,
            upper: 10.0,
            velocity: 100.0,
        },
        mass,
        Vector3::new(com[0], com[1], com[2]),
        Matrix3::identity() * 0.01,
    )
    .unwrap()
}

#[test]
fn robot_link_preserves_parameters_and_clamps_state() {
    let mut link = RobotLink::new(
        "link_1",
        JointKind::Revolute,
        Isometry3::from_parts(
            Translation3::new(2.0, 3.0, 4.0),
            UnitQuaternion::from_euler_angles(0.0, 2.0, 1.0),
        ),
        Vector3::z(),
        JointLimit {
            lower: -3.14,
            upper: 3.14,
            velocity: 100.0,
        },
        4.5,
        Vector3::new(1.1, 1.2, 1.3),
        Matrix3::new(0.1, -0.4, -0.5, -0.4, 0.2, -0.6, -0.5, -0.6, 0.3),
    )
    .unwrap();

    assert_eq!(link.name(), "link_1");
    assert_eq!(link.kind(), JointKind::Revolute);
    assert_eq!(link.mass(), 4.5);
    assert_abs_diff_eq!(link.origin().translation.vector.x, 2.0);
    assert_abs_diff_eq!(link.center_of_mass().z, 1.3);
    assert!(link.is_over_limit(4.0));
    assert!(!link.is_over_limit(0.0));
    assert_abs_diff_eq!(link.set_position(10.0), 3.14);
    assert_abs_diff_eq!(link.set_velocity(-200.0), -100.0);
    assert_abs_diff_eq!(link.set_acceleration(12.0), 12.0);
}

#[test]
fn revolute_and_prismatic_joint_frames_match_urdf_semantics() {
    let revolute = link(
        "revolute",
        JointKind::Revolute,
        [0.0, 0.0, 0.226],
        [0.0, 0.0, FRAC_PI_2],
        [0.0, 0.0, 1.0],
        0.0,
        [0.0; 3],
    );
    let frame = revolute.frame(0.3 * PI);
    let expected = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.8 * PI);
    assert_relative_eq!(frame.rotation, expected, epsilon = 1.0e-12);
    assert_abs_diff_eq!(frame.translation.vector.z, 0.226);

    let prismatic = link(
        "slide",
        JointKind::Prismatic,
        [1.0, 0.0, 0.0],
        [0.0; 3],
        [0.0, 1.0, 0.0],
        0.0,
        [0.0; 3],
    );
    assert_relative_eq!(
        prismatic.frame(0.25).translation.vector,
        Vector3::new(1.0, 0.25, 0.0),
        epsilon = 1.0e-12
    );
}

#[test]
fn urdf_rs_loads_test_arm_into_fixed_size_model() {
    let arm = test_arm();
    assert_eq!(arm.name(), "test_arm");
    assert_eq!(arm.links().len(), 4);
    assert_eq!(arm.links()[0].name(), "test_link_1");
    assert_eq!(arm.movable_joint_count(), 4);
    assert_abs_diff_eq!(arm.links()[1].mass(), 7.016);
    assert_abs_diff_eq!(arm.links()[1].origin().translation.vector.z, 0.108);

    let wrong_size = RobotArm::<3>::from_urdf_str(TEST_ARM_URDF).unwrap_err();
    assert!(matches!(
        wrong_size,
        Error::WrongJointCount {
            expected: 3,
            actual: 4
        }
    ));
}

#[test]
fn forward_kinematics_and_jacobian_agree_with_finite_difference() {
    let arm = test_arm();
    let q = JointVector::<4>::new(0.147607, 1.014764, -1.840751, 0.825987);
    let (end, jacobian) = arm.forward_kinematics_and_jacobian(&q);
    assert_relative_eq!(end, arm.forward_kinematics(&q), epsilon = 1.0e-12);
    let epsilon = 1.0e-7;

    for joint in 0..4 {
        let mut q_plus = q;
        let mut q_minus = q;
        q_plus[joint] += epsilon;
        q_minus[joint] -= epsilon;
        let plus = arm.forward_kinematics(&q_plus);
        let minus = arm.forward_kinematics(&q_minus);
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
    assert_relative_eq!(arm.jacobian(&q), expected, epsilon = 5.0e-4);

    let q = JointVector::<4>::new(0.147607, 1.014764, -1.840751, 0.825987);
    let expected = nalgebra::SMatrix::<f64, 6, 4>::from_row_slice(&[
        0.0000, 0.1471, 0.1471, 0.1471, -0.0000, -0.9891, -0.9891, -0.9891, 1.0000, 0.0000, 0.0000,
        0.0000, -0.0547, -0.0507, 0.2182, 0.0, 0.3682, -0.0075, 0.0324, 0.0, 0.0, 0.3723, 0.2033,
        0.0,
    ]);
    assert_relative_eq!(arm.jacobian(&q), expected, epsilon = 5.0e-4);
}

#[test]
fn velocity_is_jacobian_times_joint_velocity() {
    let arm = test_arm();
    let q = JointVector::<4>::new(PI / 12.0, PI / 3.0, -PI / 2.0, PI / 6.0);
    let qd = q;
    let velocity = arm.forward_velocity_kinematics(&q, &qd, &Frame::identity(), &Frame::identity());
    assert_relative_eq!(
        velocity.to_vector(),
        arm.jacobian(&q) * qd,
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
    let tool_jacobian_velocity = arm.jacobian_with_tool(&q, &tool) * qd;
    let expected_with_frames = Motion::new(
        base.rotation * tool_jacobian_velocity.fixed_rows::<3>(0).into_owned(),
        base.rotation * tool_jacobian_velocity.fixed_rows::<3>(3).into_owned(),
    );
    assert_relative_eq!(
        arm.forward_velocity_kinematics(&q, &qd, &base, &tool)
            .to_vector(),
        expected_with_frames.to_vector(),
        epsilon = 1.0e-12
    );
}

#[test]
fn jacobian_dot_and_forward_acceleration_match_finite_difference() {
    let arm = test_arm();
    let q = JointVector::<4>::new(0.2, 1.1, -0.7, 0.4);
    let qd = JointVector::<4>::new(-0.3, 0.5, -0.2, 0.8);
    let epsilon = 1.0e-7;
    let numerical =
        (arm.jacobian(&(q + epsilon * qd)) - arm.jacobian(&(q - epsilon * qd))) / (2.0 * epsilon);
    let analytical = arm.jacobian_dot(&q, &qd);
    assert_relative_eq!(analytical, numerical, epsilon = 2.0e-8);
    assert_relative_eq!(
        arm.jacobian_dot_times_velocity(&q, &qd).to_vector(),
        analytical * qd,
        epsilon = 1.0e-12
    );

    let mixed_arm = RobotArm::from_links(
        "mixed",
        [
            link(
                "rotation",
                JointKind::Revolute,
                [0.0; 3],
                [0.0; 3],
                [0.0, 0.0, 1.0],
                1.0,
                [0.2, 0.0, 0.0],
            ),
            link(
                "translation",
                JointKind::Prismatic,
                [1.0, 0.0, 0.0],
                [0.0; 3],
                [1.0, 0.0, 0.0],
                1.0,
                [0.1, 0.0, 0.0],
            ),
        ],
    );
    let mixed_q = JointVector::<2>::new(0.4, 0.2);
    let mixed_qd = JointVector::<2>::new(-0.3, 0.5);
    let mixed_numerical = (mixed_arm.jacobian(&(mixed_q + epsilon * mixed_qd))
        - mixed_arm.jacobian(&(mixed_q - epsilon * mixed_qd)))
        / (2.0 * epsilon);
    assert_relative_eq!(
        mixed_arm.jacobian_dot(&mixed_q, &mixed_qd),
        mixed_numerical,
        epsilon = 2.0e-8
    );
    assert_relative_eq!(
        mixed_arm
            .jacobian_dot_times_velocity(&mixed_q, &mixed_qd)
            .to_vector(),
        mixed_numerical * mixed_qd,
        epsilon = 2.0e-8
    );
    let mixed_qdd = JointVector::<2>::new(0.7, -0.4);
    assert_relative_eq!(
        mixed_arm
            .forward_acceleration_kinematics(&mixed_q, &mixed_qd, &mixed_qdd)
            .to_vector(),
        mixed_arm.jacobian(&mixed_q) * mixed_qdd + mixed_numerical * mixed_qd,
        epsilon = 2.0e-8
    );

    for q in [
        JointVector::<4>::new(0.0, FRAC_PI_2, 0.0, 0.0),
        JointVector::<4>::new(1.5708, 1.0472, -1.0472, 0.5236),
    ] {
        let numerical_jacobian_dot =
            (arm.jacobian(&(q + epsilon * q)) - arm.jacobian(&(q - epsilon * q))) / (2.0 * epsilon);
        let expected = arm.jacobian(&q) * q + numerical_jacobian_dot * q;
        let acceleration = arm.forward_acceleration_kinematics(&q, &q, &q);
        assert_relative_eq!(acceleration.to_vector(), expected, epsilon = 2.0e-8);
    }
}

#[test]
fn gravity_torque_matches_original_two_link_cases() {
    let arm = RobotArm::from_links(
        "test_gravity",
        [
            link(
                "link1",
                JointKind::Revolute,
                [0.0; 3],
                [0.0; 3],
                [0.0, 0.0, 1.0],
                1.0,
                [0.0, 0.5, 0.0],
            ),
            link(
                "link2",
                JointKind::Revolute,
                [1.0, 0.0, 0.0],
                [FRAC_PI_2, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                1.0,
                [0.0, 0.5, 0.0],
            ),
        ],
    );
    let q = JointVector::<2>::new(FRAC_PI_2, FRAC_PI_2);
    let (tau, _) = arm.gravity_torque(&q, &Frame::identity(), Wrench::zeros());
    assert_abs_diff_eq!(tau[0], 0.0, epsilon = 0.1);
    assert_abs_diff_eq!(tau[1], -4.903325, epsilon = 0.1);

    let vertical_base = Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_euler_angles(FRAC_PI_2, 0.0, 0.0),
    );
    let (tau, _) = arm.gravity_torque(&JointVector::zeros(), &vertical_base, Wrench::zeros());
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
        let (tau, _) = arm.inverse_dynamics(
            &q,
            &qd,
            &qdd,
            &Frame::identity(),
            Motion::zeros(),
            Motion::zeros(),
            Wrench::zeros(),
        );
        assert_relative_eq!(tau, expected, epsilon = epsilon);
    }
}

#[test]
fn joint_position_saturation_is_element_wise() {
    let lower = JointVector::<4>::new(-1.0, -2.0, -3.0, -4.0);
    let upper = JointVector::<4>::new(1.0, 2.0, 3.0, 4.0);
    let input = JointVector::<4>::new(-2.0, -1.0, 5.0, 3.0);
    assert_eq!(
        RobotArm::<4>::saturate_joint_position(&lower, &upper, &input),
        JointVector::<4>::new(-1.0, -1.0, 3.0, 3.0)
    );
}

#[derive(Clone, Copy, Debug)]
struct MimicMap;

impl PassiveJointMap<1, 2> for MimicMap {
    fn expand(&self, active: &JointVector<1>) -> JointVector<2> {
        JointVector::<2>::new(active[0], 2.0 * active[0])
    }

    fn reduce_force(&self, all: &JointVector<2>) -> JointVector<1> {
        // tau_active = A^T tau_all for q_all = A q_active.
        JointVector::<1>::new(all[0] + 2.0 * all[1])
    }
}

#[test]
fn passive_joint_adapter_expands_motion_and_reduces_force() {
    let arm = RobotArm::from_links(
        "mimic",
        [
            link(
                "active",
                JointKind::Revolute,
                [0.0; 3],
                [0.0; 3],
                [0.0, 0.0, 1.0],
                1.0,
                [0.5, 0.0, 0.0],
            ),
            link(
                "passive",
                JointKind::Revolute,
                [1.0, 0.0, 0.0],
                [0.0; 3],
                [0.0, 0.0, 1.0],
                1.0,
                [0.5, 0.0, 0.0],
            ),
        ],
    );
    let expected_arm = arm.clone();
    let passive = RobotWithPassiveJoints::new(arm, MimicMap);
    let q = JointVector::<1>::new(0.2);
    assert_relative_eq!(
        passive.forward_kinematics(&q),
        expected_arm.forward_kinematics(&MimicMap.expand(&q)),
        epsilon = 1.0e-12
    );

    let (all_force, _) =
        expected_arm.gravity_torque(&MimicMap.expand(&q), &Frame::identity(), Wrench::zeros());
    let (active_force, _) = passive.gravity_torque(&q, &Frame::identity(), Wrench::zeros());
    assert_relative_eq!(
        active_force,
        MimicMap.reduce_force(&all_force),
        epsilon = 1.0e-12
    );
}
