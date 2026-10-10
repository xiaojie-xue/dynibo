use super::*;

#[test]
fn jacobian_agrees_with_forward_kinematics_finite_difference() {
    let mut arm = test_arm();
    let q = JointVector::<4>::new(0.147607, 1.014764, -1.840751, 0.825987);
    let jacobian = arm.test_jacobian(&q, end_link(&arm)).unwrap();
    let epsilon = 1.0e-7;

    for joint in 0..4 {
        let mut q_plus = q;
        let mut q_minus = q;
        q_plus[joint] += epsilon;
        q_minus[joint] -= epsilon;
        let plus = arm
            .test_forward_kinematics(&q_plus, end_link(&arm))
            .unwrap();
        let minus = arm
            .test_forward_kinematics(&q_minus, end_link(&arm))
            .unwrap();
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
    let mut arm = test_arm();
    let q = JointVector::<4>::new(0.205506, 1.443005, -2.645997, 1.202992);
    let expected = nalgebra::SMatrix::<f64, 6, 4>::from_row_slice(&[
        -0.0000, 0.2041, 0.2041, 0.2041, -0.0000, -0.9790, -0.9790, -0.9790, 1.0000, 0.0000,
        0.0000, 0.0000, -0.0303, -0.0367, 0.2740, 0.0, 0.1455, -0.0076, 0.0571, 0.0, 0.0, 0.1487,
        0.1079, 0.0,
    ]);
    assert_relative_eq!(
        arm.test_jacobian(&q, end_link(&arm)).unwrap(),
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
        arm.test_jacobian(&q, end_link(&arm)).unwrap(),
        expected,
        epsilon = 5.0e-4
    );
}

#[test]
fn velocity_is_jacobian_times_joint_velocity() {
    let mut arm = test_arm();
    let q = JointVector::<4>::new(PI / 12.0, PI / 3.0, -PI / 2.0, PI / 6.0);
    let qd = q;
    let velocity = arm
        .test_forward_velocity_kinematics(
            &q,
            &qd,
            end_link(&arm),
            &Frame::identity(),
            &Frame::identity(),
        )
        .unwrap();
    assert_relative_eq!(
        velocity.to_vector(),
        arm.test_jacobian(&q, end_link(&arm)).unwrap() * qd,
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
    let end = arm.test_forward_kinematics(&q, end_link(&arm)).unwrap();
    let mut tool_jacobian = arm.test_jacobian(&q, end_link(&arm)).unwrap();
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
        arm.test_forward_velocity_kinematics(&q, &qd, end_link(&arm), &base, &tool)
            .unwrap()
            .to_vector(),
        expected_with_frames.to_vector(),
        epsilon = 1.0e-12
    );
}

#[test]
fn forward_acceleration_matches_finite_difference() {
    let mut arm = test_arm();
    let q = JointVector::<4>::new(0.2, 1.1, -0.7, 0.4);
    let qd = JointVector::<4>::new(-0.3, 0.5, -0.2, 0.8);
    let epsilon = 1.0e-7;
    let numerical = (arm
        .test_jacobian(&(q + epsilon * qd), end_link(&arm))
        .unwrap()
        - arm
            .test_jacobian(&(q - epsilon * qd), end_link(&arm))
            .unwrap())
        / (2.0 * epsilon);
    let qdd = JointVector::<4>::new(0.7, -0.4, 0.1, 0.3);
    assert_relative_eq!(
        arm.test_forward_acceleration_kinematics(&q, &qd, &qdd, end_link(&arm))
            .unwrap()
            .to_vector(),
        arm.test_jacobian(&q, end_link(&arm)).unwrap() * qdd + numerical * qd,
        epsilon = 2.0e-8
    );

    let mut mixed_arm = Robot::from_urdf(urdf_path("mixed_arm.urdf")).unwrap();
    let mixed_q = JointVector::<2>::new(0.4, 0.2);
    let mixed_qd = JointVector::<2>::new(-0.3, 0.5);
    let mixed_numerical = (mixed_arm
        .test_jacobian(&(mixed_q + epsilon * mixed_qd), end_link(&mixed_arm))
        .unwrap()
        - mixed_arm
            .test_jacobian(&(mixed_q - epsilon * mixed_qd), end_link(&mixed_arm))
            .unwrap())
        / (2.0 * epsilon);
    let mixed_qdd = JointVector::<2>::new(0.7, -0.4);
    assert_relative_eq!(
        mixed_arm
            .test_forward_acceleration_kinematics(
                &mixed_q,
                &mixed_qd,
                &mixed_qdd,
                end_link(&mixed_arm),
            )
            .unwrap()
            .to_vector(),
        mixed_arm
            .test_jacobian(&mixed_q, end_link(&mixed_arm))
            .unwrap()
            * mixed_qdd
            + mixed_numerical * mixed_qd,
        epsilon = 2.0e-8
    );

    for q in [
        JointVector::<4>::new(0.0, FRAC_PI_2, 0.0, 0.0),
        JointVector::<4>::new(1.5708, 1.0472, -1.0472, 0.5236),
    ] {
        let numerical_jacobian_dot = (arm
            .test_jacobian(&(q + epsilon * q), end_link(&arm))
            .unwrap()
            - arm
                .test_jacobian(&(q - epsilon * q), end_link(&arm))
                .unwrap())
            / (2.0 * epsilon);
        let expected =
            arm.test_jacobian(&q, end_link(&arm)).unwrap() * q + numerical_jacobian_dot * q;
        let acceleration = arm
            .test_forward_acceleration_kinematics(&q, &q, &q, end_link(&arm))
            .unwrap();
        assert_relative_eq!(acceleration.to_vector(), expected, epsilon = 2.0e-8);
    }
}
