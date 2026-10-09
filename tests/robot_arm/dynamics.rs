use super::*;

#[test]
fn two_link_gravity_matches_closed_form_oracle() {
    let mut arm = Robot::from_urdf(urdf_path("gravity_arm.urdf")).unwrap();
    let q = JointVector::<2>::new(FRAC_PI_2, FRAC_PI_2);
    let tau = arm.test_gravity(&q, &Frame::identity(), &[]).unwrap();
    assert_relative_eq!(
        tau,
        JointVector::<2>::new(0.0, -0.5 * STANDARD_GRAVITY),
        epsilon = 2.0e-12
    );

    let vertical_base = Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_euler_angles(FRAC_PI_2, 0.0, 0.0),
    );
    let tau = arm
        .test_gravity(&JointVector::<2>::zeros(), &vertical_base, &[])
        .unwrap();
    assert_relative_eq!(
        tau,
        JointVector::<2>::new(STANDARD_GRAVITY, 0.0),
        epsilon = 2.0e-12
    );
}

#[test]
fn single_revolute_pendulum_matches_closed_form_oracle() {
    let mut arm = Robot::from_urdf(urdf_path("single_revolute.urdf")).unwrap();
    let target = end_link(&arm);
    let mass = 2.0;
    let center_distance = 0.3;
    let center_inertia = 0.05;
    let joint_inertia = center_inertia + mass * center_distance * center_distance;

    for (q, qd, qdd) in [
        (PI / 3.0, 0.8, 0.7),
        (-PI / 4.0, -0.6, -0.2),
        (0.0, 0.0, 0.0),
    ] {
        let position = JointVector::<1>::new(q);
        let velocity = JointVector::<1>::new(qd);
        let acceleration = JointVector::<1>::new(qdd);

        assert_relative_eq!(
            arm.test_forward_kinematics(&position, target).unwrap(),
            Frame::rotation(Vector3::new(0.0, q, 0.0)),
            epsilon = 2.0e-12
        );
        assert_relative_eq!(
            arm.test_jacobian(&position, target).unwrap(),
            Jacobian::<1>::from_column_slice(&[0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            epsilon = 2.0e-12
        );
        assert_relative_eq!(
            arm.test_forward_velocity_kinematics(
                &position,
                &velocity,
                target,
                &Frame::identity(),
                &Frame::identity(),
            )
            .unwrap()
            .to_vector(),
            SVector::<f64, 6>::new(0.0, qd, 0.0, 0.0, 0.0, 0.0),
            epsilon = 2.0e-12
        );
        assert_relative_eq!(
            arm.test_forward_acceleration_kinematics(&position, &velocity, &acceleration, target,)
                .unwrap()
                .to_vector(),
            SVector::<f64, 6>::new(0.0, qdd, 0.0, 0.0, 0.0, 0.0),
            epsilon = 2.0e-12
        );

        let gravity_torque = -mass * STANDARD_GRAVITY * center_distance * q.cos();
        assert_relative_eq!(
            arm.test_gravity(&position, &Frame::identity(), &[])
                .unwrap(),
            JointVector::<1>::new(gravity_torque),
            epsilon = 2.0e-12
        );
        assert_relative_eq!(
            arm.test_inverse_dynamics(&position, &velocity, &acceleration, &[],)
                .unwrap(),
            JointVector::<1>::new(gravity_torque + joint_inertia * qdd),
            epsilon = 2.0e-12
        );
    }
}

#[test]
fn revolute_prismatic_arm_matches_closed_form_oracle() {
    let mut arm = Robot::from_urdf(urdf_path("mixed_arm.urdf")).unwrap();
    let target = end_link(&arm);
    let q = JointVector::<2>::new(PI / 6.0, 0.25);
    let qd = JointVector::<2>::new(0.4, -0.3);
    let qdd = JointVector::<2>::new(-0.2, 0.7);
    let (sin_theta, cos_theta) = q[0].sin_cos();
    let target_radius = 1.0 + q[1];

    let expected_frame = Frame::from_parts(
        Translation3::new(target_radius * cos_theta, target_radius * sin_theta, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), q[0]),
    );
    assert_relative_eq!(
        arm.test_forward_kinematics(&q, target).unwrap(),
        expected_frame,
        epsilon = 2.0e-12
    );

    let expected_jacobian = Jacobian::<2>::from_columns(&[
        SVector::<f64, 6>::new(
            0.0,
            0.0,
            1.0,
            -target_radius * sin_theta,
            target_radius * cos_theta,
            0.0,
        ),
        SVector::<f64, 6>::new(0.0, 0.0, 0.0, cos_theta, sin_theta, 0.0),
    ]);
    assert_relative_eq!(
        arm.test_jacobian(&q, target).unwrap(),
        expected_jacobian,
        epsilon = 2.0e-12
    );

    let expected_velocity = Twist::new(
        Vector3::new(0.0, 0.0, qd[0]),
        Vector3::new(
            qd[1] * cos_theta - target_radius * qd[0] * sin_theta,
            qd[1] * sin_theta + target_radius * qd[0] * cos_theta,
            0.0,
        ),
    );
    assert_relative_eq!(
        arm.test_forward_velocity_kinematics(
            &q,
            &qd,
            target,
            &Frame::identity(),
            &Frame::identity(),
        )
        .unwrap()
        .to_vector(),
        expected_velocity.to_vector(),
        epsilon = 2.0e-12
    );

    let radial_acceleration = qdd[1] - target_radius * qd[0].powi(2);
    let tangential_acceleration = target_radius * qdd[0] + 2.0 * qd[0] * qd[1];
    let expected_acceleration = Twist::new(
        Vector3::new(0.0, 0.0, qdd[0]),
        Vector3::new(
            radial_acceleration * cos_theta - tangential_acceleration * sin_theta,
            radial_acceleration * sin_theta + tangential_acceleration * cos_theta,
            0.0,
        ),
    );
    assert_relative_eq!(
        arm.test_forward_acceleration_kinematics(&q, &qd, &qdd, target)
            .unwrap()
            .to_vector(),
        expected_acceleration.to_vector(),
        epsilon = 2.0e-12
    );

    assert_relative_eq!(
        arm.test_gravity(&q, &Frame::identity(), &[]).unwrap(),
        JointVector::<2>::zeros(),
        epsilon = 2.0e-12
    );

    // The first link COM is 0.2 m from the revolute joint. The second link COM
    // is 0.1 m beyond the prismatic joint, so its radius is 1.1 + q[1]. Both
    // links have unit mass and Izz = 0.01 kg m^2.
    let second_com_radius = 1.1 + q[1];
    let rotational_inertia = 0.01 + 0.2_f64.powi(2) + 0.01 + second_com_radius.powi(2);
    let expected_torque = JointVector::<2>::new(
        rotational_inertia * qdd[0] + 2.0 * second_com_radius * qd[0] * qd[1],
        qdd[1] - second_com_radius * qd[0].powi(2),
    );
    assert_relative_eq!(
        arm.test_inverse_dynamics(&q, &qd, &qdd, &[],).unwrap(),
        expected_torque,
        epsilon = 2.0e-12
    );
}

#[test]
fn inverse_dynamics_matches_test_arm_numeric_reference() {
    let mut arm = test_arm();
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
        let tau = arm.test_inverse_dynamics(&q, &qd, &qdd, &[]).unwrap();
        assert_relative_eq!(tau, expected, epsilon = epsilon);
    }
}

#[test]
fn prismatic_inverse_dynamics_projects_linear_inertia_onto_its_axis() {
    let mut arm = Robot::from_urdf(urdf_path("mixed_arm.urdf")).unwrap();
    let zero = JointVector::<2>::zeros();
    let gravity = arm.test_gravity(&zero, &Frame::identity(), &[]).unwrap();
    let accelerated = arm
        .test_inverse_dynamics(&zero, &zero, &JointVector::<2>::new(0.0, 1.0), &[])
        .unwrap();

    assert_relative_eq!(
        accelerated - gravity,
        JointVector::<2>::new(0.0, 1.0),
        epsilon = 1.0e-12
    );
}

#[test]
fn tree_external_loads_are_isolated_and_add_linearly() {
    let mut arm = tree_arm();
    let q = JointVector::<7>::from_row_slice(&[0.2, -0.3, 0.4, -0.5, 0.6, -0.7, 0.8]);
    let left = Load {
        link: arm.link_id("left_tool").unwrap(),
        wrench: Wrench::new(Vector3::new(0.3, -0.2, 0.4), Vector3::new(1.0, 0.5, -0.7)),
    };
    let right = Load {
        link: arm.link_id("right_tool").unwrap(),
        wrench: Wrench::new(Vector3::new(-0.4, 0.1, 0.2), Vector3::new(-0.6, 0.8, 0.3)),
    };

    let baseline = arm.test_gravity(&q, &Frame::identity(), &[]).unwrap();
    let left_only = arm.test_gravity(&q, &Frame::identity(), &[left]).unwrap();
    let right_only = arm.test_gravity(&q, &Frame::identity(), &[right]).unwrap();
    let both = arm
        .test_gravity(&q, &Frame::identity(), &[left, right])
        .unwrap();

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
        let frame = arm.test_forward_kinematics(&q, external.link).unwrap();
        let torque = frame.rotation * external.wrench.torque;
        let force = frame.rotation * external.wrench.force;
        let wrench_in_base =
            SVector::<f64, 6>::from_iterator(torque.iter().chain(force.iter()).copied());
        let expected = arm.test_jacobian(&q, external.link).unwrap().transpose() * wrench_in_base;
        assert_relative_eq!(with_load - baseline, expected, epsilon = 2.0e-12);
    }
    assert_relative_eq!(both, left_only + right_only - baseline, epsilon = 2.0e-12);
}
