use super::*;

#[test]
fn floating_base_kinematics_and_dynamics_match_free_flyer_pinocchio() {
    let path = fixture();
    let mut robot = FloatingRobot::from_urdf(&path).unwrap();
    let target = robot.link_id("tool").unwrap();
    let mut pinocchio = PinocchioContext::new_floating(&robot, &path, "tool");

    for sample in 0..12 {
        let (q, qd, qdd) = deterministic_mixed_state(sample);
        let phase = sample as f64 + 1.0;
        let base = Frame::from_parts(
            Translation3::new(0.2, -0.3, 0.4),
            UnitQuaternion::from_euler_angles(
                0.3 * (phase * 0.23).sin(),
                -0.25 * (phase * 0.31).cos(),
                0.2 * (phase * 0.17).sin(),
            ),
        );
        let base_velocity = Twist::new(
            Vector3::new(0.21, -0.17, 0.13),
            Vector3::new(-0.3, 0.2, 0.1),
        );
        let base_acceleration = Twist::new(
            Vector3::new(-0.11, 0.14, 0.09),
            Vector3::new(0.35, -0.22, 0.18),
        );
        let base_state = dynibo::BaseState::new(base, base_velocity, base_acceleration).unwrap();
        let (pin_q, pin_qd, pin_qdd) =
            pinocchio.floating_state(&q, &qd, &qdd, &base, base_velocity, base_acceleration);

        let actual_frame = robot.forward_kinematics(&base_state, &q, target).unwrap();
        let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
        assert_close(
            actual_frame
                .rotation
                .to_rotation_matrix()
                .matrix()
                .as_slice(),
            expected_rotation.as_slice(),
            2.0e-11,
            1.0e-11,
            &format!("floating FK rotation sample {sample}"),
        );
        assert_close(
            actual_frame.translation.vector.as_slice(),
            expected_translation.as_slice(),
            2.0e-11,
            1.0e-11,
            &format!("floating FK translation sample {sample}"),
        );

        let mut actual_jacobian = vec![0.0; 6 * robot.generalized_count()];
        robot
            .jacobian(&base_state, &q, target, &mut actual_jacobian)
            .unwrap();
        assert_close(
            &actual_jacobian,
            &pinocchio.floating_jacobian(&pin_q, &base),
            2.0e-10,
            1.0e-10,
            &format!("floating Jacobian sample {sample}"),
        );
        let mut actual_derivative = vec![0.0; actual_jacobian.len()];
        robot
            .jacobian_derivative(&base_state, &q, &qd, target, &mut actual_derivative)
            .unwrap();
        assert_close(
            &actual_derivative,
            &pinocchio.floating_jacobian_derivative(&pin_q, &pin_qd, &base, base_velocity.angular),
            3.0e-9,
            1.0e-9,
            &format!("floating Jacobian derivative sample {sample}"),
        );

        assert_close(
            robot
                .forward_velocity_kinematics(&base_state, &q, &qd, target, &Frame::identity())
                .unwrap()
                .to_vector()
                .as_slice(),
            &pinocchio.velocity(&pin_q, &pin_qd),
            2.0e-10,
            1.0e-10,
            &format!("floating velocity sample {sample}"),
        );
        assert_close(
            robot
                .forward_acceleration_kinematics(&base_state, &q, &qd, &qdd, target)
                .unwrap()
                .to_vector()
                .as_slice(),
            &pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd),
            3.0e-9,
            1.0e-9,
            &format!("floating acceleration sample {sample}"),
        );

        let n = robot.generalized_count();
        let mut actual_mass = vec![0.0; n * n];
        robot
            .mass_matrix(&base_state, &q, &mut actual_mass)
            .unwrap();
        assert_close(
            &actual_mass,
            &pinocchio.floating_mass_matrix(&pin_q, &base),
            2.0e-9,
            1.0e-9,
            &format!("floating mass matrix sample {sample}"),
        );
        let mut actual_gravity = vec![0.0; n];
        robot
            .gravity(&base_state, &q, &[], &mut actual_gravity)
            .unwrap();
        assert_close(
            &actual_gravity,
            &pinocchio.floating_gravity(&pin_q, &base),
            2.0e-9,
            1.0e-9,
            &format!("floating gravity sample {sample}"),
        );
        let mut actual_velocity_product = vec![0.0; n];
        robot
            .velocity_product_forces(&base_state, &q, &qd, &mut actual_velocity_product)
            .unwrap();
        let coriolis = pinocchio.floating_coriolis_from_rnea(&q, &qd, &base, base_velocity);
        let generalized_velocity = [
            base_velocity.angular[0],
            base_velocity.angular[1],
            base_velocity.angular[2],
            base_velocity.linear[0],
            base_velocity.linear[1],
            base_velocity.linear[2],
            qd[0],
            qd[1],
            qd[2],
        ];
        let expected: Vec<f64> = (0..n)
            .map(|row| {
                (0..n)
                    .map(|column| coriolis[column * n + row] * generalized_velocity[column])
                    .sum()
            })
            .collect();
        assert_close(
            &actual_velocity_product,
            &expected,
            3.0e-9,
            1.0e-9,
            &format!("floating velocity product sample {sample}"),
        );
    }
}

#[test]
fn floating_aba_matches_free_flyer_pinocchio() {
    let path = fixture();
    let mut robot = FloatingRobot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new_floating(&robot, &path, "tool");

    for sample in 0..16 {
        let (q, qd, _) = deterministic_mixed_state(sample);
        let zero = [0.0; 3];
        let phase = sample as f64 + 1.0;
        let base = Frame::from_parts(
            Translation3::new(0.2, -0.3, 0.4),
            UnitQuaternion::from_euler_angles(
                0.3 * (phase * 0.23).sin(),
                -0.25 * (phase * 0.31).cos(),
                0.2 * (phase * 0.17).sin(),
            ),
        );
        let base_velocity = Twist::new(
            Vector3::new(0.21, -0.17, 0.13),
            Vector3::new(-0.3, 0.2, 0.1),
        );
        let ignored_acceleration =
            Twist::new(Vector3::new(4.0, -3.0, 2.0), Vector3::new(-5.0, 6.0, -7.0));
        let base_state = BaseState::new(base, base_velocity, ignored_acceleration).unwrap();
        let (pin_q, pin_qd, _) =
            pinocchio.floating_state(&q, &qd, &zero, &base, base_velocity, Twist::zeros());
        let generalized_forces: Vec<f64> = (0..robot.generalized_count())
            .map(|index| 7.0 * (phase * (index + 2) as f64 * 0.337).sin())
            .collect();
        let mut actual = vec![f64::NAN; robot.generalized_count()];
        robot
            .forward_dynamics(&base_state, &q, &qd, &generalized_forces, &[], &mut actual)
            .unwrap();
        let expected =
            pinocchio.floating_aba(&pin_q, &pin_qd, &generalized_forces, &base, base_velocity);
        assert_close(
            &actual,
            &expected,
            4.0e-9,
            1.0e-9,
            &format!("floating ABA sample {sample}"),
        );
    }
}

#[test]
fn floating_external_loads_match_free_flyer_pinocchio() {
    let path = fixture();
    let mut robot = FloatingRobot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new_floating(&robot, &path, "tool");

    for link_name in ["link_a", "slider_link", "tool"] {
        let wrench = Wrench::new(
            Vector3::new(0.31, -0.27, 0.19),
            Vector3::new(-0.8, 0.55, 0.42),
        );
        let dynibo_load = IndexedLoad {
            link: robot.link_id(link_name).unwrap(),
            wrench,
        };
        let pinocchio_load = pinocchio.load(link_name, wrench);

        for sample in 0..16 {
            let (q, qd, qdd) = deterministic_mixed_state(sample);
            let phase = sample as f64 + 1.0;
            let base = Frame::from_parts(
                Translation3::new(0.2, -0.3, 0.4),
                UnitQuaternion::from_euler_angles(
                    0.3 * (phase * 0.23).sin(),
                    -0.25 * (phase * 0.31).cos(),
                    0.2 * (phase * 0.17).sin(),
                ),
            );
            let base_velocity = Twist::new(
                Vector3::new(0.21, -0.17, 0.13),
                Vector3::new(-0.3, 0.2, 0.1),
            );
            let base_acceleration = Twist::new(
                Vector3::new(-0.11, 0.14, 0.09),
                Vector3::new(0.35, -0.22, 0.18),
            );
            let base_state = BaseState::new(base, base_velocity, base_acceleration).unwrap();
            let (pin_q, pin_qd, pin_qdd) =
                pinocchio.floating_state(&q, &qd, &qdd, &base, base_velocity, base_acceleration);

            let mut actual_gravity = vec![f64::NAN; robot.generalized_count()];
            let stationary_base = BaseState::new(base, Twist::zeros(), Twist::zeros()).unwrap();
            robot
                .gravity(&stationary_base, &q, &[dynibo_load], &mut actual_gravity)
                .unwrap();
            assert_close(
                &actual_gravity,
                &pinocchio.floating_gravity_with_loads(&pin_q, &base, &[pinocchio_load]),
                4.0e-9,
                1.0e-9,
                &format!("floating gravity load on {link_name}, sample {sample}"),
            );

            let mut actual_rnea = vec![f64::NAN; robot.generalized_count()];
            robot
                .inverse_dynamics(&base_state, &q, &qd, &qdd, &[dynibo_load], &mut actual_rnea)
                .unwrap();
            assert_close(
                &actual_rnea,
                &pinocchio.floating_rnea_with_loads(
                    &pin_q,
                    &pin_qd,
                    &pin_qdd,
                    &base,
                    &[pinocchio_load],
                ),
                4.0e-9,
                1.0e-9,
                &format!("floating RNEA load on {link_name}, sample {sample}"),
            );

            let generalized_forces: Vec<f64> = (0..robot.generalized_count())
                .map(|index| 7.0 * (phase * (index + 2) as f64 * 0.337).sin())
                .collect();
            let mut actual_aba = vec![f64::NAN; robot.generalized_count()];
            robot
                .forward_dynamics(
                    &base_state,
                    &q,
                    &qd,
                    &generalized_forces,
                    &[dynibo_load],
                    &mut actual_aba,
                )
                .unwrap();
            assert_close(
                &actual_aba,
                &pinocchio.floating_aba_with_loads(
                    &pin_q,
                    &pin_qd,
                    &generalized_forces,
                    &base,
                    base_velocity,
                    &[pinocchio_load],
                ),
                5.0e-9,
                1.0e-9,
                &format!("floating ABA load on {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn mixed_joint_moving_base_rnea_matches_free_flyer_pinocchio() {
    let path = fixture();
    let mut robot = FloatingRobot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new_floating(&robot, &path, "tool");

    for sample in 0..16 {
        let (q, qd, qdd) = deterministic_mixed_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let phase = sample as f64 + 1.0;
        let base = Frame::from_parts(
            Translation3::new(0.2, -0.3, 0.4),
            UnitQuaternion::from_euler_angles(
                0.3 * (phase * 0.23).sin(),
                -0.25 * (phase * 0.31).cos(),
                0.2 * (phase * 0.17).sin(),
            ),
        );
        let base_velocity = Twist::new(
            Vector3::new(0.21, -0.17, 0.13),
            Vector3::new(-0.3, 0.2, 0.1),
        );
        let base_acceleration = Twist::new(
            Vector3::new(-0.11, 0.14, 0.09),
            Vector3::new(0.35, -0.22, 0.18),
        );
        let base_state = dynibo::BaseState::new(base, base_velocity, base_acceleration).unwrap();
        let mut actual = [f64::NAN; 9];
        robot
            .inverse_dynamics(&base_state, &q, &qd, &qdd, &[], &mut actual)
            .unwrap();
        let expected = pinocchio.floating_rnea(
            &pin_q,
            &pin_qd,
            &pin_qdd,
            &base,
            base_velocity,
            base_acceleration,
        );
        assert_close(
            &actual,
            &expected,
            1.0e-9,
            1.0e-10,
            &format!("moving-base RNEA sample {sample}"),
        );
    }
}
