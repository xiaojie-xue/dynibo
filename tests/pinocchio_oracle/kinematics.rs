use super::*;

#[test]
fn serial_arm_calculations_match_pinocchio() {
    let path = serial_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let target = robot.link_id("test_link_4").unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let q = [0.2, 1.0, -0.7, 0.4];
    let qd = [-0.3, 0.5, -0.2, 0.8];
    let qdd = [0.7, -0.4, 0.1, 0.3];
    let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);

    let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
    let actual_frame = robot.forward_kinematics(&q, target).unwrap();
    assert_close(
        actual_frame
            .rotation
            .to_rotation_matrix()
            .matrix()
            .as_slice(),
        expected_rotation.as_slice(),
        1.0e-11,
        1.0e-11,
        "serial FK rotation",
    );
    assert_close(
        actual_frame.translation.vector.as_slice(),
        expected_translation.as_slice(),
        1.0e-11,
        1.0e-11,
        "serial FK translation",
    );

    let mut actual_jacobian = [f64::NAN; 24];
    robot.jacobian(&q, target, &mut actual_jacobian).unwrap();
    assert_close(
        &actual_jacobian,
        &pinocchio.jacobian(&pin_q),
        1.0e-10,
        1.0e-10,
        "serial Jacobian",
    );
    assert_close(
        robot
            .forward_velocity_kinematics(&q, &qd, target, &Frame::identity())
            .unwrap()
            .to_vector()
            .as_slice(),
        &pinocchio.velocity(&pin_q, &pin_qd),
        1.0e-10,
        1.0e-10,
        "serial velocity",
    );
    assert_close(
        robot
            .forward_acceleration_kinematics(&q, &qd, &qdd, target)
            .unwrap()
            .to_vector()
            .as_slice(),
        &pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd),
        1.0e-9,
        1.0e-10,
        "serial acceleration",
    );

    let mut actual_gravity = [f64::NAN; 4];
    robot.gravity(&q, &[], &mut actual_gravity).unwrap();
    assert_close(
        &actual_gravity,
        &pinocchio.gravity(&pin_q),
        1.0e-9,
        1.0e-10,
        "serial gravity",
    );
    let mut actual_torque = [f64::NAN; 4];
    robot
        .inverse_dynamics(&q, &qd, &qdd, &[], &mut actual_torque)
        .unwrap();
    assert_close(
        &actual_torque,
        &pinocchio.rnea(&pin_q, &pin_qd, &pin_qdd),
        1.0e-9,
        1.0e-10,
        "serial RNEA",
    );
}

#[test]
fn mixed_link_kinematics_match_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    assert_eq!(robot.joint_count(), 3);

    for link_name in ["base", "link_a", "mounted_link", "slider_link", "tool"] {
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..32 {
            let (q, qd, qdd) = deterministic_mixed_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
            let actual_frame = robot.forward_kinematics(&q, target).unwrap();
            assert_close(
                actual_frame
                    .rotation
                    .to_rotation_matrix()
                    .matrix()
                    .as_slice(),
                expected_rotation.as_slice(),
                1.0e-11,
                1.0e-11,
                &format!("FK rotation for {link_name}, sample {sample}"),
            );
            assert_close(
                actual_frame.translation.vector.as_slice(),
                expected_translation.as_slice(),
                1.0e-11,
                1.0e-11,
                &format!("FK translation for {link_name}, sample {sample}"),
            );

            let mut actual_jacobian = vec![f64::NAN; 6 * robot.joint_count()];
            robot.jacobian(&q, target, &mut actual_jacobian).unwrap();
            let expected_jacobian = pinocchio.jacobian(&pin_q);
            assert_close(
                &actual_jacobian,
                &expected_jacobian,
                1.0e-10,
                1.0e-10,
                &format!("Jacobian for {link_name}, sample {sample}"),
            );

            let actual_velocity = robot
                .forward_velocity_kinematics(&q, &qd, target, &Frame::identity())
                .unwrap();
            let expected_velocity = pinocchio.velocity(&pin_q, &pin_qd);
            assert_close(
                actual_velocity.to_vector().as_slice(),
                &expected_velocity,
                1.0e-10,
                1.0e-10,
                &format!("velocity for {link_name}, sample {sample}"),
            );

            let actual_acceleration = robot
                .forward_acceleration_kinematics(&q, &qd, &qdd, target)
                .unwrap();
            let expected_acceleration = pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd);
            assert_close(
                actual_acceleration.to_vector().as_slice(),
                &expected_acceleration,
                1.0e-9,
                1.0e-10,
                &format!("acceleration for {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn jacobian_time_variations_match_pinocchio() {
    let path = serial_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let target = robot.link_id("test_link_4").unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let zero4 = [0.0; 4];
    for sample in 0..32 {
        let (q, qd, _) = deterministic_state(sample);
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero4);
        let mut derivative = vec![f64::NAN; 24];
        robot
            .jacobian_derivative(&q, &qd, target, &mut derivative)
            .unwrap();
        assert_close(
            &derivative,
            &pinocchio.jacobian_derivative(&pin_q, &pin_qd),
            1.0e-9,
            1.0e-10,
            &format!("serial Jacobian derivative sample {sample}"),
        );
    }

    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    for link_name in ["link_a", "mounted_link", "slider_link", "tool"] {
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..32 {
            let (q, qd, _) = deterministic_mixed_state(sample);
            let zero = [0.0; 3];
            let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero);
            let mut derivative = vec![f64::NAN; 18];
            robot
                .jacobian_derivative(&q, &qd, target, &mut derivative)
                .unwrap();
            assert_close(
                &derivative,
                &pinocchio.jacobian_derivative(&pin_q, &pin_qd),
                1.0e-9,
                1.0e-10,
                &format!("tree Jacobian derivative for {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn every_branched_link_frame_and_jacobian_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();

    for link_index in 0..robot.link_count() {
        let target = robot.link_id_at(link_index).unwrap();
        let link_name = robot.link_name(target).unwrap().to_owned();
        let mut pinocchio = PinocchioContext::new(&robot, &path, &link_name);
        for sample in 0..16 {
            let (q, qd, qdd) = deterministic_tree_state(sample);
            let (pin_q, _, _) = pinocchio.state(&q, &qd, &qdd);
            let (expected_rotation, expected_translation) = pinocchio.frame(&pin_q);
            let actual_frame = robot.forward_kinematics(&q, target).unwrap();
            assert_close(
                actual_frame
                    .rotation
                    .to_rotation_matrix()
                    .matrix()
                    .as_slice(),
                expected_rotation.as_slice(),
                1.0e-11,
                1.0e-11,
                &format!("tree FK rotation for {link_name}, sample {sample}"),
            );
            assert_close(
                actual_frame.translation.vector.as_slice(),
                expected_translation.as_slice(),
                1.0e-11,
                1.0e-11,
                &format!("tree FK translation for {link_name}, sample {sample}"),
            );
            let mut actual_jacobian = vec![f64::NAN; 6 * robot.joint_count()];
            robot.jacobian(&q, target, &mut actual_jacobian).unwrap();
            let expected_jacobian = pinocchio.jacobian(&pin_q);
            assert_close(
                &actual_jacobian,
                &expected_jacobian,
                1.0e-10,
                1.0e-10,
                &format!("tree Jacobian for {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn branched_velocity_and_acceleration_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();

    for link_name in ["left_tool", "right_tool"] {
        let target = robot.link_id(link_name).unwrap();
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..32 {
            let (q, qd, qdd) = deterministic_tree_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let velocity = robot
                .forward_velocity_kinematics(&q, &qd, target, &Frame::identity())
                .unwrap();
            assert_close(
                velocity.to_vector().as_slice(),
                &pinocchio.velocity(&pin_q, &pin_qd),
                1.0e-10,
                1.0e-10,
                &format!("tree velocity for {link_name}, sample {sample}"),
            );
            let acceleration = robot
                .forward_acceleration_kinematics(&q, &qd, &qdd, target)
                .unwrap();
            assert_close(
                acceleration.to_vector().as_slice(),
                &pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd),
                1.0e-9,
                1.0e-10,
                &format!("tree acceleration for {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn mixed_joint_ik_reaches_pinocchio_generated_targets() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let target = robot.link_id("tool").unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");
    let options = InverseKinematicsOptions {
        max_iterations: 200,
        damping: 2.0e-3,
        max_step_norm: 0.25,
        ..InverseKinematicsOptions::default()
    };

    for sample in 0..16 {
        let phase = (sample + 1) as f64;
        let target_q = [
            1.0 * (phase * 0.47).sin(),
            0.22 * (phase * 0.31).sin(),
            1.7 * (phase * 0.59).sin(),
        ];
        let zero = [0.0; 3];
        let (pin_target_q, _, _) = pinocchio.state(&target_q, &zero, &zero);
        let (rotation, translation) = pinocchio.frame(&pin_target_q);
        let desired = Frame::from_parts(
            Translation3::from(translation),
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation)),
        );
        let initial = [
            target_q[0] + 0.12 * (phase * 0.73).sin(),
            target_q[1] + 0.05 * (phase * 0.41).cos(),
            target_q[2] - 0.15 * (phase * 0.37).sin(),
        ];
        let mut solution = [f64::NAN; 3];
        robot
            .inverse_kinematics(&initial, target, &desired, options, &mut solution)
            .unwrap_or_else(|error| panic!("IK failed for sample {sample}: {error}"));

        let (pin_solution_q, _, _) = pinocchio.state(&solution, &zero, &zero);
        let (solved_rotation, solved_translation) = pinocchio.frame(&pin_solution_q);
        assert_close(
            solved_rotation.as_slice(),
            desired.rotation.to_rotation_matrix().matrix().as_slice(),
            2.0e-6,
            1.0e-10,
            &format!("IK rotation sample {sample}"),
        );
        assert_close(
            solved_translation.as_slice(),
            desired.translation.vector.as_slice(),
            2.0e-6,
            1.0e-10,
            &format!("IK translation sample {sample}"),
        );
    }
}
