#![cfg(feature = "pinocchio-tests")]

#[path = "support/pinocchio.rs"]
mod pinocchio;

use std::path::PathBuf;

use dynibo::{
    BaseMode, BaseState, Frame, IndexedLoad, InverseKinematicsOptions, Robot, Twist, Wrench,
};
use nalgebra::{Rotation3, Translation3, UnitQuaternion, Vector3};
use pinocchio::PinocchioContext;

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/oracle_mixed.urdf")
}

fn tree_fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf")
}

fn deterministic_state(sample: usize) -> ([f64; 4], [f64; 4], [f64; 4]) {
    let wave =
        |joint: usize, phase: f64| ((sample + 1) as f64 * (joint + 2) as f64 * 0.619 + phase).sin();
    (
        [
            1.4 * wave(0, 0.1),
            5.0 * wave(1, 0.2),
            0.3 * wave(2, 0.3),
            2.7 * wave(3, 0.4),
        ],
        std::array::from_fn(|joint| 0.8 * wave(joint, 0.9)),
        std::array::from_fn(|joint| 1.1 * wave(joint, 1.7)),
    )
}

fn deterministic_mixed_state(sample: usize) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let (q, qd, qdd) = deterministic_state(sample);
    (
        [q[0], q[2], q[3]],
        [qd[0], qd[2], qd[3]],
        [qdd[0], qdd[2], qdd[3]],
    )
}

fn deterministic_tree_state(sample: usize) -> ([f64; 7], [f64; 7], [f64; 7]) {
    let values = |phase: f64, amplitude: f64| {
        std::array::from_fn(|joint| {
            let argument = (sample + 1) as f64 * (joint + 3) as f64 * 0.731 + phase;
            amplitude * argument.sin()
        })
    };
    (values(0.0, 0.9), values(0.7, 0.8), values(1.3, 1.1))
}

fn serial_fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf")
}

fn assert_close(actual: &[f64], expected: &[f64], absolute: f64, relative: f64, context: &str) {
    assert_eq!(actual.len(), expected.len());
    for (index, (&actual, &expected)) in actual.iter().zip(expected).enumerate() {
        let tolerance = absolute + relative * actual.abs().max(expected.abs());
        assert!(
            (actual - expected).abs() <= tolerance,
            "{context}: element {index}: actual={actual:.16e}, expected={expected:.16e}, tolerance={tolerance:.3e}"
        );
    }
}

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
    let actual_frame = robot
        .forward_kinematics(&dynibo::BaseState::fixed(), &q, target)
        .unwrap();
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
    robot
        .jacobian(
            &dynibo::BaseState::fixed(),
            &q,
            target,
            &mut actual_jacobian,
        )
        .unwrap();
    assert_close(
        &actual_jacobian,
        &pinocchio.jacobian(&pin_q),
        1.0e-10,
        1.0e-10,
        "serial Jacobian",
    );
    assert_close(
        robot
            .forward_velocity_kinematics(
                &dynibo::BaseState::fixed(),
                &q,
                &qd,
                target,
                &Frame::identity(),
            )
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
            .forward_acceleration_kinematics(&dynibo::BaseState::fixed(), &q, &qd, &qdd, target)
            .unwrap()
            .to_vector()
            .as_slice(),
        &pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd),
        1.0e-9,
        1.0e-10,
        "serial acceleration",
    );

    let mut actual_gravity = [f64::NAN; 4];
    robot
        .gravity(&dynibo::BaseState::fixed(), &q, &[], &mut actual_gravity)
        .unwrap();
    assert_close(
        &actual_gravity,
        &pinocchio.gravity(&pin_q),
        1.0e-9,
        1.0e-10,
        "serial gravity",
    );
    let mut actual_torque = [f64::NAN; 4];
    robot
        .inverse_dynamics(
            &dynibo::BaseState::fixed(),
            &q,
            &qd,
            &qdd,
            &[],
            &mut actual_torque,
        )
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
            let actual_frame = robot
                .forward_kinematics(&dynibo::BaseState::fixed(), &q, target)
                .unwrap();
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
            robot
                .jacobian(
                    &dynibo::BaseState::fixed(),
                    &q,
                    target,
                    &mut actual_jacobian,
                )
                .unwrap();
            let expected_jacobian = pinocchio.jacobian(&pin_q);
            assert_close(
                &actual_jacobian,
                &expected_jacobian,
                1.0e-10,
                1.0e-10,
                &format!("Jacobian for {link_name}, sample {sample}"),
            );

            let actual_velocity = robot
                .forward_velocity_kinematics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    target,
                    &Frame::identity(),
                )
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
                .forward_acceleration_kinematics(&dynibo::BaseState::fixed(), &q, &qd, &qdd, target)
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
fn mass_matrices_match_pinocchio() {
    let path = serial_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let zero4 = [0.0; 4];
    for sample in 0..32 {
        let (q, _, _) = deterministic_state(sample);
        let (pin_q, _, _) = pinocchio.state(&q, &zero4, &zero4);
        let mut mass = vec![f64::NAN; 16];
        robot
            .mass_matrix(&dynibo::BaseState::fixed(), &q, &mut mass)
            .unwrap();
        assert_close(
            &mass,
            &pinocchio.mass_matrix(&pin_q),
            1.0e-9,
            1.0e-10,
            &format!("serial mass matrix sample {sample}"),
        );
    }

    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");
    let zero3 = [0.0; 3];
    for sample in 0..64 {
        let (q, _, _) = deterministic_mixed_state(sample);
        let (pin_q, _, _) = pinocchio.state(&q, &zero3, &zero3);
        let mut mass = vec![f64::NAN; 9];
        robot
            .mass_matrix(&dynibo::BaseState::fixed(), &q, &mut mass)
            .unwrap();
        assert_close(
            &mass,
            &pinocchio.mass_matrix(&pin_q),
            1.0e-9,
            1.0e-10,
            &format!("mixed mass matrix sample {sample}"),
        );
    }

    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "right_tool");
    let zero7 = [0.0; 7];
    for sample in 0..32 {
        let (q, _, _) = deterministic_tree_state(sample);
        let (pin_q, _, _) = pinocchio.state(&q, &zero7, &zero7);
        let mut mass = vec![f64::NAN; 49];
        robot
            .mass_matrix(&dynibo::BaseState::fixed(), &q, &mut mass)
            .unwrap();
        assert_close(
            &mass,
            &pinocchio.mass_matrix(&pin_q),
            1.0e-9,
            1.0e-10,
            &format!("tree mass matrix sample {sample}"),
        );
    }
}

#[test]
fn velocity_products_match_pinocchio() {
    let path = serial_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let zero4 = [0.0; 4];
    for sample in 0..32 {
        let (q, qd, _) = deterministic_state(sample);
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero4);
        let mut velocity_product = vec![f64::NAN; 4];
        robot
            .velocity_product_forces(&dynibo::BaseState::fixed(), &q, &qd, &mut velocity_product)
            .unwrap();
        let coriolis = pinocchio.coriolis_matrix(&pin_q, &pin_qd);
        let expected: Vec<f64> = (0..4)
            .map(|row| {
                (0..4)
                    .map(|column| coriolis[column * 4 + row] * qd[column])
                    .sum()
            })
            .collect();
        assert_close(
            &velocity_product,
            &expected,
            1.0e-9,
            1.0e-10,
            &format!("serial velocity product sample {sample}"),
        );
    }

    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");
    for sample in 0..64 {
        let (q, qd, _) = deterministic_mixed_state(sample);
        let zero = [0.0; 3];
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero);
        let mut velocity_product = vec![f64::NAN; 3];
        robot
            .velocity_product_forces(&dynibo::BaseState::fixed(), &q, &qd, &mut velocity_product)
            .unwrap();
        let coriolis = pinocchio.coriolis_matrix(&pin_q, &pin_qd);
        let expected: Vec<f64> = (0..3)
            .map(|row| {
                (0..3)
                    .map(|column| coriolis[column * 3 + row] * qd[column])
                    .sum()
            })
            .collect();
        assert_close(
            &velocity_product,
            &expected,
            1.0e-9,
            1.0e-10,
            &format!("mixed velocity product sample {sample}"),
        );
    }

    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "right_tool");
    let zero7 = [0.0; 7];
    for sample in 0..32 {
        let (q, qd, _) = deterministic_tree_state(sample);
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero7);
        let mut velocity_product = vec![f64::NAN; 7];
        robot
            .velocity_product_forces(&dynibo::BaseState::fixed(), &q, &qd, &mut velocity_product)
            .unwrap();
        let coriolis = pinocchio.coriolis_matrix(&pin_q, &pin_qd);
        let expected: Vec<f64> = (0..7)
            .map(|row| {
                (0..7)
                    .map(|column| coriolis[column * 7 + row] * qd[column])
                    .sum()
            })
            .collect();
        assert_close(
            &velocity_product,
            &expected,
            1.0e-9,
            1.0e-10,
            &format!("tree velocity product sample {sample}"),
        );
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
            .jacobian_derivative(
                &dynibo::BaseState::fixed(),
                &q,
                &qd,
                target,
                &mut derivative,
            )
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
                .jacobian_derivative(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    target,
                    &mut derivative,
                )
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
fn mixed_joint_gravity_and_rnea_match_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");

    for sample in 0..64 {
        let (q, qd, qdd) = deterministic_mixed_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let mut actual_gravity = [f64::NAN; 3];
        robot
            .gravity(&dynibo::BaseState::fixed(), &q, &[], &mut actual_gravity)
            .unwrap();
        let expected_gravity = pinocchio.gravity(&pin_q);
        assert_close(
            &actual_gravity,
            &expected_gravity,
            1.0e-9,
            1.0e-10,
            &format!("gravity sample {sample}"),
        );

        let mut actual_torque = [f64::NAN; 3];
        robot
            .inverse_dynamics(
                &dynibo::BaseState::fixed(),
                &q,
                &qd,
                &qdd,
                &[],
                &mut actual_torque,
            )
            .unwrap();
        let expected_torque = pinocchio.rnea(&pin_q, &pin_qd, &pin_qdd);
        assert_close(
            &actual_torque,
            &expected_torque,
            1.0e-9,
            1.0e-10,
            &format!("RNEA sample {sample}"),
        );
    }
}

#[test]
fn mixed_joint_aba_matches_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");

    for sample in 0..32 {
        let (q, qd, _) = deterministic_mixed_state(sample);
        let zero = [0.0; 3];
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero);
        let torque: [f64; 3] = std::array::from_fn(|joint| {
            let phase = (sample + 1) as f64 * (joint + 2) as f64 * 0.413;
            8.0 * phase.sin()
        });
        let mut actual = [f64::NAN; 3];
        robot
            .forward_dynamics(&BaseState::fixed(), &q, &qd, &torque, &[], &mut actual)
            .unwrap();
        let expected = pinocchio.aba(&pin_q, &pin_qd, &torque);
        assert_close(
            &actual,
            &expected,
            2.0e-9,
            2.0e-10,
            &format!("ABA sample {sample}"),
        );
    }
}

#[test]
fn mixed_joint_aba_with_external_loads_matches_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let load = Wrench::new(
        Vector3::new(0.31, -0.27, 0.19),
        Vector3::new(-0.8, 0.55, 0.42),
    );

    for link_name in ["link_a", "slider_link", "tool"] {
        let indexed_load = IndexedLoad {
            link: robot.link_id(link_name).unwrap(),
            wrench: load,
        };
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..16 {
            let (q, qd, _) = deterministic_mixed_state(sample);
            let zero = [0.0; 3];
            let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero);
            let torque: [f64; 3] = std::array::from_fn(|joint| {
                let phase = (sample + 1) as f64 * (joint + 2) as f64 * 0.419;
                8.0 * phase.sin()
            });
            let mut actual = [f64::NAN; 3];
            robot
                .forward_dynamics(
                    &BaseState::fixed(),
                    &q,
                    &qd,
                    &torque,
                    &[indexed_load],
                    &mut actual,
                )
                .unwrap();
            let expected = pinocchio.aba_with_link_load(&pin_q, &pin_qd, &torque, load);
            assert_close(
                &actual,
                &expected,
                2.0e-9,
                2.0e-10,
                &format!("ABA external load on {link_name}, sample {sample}"),
            );
        }
    }
}

#[test]
fn mixed_joint_external_loads_match_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let load = Wrench::new(
        Vector3::new(0.31, -0.27, 0.19),
        Vector3::new(-0.8, 0.55, 0.42),
    );

    for link_name in ["link_a", "mounted_link", "slider_link", "tool"] {
        let target = robot.link_id(link_name).unwrap();
        let indexed_load = IndexedLoad {
            link: target,
            wrench: load,
        };
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..16 {
            let (q, qd, qdd) = deterministic_mixed_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let mut actual = [f64::NAN; 3];
            robot
                .inverse_dynamics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    &qdd,
                    &[indexed_load],
                    &mut actual,
                )
                .unwrap();
            let expected = pinocchio.rnea_with_link_load(&pin_q, &pin_qd, &pin_qdd, load);
            assert_close(
                &actual,
                &expected,
                1.0e-9,
                1.0e-10,
                &format!("external load on {link_name}, sample {sample}"),
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
            let actual_frame = robot
                .forward_kinematics(&dynibo::BaseState::fixed(), &q, target)
                .unwrap();
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
            robot
                .jacobian(
                    &dynibo::BaseState::fixed(),
                    &q,
                    target,
                    &mut actual_jacobian,
                )
                .unwrap();
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
                .forward_velocity_kinematics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    target,
                    &Frame::identity(),
                )
                .unwrap();
            assert_close(
                velocity.to_vector().as_slice(),
                &pinocchio.velocity(&pin_q, &pin_qd),
                1.0e-10,
                1.0e-10,
                &format!("tree velocity for {link_name}, sample {sample}"),
            );
            let acceleration = robot
                .forward_acceleration_kinematics(&dynibo::BaseState::fixed(), &q, &qd, &qdd, target)
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
fn branched_gravity_and_rnea_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "right_tool");

    for sample in 0..32 {
        let (q, qd, qdd) = deterministic_tree_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let mut gravity = [f64::NAN; 7];
        robot
            .gravity(&dynibo::BaseState::fixed(), &q, &[], &mut gravity)
            .unwrap();
        assert_close(
            &gravity,
            &pinocchio.gravity(&pin_q),
            1.0e-9,
            1.0e-10,
            &format!("tree gravity sample {sample}"),
        );
        let mut torque = [f64::NAN; 7];
        robot
            .inverse_dynamics(&dynibo::BaseState::fixed(), &q, &qd, &qdd, &[], &mut torque)
            .unwrap();
        assert_close(
            &torque,
            &pinocchio.rnea(&pin_q, &pin_qd, &pin_qdd),
            1.0e-9,
            1.0e-10,
            &format!("tree RNEA sample {sample}"),
        );
    }
}

#[test]
fn branched_moving_external_loads_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let load = Wrench::new(
        Vector3::new(-0.22, 0.35, 0.41),
        Vector3::new(0.74, -0.63, 0.28),
    );

    for link_name in ["trunk", "left_lower", "left_tool", "right_tool"] {
        let target = robot.link_id(link_name).unwrap();
        let indexed_load = IndexedLoad {
            link: target,
            wrench: load,
        };
        let mut pinocchio = PinocchioContext::new(&robot, &path, link_name);
        for sample in 0..16 {
            let (q, qd, qdd) = deterministic_tree_state(sample);
            let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
            let mut actual = [f64::NAN; 7];
            robot
                .inverse_dynamics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    &qdd,
                    &[indexed_load],
                    &mut actual,
                )
                .unwrap();
            let expected = pinocchio.rnea_with_link_load(&pin_q, &pin_qd, &pin_qdd, load);
            assert_close(
                &actual,
                &expected,
                1.0e-9,
                1.0e-10,
                &format!("tree external load on {link_name}, sample {sample}"),
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
            .inverse_kinematics(
                &dynibo::BaseState::fixed(),
                &initial,
                target,
                &desired,
                options,
                &mut solution,
            )
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

#[test]
fn floating_base_kinematics_and_dynamics_match_free_flyer_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf_with_base(&path, BaseMode::Floating).unwrap();
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
    let mut robot = Robot::from_urdf_with_base(&path, BaseMode::Floating).unwrap();
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
fn mixed_joint_moving_base_rnea_matches_free_flyer_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf_with_base(&path, BaseMode::Floating).unwrap();
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
