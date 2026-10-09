use super::*;

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
        robot.mass_matrix(&q, &mut mass).unwrap();
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
        robot.mass_matrix(&q, &mut mass).unwrap();
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
        robot.mass_matrix(&q, &mut mass).unwrap();
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
            .velocity_product_forces(&q, &qd, &mut velocity_product)
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
            .velocity_product_forces(&q, &qd, &mut velocity_product)
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
            .velocity_product_forces(&q, &qd, &mut velocity_product)
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
fn mixed_joint_gravity_and_rnea_match_pinocchio() {
    let path = fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "tool");

    for sample in 0..64 {
        let (q, qd, qdd) = deterministic_mixed_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let mut actual_gravity = [f64::NAN; 3];
        robot.gravity(&q, &[], &mut actual_gravity).unwrap();
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
            .inverse_dynamics(&q, &qd, &qdd, &[], &mut actual_torque)
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
            .forward_dynamics(&q, &qd, &torque, &[], &mut actual)
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
                .forward_dynamics(&q, &qd, &torque, &[indexed_load], &mut actual)
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
                .inverse_dynamics(&q, &qd, &qdd, &[indexed_load], &mut actual)
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
fn branched_gravity_and_rnea_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "right_tool");

    for sample in 0..32 {
        let (q, qd, qdd) = deterministic_tree_state(sample);
        let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
        let mut gravity = [f64::NAN; 7];
        robot.gravity(&q, &[], &mut gravity).unwrap();
        assert_close(
            &gravity,
            &pinocchio.gravity(&pin_q),
            1.0e-9,
            1.0e-10,
            &format!("tree gravity sample {sample}"),
        );
        let mut torque = [f64::NAN; 7];
        robot
            .inverse_dynamics(&q, &qd, &qdd, &[], &mut torque)
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
                .inverse_dynamics(&q, &qd, &qdd, &[indexed_load], &mut actual)
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
fn branched_aba_and_multi_link_external_loads_match_pinocchio() {
    let path = tree_fixture();
    let mut robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "right_tool");
    let load_a = Wrench::new(
        Vector3::new(-0.22, 0.35, 0.41),
        Vector3::new(0.74, -0.63, 0.28),
    );
    let load_b = Wrench::new(
        Vector3::new(0.17, -0.29, 0.13),
        Vector3::new(-0.51, 0.38, 0.62),
    );

    for sample in 0..32 {
        let (q, qd, _) = deterministic_tree_state(sample);
        let zero = [0.0; 7];
        let (pin_q, pin_qd, _) = pinocchio.state(&q, &qd, &zero);
        let torque: [f64; 7] = std::array::from_fn(|joint| {
            9.0 * ((sample + 1) as f64 * (joint + 2) as f64 * 0.379).sin()
        });

        let mut actual = [f64::NAN; 7];
        robot
            .forward_dynamics(&q, &qd, &torque, &[], &mut actual)
            .unwrap();
        assert_close(
            &actual,
            &pinocchio.aba(&pin_q, &pin_qd, &torque),
            3.0e-9,
            2.0e-10,
            &format!("tree ABA sample {sample}"),
        );

        let load_names: &[&str] = match sample % 4 {
            0 => &["trunk"],
            1 => &["left_lower"],
            2 => &["left_tool", "right_tool"],
            _ => &["trunk", "left_tool", "left_tool"],
        };
        let dynibo_loads: Vec<IndexedLoad> = load_names
            .iter()
            .enumerate()
            .map(|(index, name)| IndexedLoad {
                link: robot.link_id(name).unwrap(),
                wrench: if index.is_multiple_of(2) {
                    load_a
                } else {
                    load_b
                },
            })
            .collect();
        let pinocchio_loads: Vec<_> = load_names
            .iter()
            .enumerate()
            .map(|(index, name)| {
                pinocchio.load(
                    name,
                    if index.is_multiple_of(2) {
                        load_a
                    } else {
                        load_b
                    },
                )
            })
            .collect();
        robot
            .forward_dynamics(&q, &qd, &torque, &dynibo_loads, &mut actual)
            .unwrap();
        assert_close(
            &actual,
            &pinocchio.aba_with_loads(&pin_q, &pin_qd, &torque, &pinocchio_loads),
            4.0e-9,
            1.0e-9,
            &format!("tree ABA loads={load_names:?}, sample {sample}"),
        );
    }
}
