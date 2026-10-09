use super::*;

#[test]
fn committed_binding_references_match_pinocchio() {
    let path = serial_fixture();
    let robot = Robot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new(&robot, &path, "test_link_4");
    let q = binding_reference("q");
    let qd = binding_reference("qd");
    let qdd = binding_reference("qdd");
    let (pin_q, pin_qd, pin_qdd) = pinocchio.state(&q, &qd, &qdd);
    let (_, translation) = pinocchio.frame(&pin_q);
    assert_close(
        translation.as_slice(),
        &binding_reference("fk_translation"),
        2.0e-12,
        1.0e-12,
        "committed binding FK translation",
    );
    assert_close(
        &pinocchio.gravity(&pin_q),
        &binding_reference("gravity"),
        2.0e-10,
        1.0e-10,
        "committed binding gravity",
    );
    assert_close(
        &pinocchio.rnea(&pin_q, &pin_qd, &pin_qdd),
        &binding_reference("rnea"),
        2.0e-10,
        1.0e-10,
        "committed binding RNEA",
    );

    let floating_robot = FloatingRobot::from_urdf(&path).unwrap();
    let mut floating = PinocchioContext::new_floating(&floating_robot, &path, "test_link_4");
    let translation = binding_reference("floating_base_translation");
    let rotation = binding_reference("floating_base_rotation_xyzw");
    let base = Frame::from_parts(
        Translation3::new(translation[0], translation[1], translation[2]),
        UnitQuaternion::from_quaternion(Quaternion::new(
            rotation[3],
            rotation[0],
            rotation[1],
            rotation[2],
        )),
    );
    let velocity = binding_reference("floating_base_velocity");
    let velocity = Twist::new(
        Vector3::from_column_slice(&velocity[..3]),
        Vector3::from_column_slice(&velocity[3..]),
    );
    let acceleration = binding_reference("floating_base_acceleration");
    let acceleration = Twist::new(
        Vector3::from_column_slice(&acceleration[..3]),
        Vector3::from_column_slice(&acceleration[3..]),
    );
    let (floating_q, floating_qd, floating_qdd) =
        floating.floating_state(&q, &qd, &qdd, &base, velocity, acceleration);
    let (_, floating_translation) = floating.frame(&floating_q);
    assert_close(
        floating_translation.as_slice(),
        &binding_reference("floating_fk_translation"),
        2.0e-12,
        1.0e-12,
        "committed floating binding FK translation",
    );
    assert_close(
        &floating.floating_gravity(&floating_q, &base),
        &binding_reference("floating_gravity"),
        2.0e-10,
        1.0e-10,
        "committed floating binding gravity",
    );
    assert_close(
        &floating.floating_rnea_with_loads(&floating_q, &floating_qd, &floating_qdd, &base, &[]),
        &binding_reference("floating_rnea"),
        2.0e-10,
        1.0e-10,
        "committed floating binding RNEA",
    );
    let load = binding_reference("floating_load");
    let load = floating.load(
        "test_link_4",
        Wrench::new(
            Vector3::from_column_slice(&load[..3]),
            Vector3::from_column_slice(&load[3..]),
        ),
    );
    assert_close(
        &floating.floating_rnea_with_loads(
            &floating_q,
            &floating_qd,
            &floating_qdd,
            &base,
            &[load],
        ),
        &binding_reference("floating_rnea_loaded"),
        2.0e-10,
        1.0e-10,
        "committed loaded floating binding RNEA",
    );
}

#[test]
fn committed_floating_motion_references_match_pinocchio() {
    let path = serial_fixture();
    let robot = FloatingRobot::from_urdf(&path).unwrap();
    let mut pinocchio = PinocchioContext::new_floating(&robot, &path, "test_link_4");
    let q = binding_reference("q");
    let translation = binding_reference("floating_base_translation");
    let rotation = binding_reference("floating_motion_rotation_xyzw");
    let base = Frame::from_parts(
        Translation3::from(Vector3::from_column_slice(&translation)),
        UnitQuaternion::from_quaternion(Quaternion::new(
            rotation[3],
            rotation[0],
            rotation[1],
            rotation[2],
        )),
    );
    let velocity = binding_reference("floating_base_velocity");
    let acceleration = binding_reference("floating_base_acceleration");
    let base_velocity = Twist::new(
        Vector3::from_column_slice(&velocity[..3]),
        Vector3::from_column_slice(&velocity[3..]),
    );
    let base_acceleration = Twist::new(
        Vector3::from_column_slice(&acceleration[..3]),
        Vector3::from_column_slice(&acceleration[3..]),
    );
    let tool = Vector3::from_column_slice(&binding_reference("floating_motion_tool_translation"));
    for case in ["base_only", "moving"] {
        let qd = if case == "moving" {
            binding_reference("qd")
        } else {
            vec![0.0; q.len()]
        };
        let qdd = if case == "moving" {
            binding_reference("qdd")
        } else {
            vec![0.0; q.len()]
        };
        let (pin_q, pin_v, pin_a) =
            pinocchio.floating_state(&q, &qd, &qdd, &base, base_velocity, base_acceleration);
        let derivative =
            pinocchio.floating_jacobian_derivative(&pin_q, &pin_v, &base, base_velocity.angular);
        let velocity = pinocchio.velocity(&pin_q, &pin_v);
        let acceleration = pinocchio.acceleration(&pin_q, &pin_v, &pin_a);
        let (rotation, _) = pinocchio.frame(&pin_q);
        let tool_linear = Vector3::from_column_slice(&velocity[3..])
            + Vector3::from_column_slice(&velocity[..3]).cross(&(rotation * tool));
        let tool_velocity: Vec<_> = velocity[..3]
            .iter()
            .chain(tool_linear.iter())
            .copied()
            .collect();
        let coriolis = pinocchio.floating_coriolis_from_rnea(&q, &qd, &base, base_velocity);
        let generalized_velocity: Vec<_> = base_velocity
            .to_vector()
            .iter()
            .chain(qd.iter())
            .copied()
            .collect();
        let n = generalized_velocity.len();
        let velocity_product: Vec<_> = (0..n)
            .map(|row| {
                (0..n)
                    .map(|column| coriolis[column * n + row] * generalized_velocity[column])
                    .sum()
            })
            .collect();
        for (name, actual) in [
            ("jacobian_derivative", derivative),
            ("velocity", velocity.to_vec()),
            ("acceleration", acceleration.to_vec()),
            ("tool_velocity", tool_velocity),
            ("velocity_product", velocity_product),
        ] {
            let key = format!("floating_motion_{case}_{name}");
            assert_close(&actual, &binding_reference(&key), 3.0e-9, 1.0e-9, &key);
        }
    }
}
