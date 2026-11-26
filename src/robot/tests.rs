use crate::{Error, JointType};

use std::path::PathBuf;

use super::*;

fn robot() -> Robot {
    Robot::from_urdf(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf"))
        .unwrap()
}

#[test]
fn identifiers_queries_and_forks_preserve_model_scope() {
    let robot = robot();
    let fork = robot.fork();
    assert_eq!(
        robot
            .model
            .validate_link_id(LinkId::new(robot.model.model_id, 0))
            .unwrap(),
        0
    );
    assert!(matches!(
        robot
            .model
            .validate_link_id(LinkId::new(robot.model.model_id.wrapping_add(1), 0)),
        Err(Error::InvalidLinkId)
    ));
    assert!(matches!(
        robot
            .model
            .validate_link_id(LinkId::new(robot.model.model_id, robot.link_count())),
        Err(Error::InvalidLinkId)
    ));
    assert_eq!(robot.root_link_id(), fork.root_link_id());
    assert_eq!(robot.link_id_at(0).unwrap(), robot.root_link_id());
    assert!(matches!(
        robot.link_id_at(robot.link_count()),
        Err(Error::InvalidLinkId)
    ));
    assert_eq!(robot.joint_name(0).unwrap(), "test_joint_1");
    assert_eq!(robot.joint_type(0).unwrap(), JointType::Revolute);
    assert_eq!(robot.joint_lower_limit(0).unwrap(), -0.610865238198015);
    assert_eq!(robot.joint_upper_limit(0).unwrap(), 0.610865238198015);
    assert_eq!(robot.joint_velocity_limit(0).unwrap(), 180.0);
    assert!(matches!(
        robot.joint_name(robot.joint_count()),
        Err(Error::InvalidJointIndex { .. })
    ));
}

#[test]
fn floating_identifiers_and_metadata_match_the_shared_model() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf");
    let robot = FloatingRobot::from_urdf(path).unwrap();
    let fork = robot.fork();

    assert_eq!(robot.name(), "test_arm");
    assert_eq!(robot.link_count(), 5);
    assert_eq!(robot.joint_count(), 4);
    assert_eq!(robot.generalized_count(), 10);
    assert_eq!(robot.root_link_id(), fork.root_link_id());

    let link = robot.link_id("test_link_4").unwrap();
    assert_eq!(robot.link_id_at(4).unwrap(), link);
    assert_eq!(robot.link_name(link).unwrap(), "test_link_4");
    assert!(robot.link_mass(link).unwrap().is_finite());
    assert!(
        robot
            .link_center_of_mass(link)
            .unwrap()
            .iter()
            .all(|x| x.is_finite())
    );
    assert!(
        robot
            .link_inertia(link)
            .unwrap()
            .iter()
            .all(|x| x.is_finite())
    );

    assert_eq!(robot.joint_name(0).unwrap(), "test_joint_1");
    assert_eq!(robot.joint_type(0).unwrap(), JointType::Revolute);
    assert_eq!(robot.joint_lower_limit(0).unwrap(), -0.610865238198015);
    assert_eq!(robot.joint_upper_limit(0).unwrap(), 0.610865238198015);
    assert_eq!(robot.joint_velocity_limit(0).unwrap(), 180.0);

    assert!(matches!(
        robot.link_id("missing"),
        Err(Error::UnknownLink { .. })
    ));
    assert!(matches!(
        robot.link_id_at(robot.link_count()),
        Err(Error::InvalidLinkId)
    ));
    assert!(matches!(
        robot.joint_name(robot.joint_count()),
        Err(Error::InvalidJointIndex { .. })
    ));
}

#[test]
fn fixed_base_frame_is_instance_local_and_validated() {
    let mut robot = robot();
    let original = *robot.base_frame();
    let frame = Frame::translation(0.4, -0.2, 0.8);
    robot.set_base_frame(frame).unwrap();
    assert_eq!(*robot.base_frame(), frame);

    let fork = robot.fork();
    assert_eq!(*fork.base_frame(), frame);

    assert!(matches!(
        robot.set_base_frame(Frame::translation(f64::NAN, 0.0, 0.0)),
        Err(Error::InvalidBaseState { field: "frame", .. })
    ));
    assert_eq!(*robot.base_frame(), frame);
    assert_ne!(original, frame);
}

#[test]
fn joint_inputs_must_be_finite() {
    let robot = robot();
    let mut values = vec![0.0; robot.joint_count()];

    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        values[0] = invalid;
        assert!(matches!(
            robot.model.validate_slice("q", &values),
            Err(Error::NonFiniteInput { input: "q" })
        ));
    }
}

#[test]
fn configurable_gravity_is_validated_inherited_and_consistent_across_dynamics() {
    use approx::assert_relative_eq;
    use nalgebra::{Translation3, UnitQuaternion};

    let mut fixed = robot();
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf");
    let mut floating = FloatingRobot::from_urdf(path).unwrap();

    assert_eq!(*fixed.gravity_vector(), Vector3::new(0.0, 0.0, -9.80665));
    assert_eq!(*floating.gravity_vector(), Vector3::new(0.0, 0.0, -9.80665));

    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            fixed.set_gravity(Vector3::new(invalid, 0.0, 0.0)),
            Err(Error::NonFiniteInput { input: "gravity" })
        ));
        assert!(matches!(
            floating.set_gravity(Vector3::new(0.0, invalid, 0.0)),
            Err(Error::NonFiniteInput { input: "gravity" })
        ));
    }

    // Zero gravity yields zero static torques and pure free-floating zero acceleration.
    fixed.set_gravity(Vector3::zeros()).unwrap();
    floating.set_gravity(Vector3::zeros()).unwrap();
    assert_eq!(*fixed.fork().gravity_vector(), Vector3::zeros());
    assert_eq!(*floating.fork().gravity_vector(), Vector3::zeros());

    let q = [0.2, 0.5, -0.4, 0.3];
    let qd = [0.1, -0.2, 0.3, -0.1];
    let qdd = [0.4, -0.3, 0.2, 0.1];
    let mut g_fixed = vec![1.0; fixed.generalized_count()];
    fixed.gravity(&q, &[], &mut g_fixed).unwrap();
    assert!(g_fixed.iter().all(|v| *v == 0.0));

    let base = crate::BaseState::stationary(Frame::identity()).unwrap();
    let mut g_float = vec![1.0; floating.generalized_count()];
    floating.gravity(&base, &q, &[], &mut g_float).unwrap();
    assert!(g_float.iter().all(|v| *v == 0.0));

    // Custom tilted gravity matches RNEA/ABA round-trip and equivalent base rotation.
    let custom_g = Vector3::new(1.5, -3.7, -8.2);
    fixed.set_gravity(custom_g).unwrap();
    floating.set_gravity(custom_g).unwrap();
    let mut tau = vec![0.0; fixed.generalized_count()];
    let mut recovered_qdd = vec![0.0; fixed.generalized_count()];
    fixed
        .inverse_dynamics(&q, &qd, &qdd, &[], &mut tau)
        .unwrap();
    fixed
        .forward_dynamics(&q, &qd, &tau, &[], &mut recovered_qdd)
        .unwrap();
    for (&actual, &expected) in recovered_qdd.iter().zip(&qdd) {
        assert_relative_eq!(actual, expected, epsilon = 1.0e-10);
    }

    let mut float_tau = vec![0.0; floating.generalized_count()];
    let mut float_acc = vec![0.0; floating.generalized_count()];
    floating
        .inverse_dynamics(&base, &q, &qd, &qdd, &[], &mut float_tau)
        .unwrap();
    floating
        .forward_dynamics(&base, &q, &qd, &float_tau, &[], &mut float_acc)
        .unwrap();
    for &value in float_acc.iter().take(6) {
        assert_relative_eq!(value, 0.0, epsilon = 1.0e-10);
    }
    for (i, &expected) in qdd.iter().enumerate() {
        assert_relative_eq!(float_acc[6 + i], expected, epsilon = 1.0e-10);
    }

    // Y-down gravity on an identity base matches Z-down gravity on a +90 deg X-rotated base
    // (where local +y aligns with world +z, so world -z gravity acts along local -y).
    let mut y_down = robot();
    y_down
        .set_gravity(Vector3::new(0.0, -9.80665, 0.0))
        .unwrap();
    let mut rotated = robot();
    rotated
        .set_base_frame(Frame::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2),
        ))
        .unwrap();
    let mut g_y = vec![0.0; 4];
    let mut g_rot = vec![0.0; 4];
    y_down.gravity(&q, &[], &mut g_y).unwrap();
    rotated.gravity(&q, &[], &mut g_rot).unwrap();
    for (&a, &b) in g_y.iter().zip(&g_rot) {
        assert_relative_eq!(a, b, epsilon = 1.0e-12);
    }
}
