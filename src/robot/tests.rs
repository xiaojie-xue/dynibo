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
