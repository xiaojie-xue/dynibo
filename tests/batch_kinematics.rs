use std::path::PathBuf;

use approx::assert_relative_eq;
use dynibo::{BaseState, FloatingRobot, Frame, Robot, Wrench};
use nalgebra::{Translation3, UnitQuaternion, Vector3};

#[test]
fn all_link_fk_matches_single_targets_and_survives_workspace_reuse() {
    for fixture in [
        "test_tree_7.urdf",
        "test_arm_40.urdf",
        "small_floating_base.urdf",
    ] {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/data")
            .join(fixture);
        let mut fixed = Robot::from_urdf(&path).unwrap();
        let mut floating = FloatingRobot::from_urdf(&path).unwrap();
        let q = vec![0.13; fixed.joint_count()];
        let qd = vec![0.21; fixed.joint_count()];
        let mut poses = vec![Frame::identity(); fixed.link_count()];
        for offset in [0.2, 1e12] {
            let frame = Frame::from_parts(
                Translation3::new(offset, -offset, 0.3),
                UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4),
            );
            fixed.set_base_frame(frame).unwrap();
            let base = BaseState::stationary(frame).unwrap();
            fixed.forward_kinematics_all(&q, &mut poses).unwrap();
            for (i, pose) in poses.iter().enumerate() {
                let id = fixed.link_id_at(i).unwrap();
                assert_relative_eq!(
                    *pose,
                    fixed.forward_kinematics(&q, id).unwrap(),
                    epsilon = 1e-12
                );
                let mut reused = vec![0.0; 6 * fixed.generalized_count()];
                let mut clean = reused.clone();
                fixed.jacobian_derivative(&q, &qd, id, &mut reused).unwrap();
                fixed
                    .fork()
                    .jacobian_derivative(&q, &qd, id, &mut clean)
                    .unwrap();
                assert_eq!(reused, clean);
            }
            floating
                .forward_kinematics_all(&base, &q, &mut poses)
                .unwrap();
            for (i, pose) in poses.iter().enumerate() {
                assert_relative_eq!(
                    *pose,
                    floating
                        .forward_kinematics(&base, &q, floating.link_id_at(i).unwrap())
                        .unwrap(),
                    epsilon = 1e-12
                );
            }
            let old = poses.clone();
            assert!(fixed.forward_kinematics_all(&q, &mut poses[1..]).is_err());
            assert_eq!(poses, old);
            let bad = vec![f64::NAN; q.len()];
            if !bad.is_empty() {
                assert!(
                    floating
                        .forward_kinematics_all(&base, &bad, &mut poses)
                        .is_err()
                );
                assert_eq!(poses, old);
            }
        }
    }
}

#[test]
fn reusable_load_buffer_preserves_scope_and_recovers_from_failed_updates() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf");
    let mut robot = Robot::from_urdf(&path).unwrap();
    let other = Robot::from_urdf(&path).unwrap();
    let mut loads = robot.load_buffer();
    let left = robot.link_id("left_tool").unwrap();
    let right = robot.link_id("right_tool").unwrap();
    let wrench = Wrench::new(Vector3::repeat(0.3), Vector3::repeat(0.2));
    loads.set(left, wrench).unwrap();
    loads.add(left, wrench).unwrap();
    assert_eq!(loads.len(), 1);
    assert_eq!(loads.as_slice()[0].wrench.torque, Vector3::repeat(0.6));
    let saved = loads.as_slice().to_vec();
    assert!(loads.set(other.link_id_at(0).unwrap(), wrench).is_err());
    assert!(
        loads
            .set(
                left,
                Wrench::new(Vector3::repeat(f64::NAN), Vector3::zeros())
            )
            .is_err()
    );
    assert_eq!(loads.as_slice(), saved);
    loads.set(right, wrench).unwrap();
    loads.remove(left).unwrap();
    loads.add(right, wrench).unwrap();
    assert_eq!(loads.len(), 1);
    assert_eq!(loads.as_slice()[0].link, right);
    loads.clear();
    assert!(loads.is_empty());
    let huge = Wrench::new(Vector3::repeat(f64::MAX), Vector3::zeros());
    loads.set(left, huge).unwrap();
    assert!(loads.add(left, huge).is_err());
    assert_eq!(loads.as_slice()[0].wrench, huge);
    loads.clear();
    loads.set(right, wrench).unwrap();
    let q = vec![0.0; robot.joint_count()];
    let mut actual = vec![0.0; q.len()];
    let mut expected = actual.clone();
    robot.gravity(&q, loads.as_slice(), &mut actual).unwrap();
    robot
        .fork()
        .gravity(&q, loads.as_slice(), &mut expected)
        .unwrap();
    assert_eq!(actual, expected);
}

#[test]
fn load_buffer_add_and_remove_handle_absent_entries_and_non_finite_inputs() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf");
    let robot = Robot::from_urdf(path).unwrap();
    let mut loads = robot.load_buffer();
    let left = robot.link_id("left_tool").unwrap();
    let right = robot.link_id("right_tool").unwrap();
    let wrench = Wrench::new(Vector3::repeat(0.3), Vector3::repeat(0.2));
    loads.remove(left).unwrap();
    assert!(loads.is_empty());
    loads.add(left, wrench).unwrap();
    assert_eq!(loads.len(), 1);
    assert_eq!(loads.as_slice()[0].link, left);
    assert_eq!(loads.as_slice()[0].wrench, wrench);
    let saved = loads.as_slice().to_vec();
    loads.remove(right).unwrap();
    assert_eq!(loads.as_slice(), saved);
    for link in [left, right] {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for invalid in [
                Wrench::new(Vector3::repeat(value), Vector3::zeros()),
                Wrench::new(Vector3::zeros(), Vector3::repeat(value)),
            ] {
                assert!(matches!(
                    loads.add(link, invalid),
                    Err(dynibo::Error::NonFiniteInput { input: "load" })
                ));
                assert_eq!(loads.as_slice(), saved);
            }
        }
    }
    loads.add(right, wrench).unwrap();
    loads.remove(left).unwrap();
    loads.add(right, wrench).unwrap();
    assert_eq!(loads.len(), 1);
    assert_eq!(loads.as_slice()[0].link, right);
    assert_eq!(loads.as_slice()[0].wrench.torque, wrench.torque * 2.0);
    assert_eq!(loads.as_slice()[0].wrench.force, wrench.force * 2.0);
}
