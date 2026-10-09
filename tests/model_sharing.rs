use std::{path::PathBuf, thread};

use dynibo::{BaseState, Frame, RobotModel, Wrench};

fn path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

#[test]
fn shared_models_preserve_ids_lifetimes_and_instance_local_base_frames() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<RobotModel>();
    let model = RobotModel::from_urdf(path("test_arm.urdf")).unwrap();
    let clone = model.clone();
    let target = model.link_id("test_link_4").unwrap();
    let mut fixed = model.robot();
    let mut floating = model.floating_robot().unwrap();
    assert_eq!(fixed.root_link_id(), floating.root_link_id());
    assert_eq!(model.link_id_at(4).unwrap(), target);
    assert_eq!(model.joint_name(0).unwrap(), fixed.joint_name(0).unwrap());
    assert_eq!(
        model.link_inertia(target).unwrap(),
        floating.link_inertia(target).unwrap()
    );
    let mut loads = model.load_buffer();
    loads.set(target, Wrench::zeros()).unwrap();
    let q = vec![0.0; model.joint_count()];
    let expected = fixed.forward_kinematics(&q, target).unwrap();
    fixed
        .set_base_frame(Frame::translation(1.0, 2.0, 3.0))
        .unwrap();
    let mut fork = fixed.fork();
    fixed.set_base_frame(Frame::identity()).unwrap();
    assert_eq!(fork.base_frame(), &Frame::translation(1.0, 2.0, 3.0));
    // The read-only model contains no base pose; new fixed instances start at identity.
    assert_eq!(fork.model().robot().base_frame(), &Frame::identity());
    drop(model);
    drop(fixed);
    assert_eq!(clone.link_id("test_link_4").unwrap(), target);
    let mut output = vec![0.0; floating.generalized_count()];
    floating
        .gravity(
            &BaseState::stationary(Frame::identity()).unwrap(),
            &q,
            loads.as_slice(),
            &mut output,
        )
        .unwrap();
    let poses = thread::scope(|scope| {
        let q = &q;
        let cloned = &clone;
        let a = scope.spawn(move || cloned.robot().forward_kinematics(q, target).unwrap());
        let b = scope.spawn(move || fork.forward_kinematics(q, target).unwrap());
        (a.join().unwrap(), b.join().unwrap())
    });
    assert_eq!(poses.0, expected);
    assert_eq!(
        poses.1.translation.vector,
        expected.translation.vector + nalgebra::Vector3::new(1.0, 2.0, 3.0)
    );
    assert_eq!(floating.model().root_link_id(), clone.root_link_id());
}

#[test]
fn model_queries_and_floating_validation_do_not_require_a_workspace() {
    let model = RobotModel::from_urdf(path("fixed_arm.urdf")).unwrap();
    assert!(model.validate_floating_base().is_err());
    assert!(model.floating_robot().is_err());
    assert_eq!(model.joint_count(), 0);
    assert!(model.link_id("missing").is_err());
    assert!(model.link_id_at(model.link_count()).is_err());
    assert!(model.joint_name(0).is_err());
    let mut robot = model.robot();
    assert!(
        robot
            .forward_kinematics(&[], model.link_id("tool").unwrap())
            .is_ok()
    );
    let unrelated = RobotModel::from_urdf(path("fixed_arm.urdf")).unwrap();
    assert!(model.link_name(unrelated.root_link_id()).is_err());
}
