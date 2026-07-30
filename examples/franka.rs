use std::path::PathBuf;

use dyno::{Frame, JointVector, Robot};

fn main() -> dyno::Result<()> {
    let urdf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("examples/data/franka_fer.urdf");
    let robot = Robot::from_urdf(urdf)?;
    let flange = robot.link("fer_link8")?;

    // The first seven entries are arm joints. Joint 8 is the fixed flange joint
    // and therefore remains zero (fixed joints currently occupy one entry).
    let q = JointVector::from([0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7, 0.0]);

    let flange_frame = robot.forward_kinematics(&q, flange)?;
    let jacobian = robot.jacobian(&q, flange)?;
    let gravity_torque = robot.gravity(&q, &Frame::identity(), &[])?;

    println!(
        "loaded {}: {} links, {} joints",
        robot.name(),
        robot.link_count(),
        robot.joint_count()
    );
    println!(
        "flange translation [m]: {:.4}",
        flange_frame.translation.vector.transpose()
    );
    println!(
        "flange rotation:\n{:.4}",
        flange_frame.rotation.to_rotation_matrix()
    );
    println!("flange Jacobian (angular rows first):\n{jacobian:.4}");
    println!(
        "gravity joint torque [N m]: {:.4}",
        gravity_torque.transpose()
    );
    Ok(())
}
