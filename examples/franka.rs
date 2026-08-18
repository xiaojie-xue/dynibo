use std::path::PathBuf;

use dynibo::Robot;
use nalgebra::{DMatrixView, DVectorView};

fn main() -> dynibo::Result<()> {
    let urdf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("examples/data/franka_fer.urdf");
    let robot = Robot::from_urdf(urdf)?;
    let flange = robot.link_id("fer_link8")?;
    let mut workspace = robot.workspace();

    // The first seven entries are arm joints. Joint 8 is the fixed flange joint
    // and therefore remains zero (fixed joints currently occupy one entry).
    let q = [0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7, 0.0];
    let mut jacobian = vec![0.0; 6 * robot.joint_count()];
    let mut gravity_torque = vec![0.0; robot.joint_count()];

    let flange_frame = robot.forward_kinematics(&q, flange, &mut workspace)?;
    robot.jacobian(&q, flange, &mut workspace, &mut jacobian)?;
    robot.gravity(&q, &[], &mut workspace, &mut gravity_torque)?;

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
    println!(
        "flange Jacobian (angular rows first):\n{:.4}",
        DMatrixView::from_slice(&jacobian, 6, robot.joint_count())
    );
    println!(
        "gravity joint torque [N m]: {:.4}",
        DVectorView::from_slice(&gravity_torque, robot.joint_count()).transpose()
    );
    Ok(())
}
