use std::path::PathBuf;

use dynibo::{BaseState, FloatingRobot, Frame, Twist};
use nalgebra::Vector3;

fn main() -> dynibo::Result<()> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples/data/unitree-g1/g1_29dof_mode_11.urdf");
    let mut robot = FloatingRobot::from_urdf(path)?;
    let target = robot.link_id("left_rubber_hand")?;
    let (n, g) = (robot.joint_count(), robot.generalized_count());
    assert_eq!((n, g), (29, 35));

    // q, qd and qdd contain only actuated joints, in robot.joint_name(i) order.
    // Base twists are angular-first, expressed in world axes at the root origin.
    let base = BaseState::new(
        Frame::translation(0.0, 0.0, 0.8),
        Twist::new(Vector3::new(0.1, -0.05, 0.08), Vector3::new(0.2, 0.0, -0.1)),
        Twist::new(
            Vector3::new(0.02, 0.03, -0.01),
            Vector3::new(0.1, -0.2, 0.05),
        ),
    )?;
    let q = vec![0.0; n];
    let qd: Vec<_> = (0..n).map(|i| 0.1 * ((i + 1) as f64).cos()).collect();
    let qdd: Vec<_> = (0..n).map(|i| 0.2 * ((i + 1) as f64).sin()).collect();
    let pose = robot.forward_kinematics(&base, &q, target)?;
    let mut jacobian = vec![0.0; 6 * g];
    let mut forces = vec![0.0; g];
    let mut acceleration = vec![0.0; g];
    robot.jacobian(&base, &q, target, &mut jacobian)?;
    robot.inverse_dynamics(&base, &q, &qd, &qdd, &[], &mut forces)?;
    robot.forward_dynamics(&base, &q, &qd, &forces, &[], &mut acceleration)?;

    let expected: Vec<_> = base
        .acceleration()
        .to_vector()
        .iter()
        .copied()
        .chain(qdd.iter().copied())
        .collect();
    assert!(acceleration.iter().all(|value| value.is_finite()));
    let error = acceleration
        .iter()
        .zip(&expected)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f64, f64::max);
    assert!(error < 1e-8, "RNEA/ABA round-trip error: {error}");
    println!(
        "{}: {n} joints, {g} generalized velocities (floating base)",
        robot.name()
    );
    println!(
        "left hand position [m]: {}",
        pose.translation.vector.transpose()
    );
    println!("Jacobian: 6 x {g}, column-major; angular rows before linear rows");
    println!("RNEA base wrench [torque, force]: {:?}", &forces[..6]);
    println!("RNEA joint torques: {:?}", &forces[6..]);
    println!("RNEA -> ABA maximum acceleration error: {error:.3e}");

    // RNEA above returns the base wrench needed for prescribed acceleration.
    // A freely moving, unactuated base has ZERO applied generalized base wrench.
    // This is free-flight dynamics, not a ground-contact or balance simulation.
    forces[..6].fill(0.0);
    robot.forward_dynamics(&base, &q, &qd, &forces, &[], &mut acceleration)?;
    println!(
        "ABA unactuated-base acceleration [angular, linear]: {:?}",
        &acceleration[..6]
    );
    Ok(())
}
