//! Six verified tasks using the bundled one-joint URDF, without ROS or meshes.

use dynibo::{BaseState, FloatingRobot, Frame, Robot};

fn main() -> dynibo::Result<()> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/data/pendulum.urdf");
    let mut robot = Robot::from_urdf(path)?;
    let target = robot.link_id("tool")?;
    let q = [0.3_f64];
    let qd = [0.2];
    let qdd = [-0.4];

    // 1. URDF forward kinematics: a one-metre arm rotating about y.
    let pose = robot.forward_kinematics(&q, target)?;
    assert!((pose.translation.x - q[0].cos()).abs() < 1.0e-12);
    assert!((pose.translation.z + q[0].sin()).abs() < 1.0e-12);

    // 2. Column-major Jacobian: angular rows, then target-origin linear rows.
    let mut jacobian = vec![0.0; 6 * robot.generalized_count()];
    robot.jacobian(&q, target, &mut jacobian)?;
    assert!((jacobian[1] - 1.0).abs() < 1.0e-12);
    assert!((jacobian[3] + q[0].sin()).abs() < 1.0e-12);
    assert!((jacobian[5] + q[0].cos()).abs() < 1.0e-12);

    // 3. Gravity compensation: a 1 kg arm with its centre of mass at x = 0.5 m.
    let mut gravity = vec![0.0; robot.generalized_count()];
    robot.gravity(&q, &[], &mut gravity)?;
    assert!((gravity[0] + 0.5 * 9.80665 * q[0].cos()).abs() < 1.0e-12);

    // 4. RNEA / ABA round trip and the one-joint inertia M = 0.35 kg m².
    let mut effort = vec![0.0; robot.generalized_count()];
    let mut acceleration = vec![0.0; robot.generalized_count()];
    robot.inverse_dynamics(&q, &qd, &qdd, &[], &mut effort)?;
    robot.forward_dynamics(&q, &qd, &effort, &[], &mut acceleration)?;
    assert!((acceleration[0] - qdd[0]).abs() < 1.0e-12);
    let mut mass = vec![0.0; robot.generalized_count().pow(2)];
    robot.mass_matrix(&q, &mut mass)?;
    assert!((mass[0] - 0.35).abs() < 1.0e-12);

    // 5. Floating base: six world angular / linear entries precede the joints.
    let mut floating = FloatingRobot::from_urdf(path)?;
    let base = BaseState::stationary(Frame::identity())?;
    let mut floating_effort = vec![0.0; floating.generalized_count()];
    let mut floating_acceleration = vec![0.0; floating.generalized_count()];
    floating.inverse_dynamics(&base, &q, &qd, &qdd, &[], &mut floating_effort)?;
    floating.forward_dynamics(
        &base,
        &q,
        &qd,
        &floating_effort,
        &[],
        &mut floating_acceleration,
    )?;
    assert!(floating_acceleration[..6].iter().all(|a| a.abs() < 1.0e-11));
    assert!((floating_acceleration[6] - qdd[0]).abs() < 1.0e-11);

    // 6. Reuse the same input and output storage across repeated calculations.
    let mut positions = q;
    let loads = robot.load_buffer();
    for angle in [0.0_f64, 0.1, 0.2] {
        positions[0] = angle;
        robot.jacobian(&positions, target, &mut jacobian)?;
        robot.gravity(&positions, loads.as_slice(), &mut gravity)?;
        assert!((gravity[0] + 0.5 * 9.80665 * angle.cos()).abs() < 1.0e-12);
    }
    println!("Verified FK, Jacobian, gravity, RNEA/ABA, floating base, and buffer reuse.");
    Ok(())
}
