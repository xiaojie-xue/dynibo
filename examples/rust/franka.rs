use std::path::PathBuf;

use dynibo::{BaseState, IndexedLoad, InverseKinematicsOptions, Robot, Wrench};
use nalgebra::{DMatrixView, DVectorView, Isometry3, Vector3};

fn print_vector(label: &str, values: &[f64]) {
    println!(
        "{label}: {:.5}",
        DVectorView::from_slice(values, values.len()).transpose()
    );
}

fn main() -> dynibo::Result<()> {
    let urdf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("examples/data/franka_fer.urdf");
    let mut robot = Robot::from_urdf(urdf)?;
    let base = BaseState::fixed();
    let flange = robot.link_id("fer_link8")?;

    // Only non-fixed joints occupy state-vector entries; fer_joint8 is fixed.
    let q = [0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7];
    let qd = [0.10, -0.20, 0.15, 0.05, -0.10, 0.20, -0.05];
    let qdd = [0.20, 0.10, -0.10, 0.05, 0.10, -0.05, 0.15];
    let ik_initial_q = [0.05, -0.2, -0.05, -1.6, 0.05, 1.4, 0.6];
    let generalized_count = robot.generalized_count();

    let mut jacobian = vec![0.0; 6 * generalized_count];
    let mut jacobian_derivative = vec![0.0; 6 * generalized_count];
    let mut mass_matrix = vec![0.0; generalized_count * generalized_count];
    let mut velocity_forces = vec![0.0; generalized_count];
    let mut gravity = vec![0.0; generalized_count];
    let mut joint_forces = vec![0.0; generalized_count];

    // forward_kinematics -- target-link pose in the world frame.
    let pose = robot.forward_kinematics(&base, &q, flange)?;

    // jacobian and jacobian_derivative -- column-major, angular rows first.
    robot.jacobian(&base, &q, flange, &mut jacobian)?;
    robot.jacobian_derivative(&base, &q, &qd, flange, &mut jacobian_derivative)?;

    // forward_velocity_kinematics and forward_acceleration_kinematics.
    let velocity =
        robot.forward_velocity_kinematics(&base, &q, &qd, flange, &Isometry3::identity())?;
    let acceleration = robot.forward_acceleration_kinematics(&base, &q, &qd, &qdd, flange)?;

    // inverse_kinematics -- recover a known, reachable pose with damped least squares.
    let mut ik_solution = vec![0.0; robot.joint_count()];
    robot.inverse_kinematics(
        &base,
        &ik_initial_q,
        flange,
        &pose,
        InverseKinematicsOptions::default(),
        &mut ik_solution,
    )?;

    // mass_matrix and velocity_product_forces.
    robot.mass_matrix(&base, &q, &mut mass_matrix)?;
    robot.velocity_product_forces(&base, &q, &qd, &mut velocity_forces)?;

    // gravity and inverse_dynamics, both with a link-local external load.
    let loads = [IndexedLoad {
        link: flange,
        wrench: Wrench::new(Vector3::zeros(), Vector3::new(0.0, 0.0, -5.0)),
    }];
    robot.gravity(&base, &q, &loads, &mut gravity)?;
    robot.inverse_dynamics(&base, &q, &qd, &qdd, &loads, &mut joint_forces)?;

    println!(
        "loaded {}: {} links, {} non-fixed joints",
        robot.name(),
        robot.link_count(),
        robot.joint_count()
    );
    println!(
        "forward_kinematics translation [m]: {:.5}",
        pose.translation.vector.transpose()
    );
    println!(
        "forward_kinematics rotation:\n{:.5}",
        pose.rotation.to_rotation_matrix()
    );
    println!(
        "jacobian (6 x {generalized_count}):\n{:.5}",
        DMatrixView::from_slice(&jacobian, 6, generalized_count)
    );
    println!(
        "jacobian_derivative (6 x {generalized_count}):\n{:.5}",
        DMatrixView::from_slice(&jacobian_derivative, 6, generalized_count)
    );
    println!(
        "forward_velocity_kinematics: {:.5}",
        velocity.to_vector().transpose()
    );
    println!(
        "forward_acceleration_kinematics: {:.5}",
        acceleration.to_vector().transpose()
    );
    print_vector("inverse_kinematics", &ik_solution);
    println!(
        "mass_matrix ({generalized_count} x {generalized_count}):\n{:.5}",
        DMatrixView::from_slice(&mass_matrix, generalized_count, generalized_count)
    );
    print_vector("velocity_product_forces", &velocity_forces);
    print_vector("gravity (including external load)", &gravity);
    print_vector("inverse_dynamics (including external load)", &joint_forces);
    Ok(())
}
