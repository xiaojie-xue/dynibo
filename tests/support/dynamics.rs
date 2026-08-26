use dynibo::{BaseState, IndexedLoad, Robot};
use nalgebra::{DMatrix, DVector};

pub fn inverse_dynamics_bias(
    robot: &mut Robot,
    _base: &BaseState,
    q: &[f64],
    qd: &[f64],
    loads: &[IndexedLoad],
) -> Vec<f64> {
    let zero = vec![0.0; robot.joint_count()];
    let mut bias = vec![f64::NAN; robot.generalized_count()];
    robot
        .inverse_dynamics(q, qd, &zero, loads, &mut bias)
        .expect("bias inverse dynamics must succeed");
    bias
}

pub fn dense_forward_dynamics(
    robot: &mut Robot,
    base: &BaseState,
    q: &[f64],
    qd: &[f64],
    generalized_forces: &[f64],
    loads: &[IndexedLoad],
) -> Vec<f64> {
    let n = robot.generalized_count();
    assert_eq!(generalized_forces.len(), n);
    let bias = inverse_dynamics_bias(robot, base, q, qd, loads);
    let mut mass = vec![f64::NAN; n * n];
    robot
        .mass_matrix(q, &mut mass)
        .expect("mass matrix must succeed");
    let mass = DMatrix::from_column_slice(n, n, &mass);
    let rhs = DVector::from_iterator(
        n,
        generalized_forces
            .iter()
            .zip(&bias)
            .map(|(force, bias)| force - bias),
    );
    mass.cholesky()
        .expect("well-conditioned test mass matrix must be positive definite")
        .solve(&rhs)
        .as_slice()
        .to_vec()
}

pub fn generalized_force_for_acceleration(
    robot: &mut Robot,
    _base: &BaseState,
    q: &[f64],
    qd: &[f64],
    generalized_acceleration: &[f64],
    loads: &[IndexedLoad],
) -> Vec<f64> {
    assert_eq!(generalized_acceleration.len(), robot.generalized_count());
    let qdd = generalized_acceleration;
    let mut forces = vec![f64::NAN; robot.generalized_count()];
    robot
        .inverse_dynamics(q, qd, qdd, loads, &mut forces)
        .expect("inverse dynamics must produce generalized forces");
    forces
}
