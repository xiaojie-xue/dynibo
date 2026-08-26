use dynibo::{FloatingRobot, IndexedLoad, Robot, Wrench};

use super::{
    context::TestContext,
    matrix::{
        AlgorithmCase, MatrixCase, execute_algorithm, execute_algorithm_floating,
        execute_algorithm_floating_with_loads, execute_algorithm_with_loads,
    },
    model_gen::StableRng,
    numeric::DYNAMICS,
    observation::{ObservedError, ObservedResult, assert_observation_close},
};

#[derive(Clone, Debug)]
pub enum Operation {
    Valid {
        state: usize,
        algorithm: AlgorithmCase,
    },
    InvalidForwardDynamicsLength {
        state: usize,
    },
    InvalidForeignLoad {
        state: usize,
    },
}

pub fn deterministic_operation_sequence(
    state_count: usize,
    targets: &[String],
    seed: u64,
) -> Vec<Operation> {
    assert!(state_count > 0);
    assert!(!targets.is_empty());
    let mut rng = StableRng::new(seed);
    let algorithms = [
        AlgorithmCase::ForwardKinematics {
            target: targets[0].clone(),
        },
        AlgorithmCase::ForwardVelocity {
            target: targets[targets.len() - 1].clone(),
        },
        AlgorithmCase::ForwardAcceleration {
            target: targets[0].clone(),
        },
        AlgorithmCase::MassMatrix,
        AlgorithmCase::Gravity,
        AlgorithmCase::InverseDynamics,
        AlgorithmCase::ForwardDynamics,
        AlgorithmCase::Jacobian {
            target: targets[targets.len() - 1].clone(),
        },
        AlgorithmCase::JacobianDerivative {
            target: targets[0].clone(),
        },
        AlgorithmCase::VelocityProduct,
    ];
    let mut operations: Vec<_> = (0..24)
        .map(|step| Operation::Valid {
            state: rng.next_u64() as usize % state_count,
            algorithm: algorithms[step % algorithms.len()].clone(),
        })
        .collect();
    operations.insert(7, Operation::InvalidForwardDynamicsLength { state: 0 });
    operations.insert(
        16,
        Operation::InvalidForeignLoad {
            state: 1 % state_count,
        },
    );
    operations
}

pub fn run_workspace_sequence(
    prototype: &Robot,
    cases: &[MatrixCase],
    operations: &[Operation],
    foreign_link: dynibo::LinkId,
    fixture: &str,
    seed: u64,
) {
    let mut reused = prototype.fork();
    for (step, operation) in operations.iter().enumerate() {
        let mut clean = prototype.fork();
        let actual = execute_operation(&mut reused, cases, operation, foreign_link);
        let expected = execute_operation(&mut clean, cases, operation, foreign_link);
        let (state, name) = match operation {
            Operation::Valid { state, algorithm } => (*state, algorithm.name()),
            Operation::InvalidForwardDynamicsLength { state } => {
                (*state, "invalid_forward_dynamics_length")
            }
            Operation::InvalidForeignLoad { state } => (*state, "invalid_foreign_load"),
        };
        let mut context = TestContext::new(name, fixture)
            .seed(seed)
            .sample(state)
            .base_mode(super::context::TestBaseMode::Fixed)
            .step(step)
            .load_case(&cases[state].load_case);
        if let Operation::Valid { algorithm, .. } = operation
            && let Some(target) = algorithm.target()
        {
            context = context.target(target);
        }
        assert_observation_close(&actual, &expected, DYNAMICS, &context);
    }
}

pub fn run_floating_workspace_sequence(
    prototype: &FloatingRobot,
    cases: &[MatrixCase],
    operations: &[Operation],
    foreign_link: dynibo::LinkId,
    fixture: &str,
    seed: u64,
) {
    let mut reused = prototype.fork();
    for (step, operation) in operations.iter().enumerate() {
        let mut clean = prototype.fork();
        let actual = execute_floating_operation(&mut reused, cases, operation, foreign_link);
        let expected = execute_floating_operation(&mut clean, cases, operation, foreign_link);
        let (state, name) = match operation {
            Operation::Valid { state, algorithm } => (*state, algorithm.name()),
            Operation::InvalidForwardDynamicsLength { state } => {
                (*state, "invalid_forward_dynamics_length")
            }
            Operation::InvalidForeignLoad { state } => (*state, "invalid_foreign_load"),
        };
        let mut context = TestContext::new(name, fixture)
            .seed(seed)
            .sample(state)
            .base_mode(super::context::TestBaseMode::Floating)
            .step(step)
            .load_case(&cases[state].load_case);
        if let Operation::Valid { algorithm, .. } = operation
            && let Some(target) = algorithm.target()
        {
            context = context.target(target);
        }
        assert_observation_close(&actual, &expected, DYNAMICS, &context);
    }
}

fn execute_operation(
    robot: &mut Robot,
    cases: &[MatrixCase],
    operation: &Operation,
    foreign_link: dynibo::LinkId,
) -> ObservedResult {
    match operation {
        Operation::Valid { state, algorithm } => {
            execute_algorithm(robot, &cases[*state], algorithm)
        }
        Operation::InvalidForwardDynamicsLength { state } => {
            let case = &cases[*state];
            let mut output = vec![0.0; robot.generalized_count()];
            let short_q = &case.state.q[..case.state.q.len().saturating_sub(1)];
            robot
                .forward_dynamics(short_q, &case.state.qd, &case.state.tau, &[], &mut output)
                .map(|()| super::observation::Observation::Vector(output))
                .map_err(|error| ObservedError {
                    category: error.category(),
                    message: error.to_string(),
                })
        }
        Operation::InvalidForeignLoad { state } => {
            let case = &cases[*state];
            let load = IndexedLoad {
                link: foreign_link,
                wrench: Wrench::zeros(),
            };
            execute_algorithm_with_loads(robot, case, &AlgorithmCase::ForwardDynamics, &[load])
        }
    }
}

fn execute_floating_operation(
    robot: &mut FloatingRobot,
    cases: &[MatrixCase],
    operation: &Operation,
    foreign_link: dynibo::LinkId,
) -> ObservedResult {
    match operation {
        Operation::Valid { state, algorithm } => {
            execute_algorithm_floating(robot, &cases[*state], algorithm)
        }
        Operation::InvalidForwardDynamicsLength { state } => {
            let case = &cases[*state];
            let mut output = vec![0.0; robot.generalized_count()];
            let short_q = &case.state.q[..case.state.q.len().saturating_sub(1)];
            robot
                .forward_dynamics(
                    &case.base,
                    short_q,
                    &case.state.qd,
                    &case.state.tau,
                    &[],
                    &mut output,
                )
                .map(|()| super::observation::Observation::Vector(output))
                .map_err(|error| ObservedError {
                    category: error.category(),
                    message: error.to_string(),
                })
        }
        Operation::InvalidForeignLoad { state } => {
            let case = &cases[*state];
            let load = IndexedLoad {
                link: foreign_link,
                wrench: Wrench::zeros(),
            };
            execute_algorithm_floating_with_loads(
                robot,
                case,
                &AlgorithmCase::ForwardDynamics,
                &[load],
            )
        }
    }
}
