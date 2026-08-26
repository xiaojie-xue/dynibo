use dynibo::{BaseState, FloatingRobot, IndexedLoad, Robot};

use super::{
    fixtures::LoadSpec,
    observation::{Observation, ObservedError, ObservedResult},
    states::JointState,
};

#[derive(Clone, Debug)]
pub struct MatrixCase {
    pub state: JointState,
    pub base: BaseState,
    pub loads: Vec<LoadSpec>,
    pub load_case: String,
}

impl MatrixCase {
    pub fn resolved_loads(&self, robot: &Robot) -> Vec<IndexedLoad> {
        self.loads.iter().map(|load| load.resolve(robot)).collect()
    }

    pub fn resolved_floating_loads(&self, robot: &FloatingRobot) -> Vec<IndexedLoad> {
        self.loads
            .iter()
            .map(|load| IndexedLoad {
                link: robot
                    .link_id(&load.link_name)
                    .expect("load link must resolve"),
                wrench: load.wrench,
            })
            .collect()
    }
}

#[derive(Clone, Debug)]
pub enum AlgorithmCase {
    ForwardKinematics { target: String },
    ForwardVelocity { target: String },
    ForwardAcceleration { target: String },
    Jacobian { target: String },
    JacobianDerivative { target: String },
    MassMatrix,
    Gravity,
    VelocityProduct,
    InverseDynamics,
    ForwardDynamics,
}

impl AlgorithmCase {
    pub const fn name(&self) -> &'static str {
        match self {
            Self::ForwardKinematics { .. } => "forward_kinematics",
            Self::ForwardVelocity { .. } => "forward_velocity",
            Self::ForwardAcceleration { .. } => "forward_acceleration",
            Self::Jacobian { .. } => "jacobian",
            Self::JacobianDerivative { .. } => "jacobian_derivative",
            Self::MassMatrix => "mass_matrix",
            Self::Gravity => "gravity",
            Self::VelocityProduct => "velocity_product",
            Self::InverseDynamics => "inverse_dynamics",
            Self::ForwardDynamics => "forward_dynamics",
        }
    }

    pub fn target(&self) -> Option<&str> {
        match self {
            Self::ForwardKinematics { target }
            | Self::ForwardVelocity { target }
            | Self::ForwardAcceleration { target }
            | Self::Jacobian { target }
            | Self::JacobianDerivative { target } => Some(target),
            _ => None,
        }
    }
}

pub fn execute_algorithm(
    robot: &mut Robot,
    case: &MatrixCase,
    algorithm: &AlgorithmCase,
) -> ObservedResult {
    let loads = case.resolved_loads(robot);
    execute_algorithm_with_loads(robot, case, algorithm, &loads)
}

fn observe<T>(result: dynibo::Result<T>, convert: impl FnOnce(T) -> Observation) -> ObservedResult {
    result.map(convert).map_err(|error| ObservedError {
        category: error.category(),
        message: error.to_string(),
    })
}

pub fn execute_algorithm_with_loads(
    robot: &mut Robot,
    case: &MatrixCase,
    algorithm: &AlgorithmCase,
    loads: &[IndexedLoad],
) -> ObservedResult {
    let state = &case.state;
    match algorithm {
        AlgorithmCase::ForwardKinematics { target } => {
            let target = robot.link_id(target).map_err(|error| ObservedError {
                category: error.category(),
                message: error.to_string(),
            })?;
            observe(
                robot.forward_kinematics(&state.q, target),
                Observation::Frame,
            )
        }
        AlgorithmCase::ForwardVelocity { target } => {
            let target = robot.link_id(target).map_err(|error| ObservedError {
                category: error.category(),
                message: error.to_string(),
            })?;
            observe(
                robot.forward_velocity_kinematics(
                    &state.q,
                    &state.qd,
                    target,
                    &dynibo::Frame::identity(),
                ),
                Observation::Twist,
            )
        }
        AlgorithmCase::ForwardAcceleration { target } => {
            let target = robot.link_id(target).map_err(|error| ObservedError {
                category: error.category(),
                message: error.to_string(),
            })?;
            observe(
                robot.forward_acceleration_kinematics(&state.q, &state.qd, &state.qdd, target),
                Observation::Twist,
            )
        }
        AlgorithmCase::Jacobian { target } | AlgorithmCase::JacobianDerivative { target } => {
            let target = robot.link_id(target).map_err(|error| ObservedError {
                category: error.category(),
                message: error.to_string(),
            })?;
            let columns = robot.generalized_count();
            let mut output = vec![f64::NAN; 6 * columns];
            let result = if matches!(algorithm, AlgorithmCase::Jacobian { .. }) {
                robot.jacobian(&state.q, target, &mut output)
            } else {
                robot.jacobian_derivative(&state.q, &state.qd, target, &mut output)
            };
            observe(result, |()| Observation::Matrix {
                rows: 6,
                columns,
                values: output,
            })
        }
        AlgorithmCase::MassMatrix => {
            let n = robot.generalized_count();
            let mut output = vec![f64::NAN; n * n];
            observe(robot.mass_matrix(&state.q, &mut output), |()| {
                Observation::Matrix {
                    rows: n,
                    columns: n,
                    values: output,
                }
            })
        }
        AlgorithmCase::Gravity
        | AlgorithmCase::VelocityProduct
        | AlgorithmCase::InverseDynamics
        | AlgorithmCase::ForwardDynamics => {
            let mut output = vec![f64::NAN; robot.generalized_count()];
            let result = match algorithm {
                AlgorithmCase::Gravity => robot.gravity(&state.q, loads, &mut output),
                AlgorithmCase::VelocityProduct => {
                    robot.velocity_product_forces(&state.q, &state.qd, &mut output)
                }
                AlgorithmCase::InverseDynamics => {
                    robot.inverse_dynamics(&state.q, &state.qd, &state.qdd, loads, &mut output)
                }
                AlgorithmCase::ForwardDynamics => {
                    robot.forward_dynamics(&state.q, &state.qd, &state.tau, loads, &mut output)
                }
                _ => unreachable!(),
            };
            observe(result, |()| Observation::Vector(output))
        }
    }
}

pub fn execute_algorithm_floating(
    robot: &mut FloatingRobot,
    case: &MatrixCase,
    algorithm: &AlgorithmCase,
) -> ObservedResult {
    let loads = case.resolved_floating_loads(robot);
    execute_algorithm_floating_with_loads(robot, case, algorithm, &loads)
}

pub fn execute_algorithm_floating_with_loads(
    robot: &mut FloatingRobot,
    case: &MatrixCase,
    algorithm: &AlgorithmCase,
    loads: &[IndexedLoad],
) -> ObservedResult {
    let state = &case.state;
    let base = &case.base;
    match algorithm {
        AlgorithmCase::ForwardKinematics { target } => {
            let target = robot.link_id(target).map_err(|error| ObservedError {
                category: error.category(),
                message: error.to_string(),
            })?;
            observe(
                robot.forward_kinematics(base, &state.q, target),
                Observation::Frame,
            )
        }
        AlgorithmCase::ForwardVelocity { target } => {
            let target = robot.link_id(target).map_err(|error| ObservedError {
                category: error.category(),
                message: error.to_string(),
            })?;
            observe(
                robot.forward_velocity_kinematics(
                    base,
                    &state.q,
                    &state.qd,
                    target,
                    &dynibo::Frame::identity(),
                ),
                Observation::Twist,
            )
        }
        AlgorithmCase::ForwardAcceleration { target } => {
            let target = robot.link_id(target).map_err(|error| ObservedError {
                category: error.category(),
                message: error.to_string(),
            })?;
            observe(
                robot
                    .forward_acceleration_kinematics(base, &state.q, &state.qd, &state.qdd, target),
                Observation::Twist,
            )
        }
        AlgorithmCase::Jacobian { target } | AlgorithmCase::JacobianDerivative { target } => {
            let target = robot.link_id(target).map_err(|error| ObservedError {
                category: error.category(),
                message: error.to_string(),
            })?;
            let columns = robot.generalized_count();
            let mut output = vec![f64::NAN; 6 * columns];
            let result = if matches!(algorithm, AlgorithmCase::Jacobian { .. }) {
                robot.jacobian(base, &state.q, target, &mut output)
            } else {
                robot.jacobian_derivative(base, &state.q, &state.qd, target, &mut output)
            };
            observe(result, |()| Observation::Matrix {
                rows: 6,
                columns,
                values: output,
            })
        }
        AlgorithmCase::MassMatrix => {
            let n = robot.generalized_count();
            let mut output = vec![f64::NAN; n * n];
            observe(robot.mass_matrix(base, &state.q, &mut output), |()| {
                Observation::Matrix {
                    rows: n,
                    columns: n,
                    values: output,
                }
            })
        }
        AlgorithmCase::Gravity
        | AlgorithmCase::VelocityProduct
        | AlgorithmCase::InverseDynamics
        | AlgorithmCase::ForwardDynamics => {
            let mut output = vec![f64::NAN; robot.generalized_count()];
            let result = match algorithm {
                AlgorithmCase::Gravity => robot.gravity(base, &state.q, loads, &mut output),
                AlgorithmCase::VelocityProduct => {
                    robot.velocity_product_forces(base, &state.q, &state.qd, &mut output)
                }
                AlgorithmCase::InverseDynamics => robot.inverse_dynamics(
                    base,
                    &state.q,
                    &state.qd,
                    &state.qdd,
                    loads,
                    &mut output,
                ),
                AlgorithmCase::ForwardDynamics => robot.forward_dynamics(
                    base,
                    &state.q,
                    &state.qd,
                    &state.tau,
                    loads,
                    &mut output,
                ),
                _ => unreachable!(),
            };
            observe(result, |()| Observation::Vector(output))
        }
    }
}
