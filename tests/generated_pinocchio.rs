#![cfg(feature = "pinocchio-tests")]

mod support;

use support::context::TestRootType as RootType;

use std::collections::HashMap;

use dynibo::{BaseState, FloatingRobot, Frame, Twist, Wrench};
use nalgebra::{Rotation3, Translation3, UnitQuaternion, Vector3};
use support::{
    context::TestContext,
    fixtures::LoadSpec,
    matrix::{AlgorithmCase, MatrixCase, execute_algorithm, execute_algorithm_floating},
    model_gen::{generate_case, selected_model_cases},
    numeric::DYNAMICS,
    observation::{Observation, ObservedResult, assert_observation_close},
    pinocchio::PinocchioContext,
    states::deterministic_joint_state,
};

fn twist(values: &[f64]) -> Twist {
    Twist::new(
        Vector3::from_column_slice(&values[..3]),
        Vector3::from_column_slice(&values[3..]),
    )
}

#[allow(clippy::too_many_arguments)]
fn floating_expected(
    pinocchio: &mut PinocchioContext,
    algorithm: &AlgorithmCase,
    state: &support::states::JointState,
    base: &BaseState,
    pin_q: &[f64],
    pin_qd: &[f64],
    pin_qdd: &[f64],
    loads: &[support::pinocchio::PinocchioLoad],
    generalized_count: usize,
) -> ObservedResult {
    match algorithm {
        AlgorithmCase::ForwardKinematics { .. } => {
            let (rotation, translation) = pinocchio.frame(pin_q);
            Ok(Observation::Frame(Frame::from_parts(
                Translation3::from(translation),
                UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation)),
            )))
        }
        AlgorithmCase::ForwardVelocity { .. } => Ok(Observation::Twist(twist(
            &pinocchio.velocity(pin_q, pin_qd),
        ))),
        AlgorithmCase::ForwardAcceleration { .. } => Ok(Observation::Twist(twist(
            &pinocchio.acceleration(pin_q, pin_qd, pin_qdd),
        ))),
        AlgorithmCase::Jacobian { .. } => Ok(Observation::Matrix {
            rows: 6,
            columns: generalized_count,
            values: pinocchio.floating_jacobian(pin_q, base.frame()),
        }),
        AlgorithmCase::JacobianDerivative { .. } => Ok(Observation::Matrix {
            rows: 6,
            columns: generalized_count,
            values: pinocchio.floating_jacobian_derivative(
                pin_q,
                pin_qd,
                base.frame(),
                base.velocity().angular,
            ),
        }),
        AlgorithmCase::MassMatrix => Ok(Observation::Matrix {
            rows: generalized_count,
            columns: generalized_count,
            values: pinocchio.floating_mass_matrix(pin_q, base.frame()),
        }),
        AlgorithmCase::Gravity => Ok(Observation::Vector(pinocchio.floating_gravity_with_loads(
            pin_q,
            base.frame(),
            loads,
        ))),
        AlgorithmCase::VelocityProduct => {
            let zero = vec![0.0; state.q.len()];
            let (q, qd, qdd) = pinocchio.floating_state(
                &state.q,
                &state.qd,
                &zero,
                base.frame(),
                base.velocity(),
                Twist::zeros(),
            );
            let bias = pinocchio.floating_rnea_with_loads(&q, &qd, &qdd, base.frame(), &[]);
            let gravity = pinocchio.floating_gravity(&q, base.frame());
            Ok(Observation::Vector(
                bias.iter()
                    .zip(gravity)
                    .map(|(bias, gravity)| bias - gravity)
                    .collect(),
            ))
        }
        AlgorithmCase::InverseDynamics => Ok(Observation::Vector(
            pinocchio.floating_rnea_with_loads(pin_q, pin_qd, pin_qdd, base.frame(), loads),
        )),
        AlgorithmCase::ForwardDynamics => {
            Ok(Observation::Vector(pinocchio.floating_aba_with_loads(
                pin_q,
                pin_qd,
                &state.tau,
                base.frame(),
                base.velocity(),
                loads,
            )))
        }
    }
}

#[test]
fn generated_models_match_pinocchio() {
    for case in selected_model_cases(24) {
        let seed = case.seed;
        let base_mode = case.options.base_mode;
        if base_mode != RootType::Fixed {
            continue;
        }
        let generated = generate_case(&case);
        let mut robot = generated.robot();
        let mut contexts = HashMap::<String, PinocchioContext>::new();

        for sample in 0..8 {
            let target_name = generated.metadata.branch_targets
                [sample % generated.metadata.branch_targets.len()]
            .clone();
            let load_spec = LoadSpec::new(
                &target_name,
                Wrench::new(
                    Vector3::new(0.23, -0.17, 0.11),
                    Vector3::new(-0.7, 0.4, -0.2),
                ),
            );
            let state = deterministic_joint_state(robot.joint_count(), sample);
            let base = BaseState::stationary(Frame::identity()).unwrap();
            let load_specs = if sample.is_multiple_of(2) {
                vec![]
            } else {
                vec![load_spec]
            };
            let matrix_case = MatrixCase {
                state: state.clone(),
                base,
                loads: load_specs.clone(),
                load_case: if load_specs.is_empty() {
                    "none".to_owned()
                } else {
                    "target".to_owned()
                },
            };
            let pinocchio = contexts.entry(target_name.clone()).or_insert_with(|| {
                if base_mode == RootType::Fixed {
                    PinocchioContext::new(&robot, generated.path(), &target_name)
                } else {
                    unreachable!("floating cases are handled by the typed floating oracle suite")
                }
            });
            let pinocchio_loads: Vec<_> = load_specs
                .iter()
                .map(|load| pinocchio.load(&load.link_name, load.wrench))
                .collect();
            let (pin_q, pin_qd, pin_qdd) = match base_mode {
                RootType::Fixed => pinocchio.state(&state.q, &state.qd, &state.qdd),
                RootType::Floating => pinocchio.floating_state(
                    &state.q,
                    &state.qd,
                    &state.qdd,
                    base.frame(),
                    base.velocity(),
                    base.acceleration(),
                ),
            };

            for algorithm in [
                AlgorithmCase::ForwardKinematics {
                    target: target_name.clone(),
                },
                AlgorithmCase::ForwardVelocity {
                    target: target_name.clone(),
                },
                AlgorithmCase::ForwardAcceleration {
                    target: target_name.clone(),
                },
                AlgorithmCase::Jacobian {
                    target: target_name.clone(),
                },
                AlgorithmCase::JacobianDerivative {
                    target: target_name.clone(),
                },
                AlgorithmCase::MassMatrix,
                AlgorithmCase::Gravity,
                AlgorithmCase::VelocityProduct,
                AlgorithmCase::InverseDynamics,
                AlgorithmCase::ForwardDynamics,
            ] {
                let context = TestContext::new(algorithm.name(), "generated-pinocchio")
                    .seed(seed)
                    .sample(sample)
                    .base_mode(base_mode)
                    .target(&target_name)
                    .load_case(&matrix_case.load_case);
                let actual = execute_algorithm(&mut robot, &matrix_case, &algorithm);
                let expected = match algorithm {
                    AlgorithmCase::ForwardKinematics { .. } => {
                        let (rotation, translation) = pinocchio.frame(&pin_q);
                        Ok(Observation::Frame(Frame::from_parts(
                            Translation3::from(translation),
                            UnitQuaternion::from_rotation_matrix(
                                &Rotation3::from_matrix_unchecked(rotation),
                            ),
                        )))
                    }
                    AlgorithmCase::ForwardVelocity { .. } => Ok(Observation::Twist(twist(
                        &pinocchio.velocity(&pin_q, &pin_qd),
                    ))),
                    AlgorithmCase::ForwardAcceleration { .. } => Ok(Observation::Twist(twist(
                        &pinocchio.acceleration(&pin_q, &pin_qd, &pin_qdd),
                    ))),
                    AlgorithmCase::Jacobian { .. } => {
                        let values = match base_mode {
                            RootType::Fixed => pinocchio.jacobian(&pin_q),
                            RootType::Floating => pinocchio.floating_jacobian(&pin_q, base.frame()),
                        };
                        Ok(Observation::Matrix {
                            rows: 6,
                            columns: robot.generalized_count(),
                            values,
                        })
                    }
                    AlgorithmCase::JacobianDerivative { .. } => {
                        let values = match base_mode {
                            RootType::Fixed => pinocchio.jacobian_derivative(&pin_q, &pin_qd),
                            RootType::Floating => pinocchio.floating_jacobian_derivative(
                                &pin_q,
                                &pin_qd,
                                base.frame(),
                                base.velocity().angular,
                            ),
                        };
                        Ok(Observation::Matrix {
                            rows: 6,
                            columns: robot.generalized_count(),
                            values,
                        })
                    }
                    AlgorithmCase::MassMatrix => {
                        let values = match base_mode {
                            RootType::Fixed => pinocchio.mass_matrix(&pin_q),
                            RootType::Floating => {
                                pinocchio.floating_mass_matrix(&pin_q, base.frame())
                            }
                        };
                        let n = robot.generalized_count();
                        Ok(Observation::Matrix {
                            rows: n,
                            columns: n,
                            values,
                        })
                    }
                    AlgorithmCase::Gravity => Ok(Observation::Vector(match base_mode {
                        RootType::Fixed => pinocchio.gravity_with_loads(&pin_q, &pinocchio_loads),
                        RootType::Floating => pinocchio.floating_gravity_with_loads(
                            &pin_q,
                            base.frame(),
                            &pinocchio_loads,
                        ),
                    })),
                    AlgorithmCase::VelocityProduct => {
                        let zero = vec![0.0; state.q.len()];
                        let values = match base_mode {
                            RootType::Fixed => {
                                let (_, _, pin_zero) = pinocchio.state(&state.q, &state.qd, &zero);
                                let bias = pinocchio.rnea(&pin_q, &pin_qd, &pin_zero);
                                let gravity = pinocchio.gravity(&pin_q);
                                bias.iter().zip(gravity).map(|(b, g)| b - g).collect()
                            }
                            RootType::Floating => {
                                let (velocity_q, velocity_qd, velocity_qdd) = pinocchio
                                    .floating_state(
                                        &state.q,
                                        &state.qd,
                                        &zero,
                                        base.frame(),
                                        base.velocity(),
                                        Twist::zeros(),
                                    );
                                let bias = pinocchio.floating_rnea_with_loads(
                                    &velocity_q,
                                    &velocity_qd,
                                    &velocity_qdd,
                                    base.frame(),
                                    &[],
                                );
                                let gravity = pinocchio.floating_gravity(&velocity_q, base.frame());
                                bias.iter().zip(gravity).map(|(b, g)| b - g).collect()
                            }
                        };
                        Ok(Observation::Vector(values))
                    }
                    AlgorithmCase::InverseDynamics => Ok(Observation::Vector(match base_mode {
                        RootType::Fixed => {
                            pinocchio.rnea_with_loads(&pin_q, &pin_qd, &pin_qdd, &pinocchio_loads)
                        }
                        RootType::Floating => pinocchio.floating_rnea_with_loads(
                            &pin_q,
                            &pin_qd,
                            &pin_qdd,
                            base.frame(),
                            &pinocchio_loads,
                        ),
                    })),
                    AlgorithmCase::ForwardDynamics => Ok(Observation::Vector(match base_mode {
                        RootType::Fixed => {
                            pinocchio.aba_with_loads(&pin_q, &pin_qd, &state.tau, &pinocchio_loads)
                        }
                        RootType::Floating => pinocchio.floating_aba_with_loads(
                            &pin_q,
                            &pin_qd,
                            &state.tau,
                            base.frame(),
                            base.velocity(),
                            &pinocchio_loads,
                        ),
                    })),
                };
                assert_observation_close(&actual, &expected, DYNAMICS, &context);
            }
        }
    }
}

#[test]
fn generated_floating_models_match_pinocchio() {
    for case in selected_model_cases(24) {
        if case.options.base_mode != RootType::Floating {
            continue;
        }
        let generated = generate_case(&case);
        let mut robot = FloatingRobot::from_urdf(generated.path()).unwrap();
        let mut contexts = HashMap::<String, PinocchioContext>::new();
        for sample in 0..8 {
            let target_name = generated.metadata.branch_targets
                [sample % generated.metadata.branch_targets.len()]
            .clone();
            let pinocchio = contexts.entry(target_name.clone()).or_insert_with(|| {
                PinocchioContext::new_floating(&robot, generated.path(), &target_name)
            });
            let base = support::states::deterministic_base_state(sample);
            let mut state = deterministic_joint_state(robot.joint_count(), sample);
            let base_tau =
                (0..6).map(|index| 5.0 * ((sample + 1) as f64 * (index + 2) as f64 * 0.281).sin());
            state.tau = base_tau.chain(state.tau).collect();
            let load_specs = if sample.is_multiple_of(2) {
                vec![]
            } else {
                vec![LoadSpec::new(
                    &target_name,
                    Wrench::new(
                        Vector3::new(0.23, -0.17, 0.11),
                        Vector3::new(-0.7, 0.4, -0.2),
                    ),
                )]
            };
            let matrix_case = MatrixCase {
                state: state.clone(),
                base,
                loads: load_specs.clone(),
                load_case: if load_specs.is_empty() {
                    "none"
                } else {
                    "target"
                }
                .to_owned(),
            };
            let pinocchio_loads: Vec<_> = load_specs
                .iter()
                .map(|load| pinocchio.load(&load.link_name, load.wrench))
                .collect();
            let (pin_q, pin_qd, pin_qdd) = pinocchio.floating_state(
                &state.q,
                &state.qd,
                &state.qdd,
                matrix_case.base.frame(),
                matrix_case.base.velocity(),
                matrix_case.base.acceleration(),
            );
            for algorithm in [
                AlgorithmCase::ForwardKinematics {
                    target: target_name.clone(),
                },
                AlgorithmCase::ForwardVelocity {
                    target: target_name.clone(),
                },
                AlgorithmCase::ForwardAcceleration {
                    target: target_name.clone(),
                },
                AlgorithmCase::Jacobian {
                    target: target_name.clone(),
                },
                AlgorithmCase::JacobianDerivative {
                    target: target_name.clone(),
                },
                AlgorithmCase::MassMatrix,
                AlgorithmCase::Gravity,
                AlgorithmCase::VelocityProduct,
                AlgorithmCase::InverseDynamics,
                AlgorithmCase::ForwardDynamics,
            ] {
                let actual = execute_algorithm_floating(&mut robot, &matrix_case, &algorithm);
                let expected = floating_expected(
                    &mut *pinocchio,
                    &algorithm,
                    &state,
                    &matrix_case.base,
                    &pin_q,
                    &pin_qd,
                    &pin_qdd,
                    &pinocchio_loads,
                    robot.generalized_count(),
                );
                let context = TestContext::new(algorithm.name(), "generated-pinocchio")
                    .seed(case.seed)
                    .sample(sample)
                    .base_mode(RootType::Floating)
                    .target(&target_name)
                    .load_case(&matrix_case.load_case);
                assert_observation_close(&actual, &expected, DYNAMICS, &context);
            }
        }
    }
}
