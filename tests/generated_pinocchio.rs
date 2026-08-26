#![cfg(feature = "pinocchio-tests")]

mod support;

use std::collections::HashMap;

use dynibo::{BaseMode, BaseState, Frame, Twist, Wrench};
use nalgebra::{Rotation3, Translation3, UnitQuaternion, Vector3};
use support::{
    context::TestContext,
    fixtures::LoadSpec,
    matrix::{AlgorithmCase, MatrixCase, execute_algorithm},
    model_gen::{generate_case, selected_model_cases},
    numeric::DYNAMICS,
    observation::{Observation, assert_observation_close},
    pinocchio::PinocchioContext,
    states::{deterministic_base_state, deterministic_joint_state},
};

fn twist(values: &[f64]) -> Twist {
    Twist::new(
        Vector3::from_column_slice(&values[..3]),
        Vector3::from_column_slice(&values[3..]),
    )
}

#[test]
fn generated_models_match_pinocchio() {
    for case in selected_model_cases(24) {
        let seed = case.seed;
        let base_mode = case.options.base_mode;
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
            let mut state = deterministic_joint_state(robot.joint_count(), sample);
            let base = match base_mode {
                BaseMode::Fixed => BaseState::fixed(),
                BaseMode::Floating => deterministic_base_state(sample),
            };
            if base_mode == BaseMode::Floating {
                let base_tau = (0..6)
                    .map(|index| 5.0 * ((sample + 1) as f64 * (index + 2) as f64 * 0.281).sin());
                state.tau = base_tau.chain(state.tau).collect();
            }
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
                if base_mode == BaseMode::Fixed {
                    PinocchioContext::new(&robot, generated.path(), &target_name)
                } else {
                    PinocchioContext::new_floating(&robot, generated.path(), &target_name)
                }
            });
            let pinocchio_loads: Vec<_> = load_specs
                .iter()
                .map(|load| pinocchio.load(&load.link_name, load.wrench))
                .collect();
            let (pin_q, pin_qd, pin_qdd) = match base_mode {
                BaseMode::Fixed => pinocchio.state(&state.q, &state.qd, &state.qdd),
                BaseMode::Floating => pinocchio.floating_state(
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
                            BaseMode::Fixed => pinocchio.jacobian(&pin_q),
                            BaseMode::Floating => pinocchio.floating_jacobian(&pin_q, base.frame()),
                        };
                        Ok(Observation::Matrix {
                            rows: 6,
                            columns: robot.generalized_count(),
                            values,
                        })
                    }
                    AlgorithmCase::JacobianDerivative { .. } => {
                        let values = match base_mode {
                            BaseMode::Fixed => pinocchio.jacobian_derivative(&pin_q, &pin_qd),
                            BaseMode::Floating => pinocchio.floating_jacobian_derivative(
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
                            BaseMode::Fixed => pinocchio.mass_matrix(&pin_q),
                            BaseMode::Floating => {
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
                        BaseMode::Fixed => pinocchio.gravity_with_loads(&pin_q, &pinocchio_loads),
                        BaseMode::Floating => pinocchio.floating_gravity_with_loads(
                            &pin_q,
                            base.frame(),
                            &pinocchio_loads,
                        ),
                    })),
                    AlgorithmCase::VelocityProduct => {
                        let zero = vec![0.0; state.q.len()];
                        let values = match base_mode {
                            BaseMode::Fixed => {
                                let (_, _, pin_zero) = pinocchio.state(&state.q, &state.qd, &zero);
                                let bias = pinocchio.rnea(&pin_q, &pin_qd, &pin_zero);
                                let gravity = pinocchio.gravity(&pin_q);
                                bias.iter().zip(gravity).map(|(b, g)| b - g).collect()
                            }
                            BaseMode::Floating => {
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
                        BaseMode::Fixed => {
                            pinocchio.rnea_with_loads(&pin_q, &pin_qd, &pin_qdd, &pinocchio_loads)
                        }
                        BaseMode::Floating => pinocchio.floating_rnea_with_loads(
                            &pin_q,
                            &pin_qd,
                            &pin_qdd,
                            base.frame(),
                            &pinocchio_loads,
                        ),
                    })),
                    AlgorithmCase::ForwardDynamics => Ok(Observation::Vector(match base_mode {
                        BaseMode::Fixed => {
                            pinocchio.aba_with_loads(&pin_q, &pin_qd, &state.tau, &pinocchio_loads)
                        }
                        BaseMode::Floating => pinocchio.floating_aba_with_loads(
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
