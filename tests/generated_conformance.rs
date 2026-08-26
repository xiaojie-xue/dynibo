mod support;

use support::context::TestBaseMode as BaseMode;

use dynibo::{BaseState, FloatingRobot, Frame, IndexedLoad, Wrench};
use nalgebra::Vector3;
use support::{
    context::TestContext,
    dynamics::{dense_forward_dynamics, generalized_force_for_acceleration, inverse_dynamics_bias},
    fixtures::LoadSpec,
    matrix::{AlgorithmCase, MatrixCase, execute_algorithm},
    model_gen::{generate_case, selected_model_cases},
    numeric::{DYNAMICS, STRICT, assert_slice_close},
    observation::assert_observation_finite,
    states::{deterministic_base_state, deterministic_joint_state},
};

#[test]
fn generated_model_matrix_preserves_dynamics_and_aba_identities() {
    for case in selected_model_cases(24) {
        let seed = case.seed;
        let options = case.options;
        if options.base_mode != BaseMode::Fixed {
            continue;
        }
        let generated = generate_case(&case);
        let mut robot = generated.robot();

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
            let load = load_spec.resolve(&robot);
            let state = deterministic_joint_state(robot.joint_count(), sample);
            let base = BaseState::stationary(Frame::identity()).unwrap();
            let loads = if sample.is_multiple_of(2) {
                &[][..]
            } else {
                &[load][..]
            };
            let context = TestContext::new("generated-dynamics", "generated-urdf")
                .seed(seed)
                .sample(sample)
                .base_mode(options.base_mode)
                .target(&target_name)
                .load_case(if loads.is_empty() { "none" } else { "tool" });

            let matrix_case = MatrixCase {
                state: state.clone(),
                base,
                loads: if loads.is_empty() {
                    vec![]
                } else {
                    vec![load_spec.clone()]
                },
                load_case: if loads.is_empty() { "none" } else { "tool" }.to_owned(),
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
                let algorithm_context = TestContext {
                    operation: algorithm.name().to_owned(),
                    ..context.clone()
                };
                let observation = execute_algorithm(&mut robot, &matrix_case, &algorithm);
                assert_observation_finite(&observation, &algorithm_context);
            }

            let bias = inverse_dynamics_bias(&mut robot, &base, &state.q, &state.qd, loads);
            let zero = vec![0.0; robot.joint_count()];
            let mut rnea_bias = vec![f64::NAN; robot.generalized_count()];
            robot
                .inverse_dynamics(&state.q, &state.qd, &zero, loads, &mut rnea_bias)
                .unwrap();
            assert_slice_close(&bias, &rnea_bias, STRICT, &context);

            let dense =
                dense_forward_dynamics(&mut robot, &base, &state.q, &state.qd, &state.tau, loads);
            let mut aba = vec![f64::NAN; robot.generalized_count()];
            robot
                .forward_dynamics(&state.q, &state.qd, &state.tau, loads, &mut aba)
                .unwrap();
            assert_slice_close(&aba, &dense, DYNAMICS, &context);

            let expected_acceleration = state.qdd.clone();
            let forces = generalized_force_for_acceleration(
                &mut robot,
                &base,
                &state.q,
                &state.qd,
                &expected_acceleration,
                loads,
            );
            let mut recovered = vec![f64::NAN; robot.generalized_count()];
            robot
                .forward_dynamics(&state.q, &state.qd, &forces, loads, &mut recovered)
                .unwrap();
            assert_slice_close(&recovered, &expected_acceleration, DYNAMICS, &context);
        }
    }
}

#[test]
fn generated_floating_models_produce_finite_results() {
    for case in selected_model_cases(24) {
        if case.options.base_mode != BaseMode::Floating {
            continue;
        }
        let generated = generate_case(&case);
        let mut robot = FloatingRobot::from_urdf(generated.path()).unwrap();
        let target_name = &generated.metadata.branch_targets[0];
        let target = robot.link_id(target_name).unwrap();

        for sample in 0..8 {
            let base = deterministic_base_state(sample);
            let state = deterministic_joint_state(robot.joint_count(), sample);
            let load = IndexedLoad {
                link: target,
                wrench: Wrench::new(
                    Vector3::new(0.23, -0.17, 0.11),
                    Vector3::new(-0.7, 0.4, -0.2),
                ),
            };
            let loads = if sample.is_multiple_of(2) {
                &[][..]
            } else {
                &[load][..]
            };
            let n = robot.generalized_count();
            let mut jacobian = vec![f64::NAN; 6 * n];
            let mut mass = vec![f64::NAN; n * n];
            let mut gravity = vec![f64::NAN; n];
            let mut forces = vec![f64::NAN; n];
            let mut acceleration = vec![f64::NAN; n];
            let tau: Vec<_> = (0..6)
                .map(|index| 0.3 * (sample + index + 1) as f64)
                .chain(state.tau.iter().copied())
                .collect();

            let frame = robot.forward_kinematics(&base, &state.q, target).unwrap();
            assert!(
                frame
                    .translation
                    .vector
                    .iter()
                    .all(|value| value.is_finite())
            );
            robot
                .jacobian(&base, &state.q, target, &mut jacobian)
                .unwrap();
            robot.mass_matrix(&base, &state.q, &mut mass).unwrap();
            robot.gravity(&base, &state.q, loads, &mut gravity).unwrap();
            robot
                .inverse_dynamics(&base, &state.q, &state.qd, &state.qdd, loads, &mut forces)
                .unwrap();
            robot
                .forward_dynamics(&base, &state.q, &state.qd, &tau, loads, &mut acceleration)
                .unwrap();

            for values in [&jacobian, &mass, &gravity, &forces, &acceleration] {
                assert!(values.iter().all(|value| value.is_finite()));
            }
        }
    }
}
