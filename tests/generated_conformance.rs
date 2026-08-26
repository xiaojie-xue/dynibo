mod support;

use dynibo::{BaseMode, BaseState, Twist, Wrench};
use nalgebra::Vector3;
use support::{
    context::TestContext,
    dynamics::{dense_forward_dynamics, generalized_force_for_acceleration, inverse_dynamics_bias},
    fixtures::LoadSpec,
    matrix::{AlgorithmCase, MatrixCase, execute_algorithm},
    model_gen::{generate_case, selected_model_cases},
    numeric::{DYNAMICS, STRICT, assert_slice_close},
    observation::assert_observation_finite,
    states::{deterministic_base_state, deterministic_joint_state, generalized_acceleration},
};

#[test]
fn generated_model_matrix_preserves_dynamics_and_aba_identities() {
    for case in selected_model_cases(24) {
        let seed = case.seed;
        let options = case.options;
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
            let mut state = deterministic_joint_state(robot.joint_count(), sample);
            let base = match options.base_mode {
                BaseMode::Fixed => BaseState::fixed(),
                BaseMode::Floating => deterministic_base_state(sample),
            };
            if options.base_mode == BaseMode::Floating {
                let base_tau = (0..6)
                    .map(|index| 5.0 * ((sample + 1) as f64 * (index + 2) as f64 * 0.281).sin());
                state.tau = base_tau.chain(state.tau).collect();
            }
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
            let zero_base = match options.base_mode {
                BaseMode::Fixed => base,
                BaseMode::Floating => {
                    support::states::base_with_acceleration(&base, Twist::zeros())
                }
            };
            let mut rnea_bias = vec![f64::NAN; robot.generalized_count()];
            robot
                .inverse_dynamics(
                    &zero_base,
                    &state.q,
                    &state.qd,
                    &zero,
                    loads,
                    &mut rnea_bias,
                )
                .unwrap();
            assert_slice_close(&bias, &rnea_bias, STRICT, &context);

            let dense =
                dense_forward_dynamics(&mut robot, &base, &state.q, &state.qd, &state.tau, loads);
            let mut aba = vec![f64::NAN; robot.generalized_count()];
            robot
                .forward_dynamics(&base, &state.q, &state.qd, &state.tau, loads, &mut aba)
                .unwrap();
            assert_slice_close(&aba, &dense, DYNAMICS, &context);

            let expected_acceleration = match options.base_mode {
                BaseMode::Fixed => state.qdd.clone(),
                BaseMode::Floating => generalized_acceleration(base.acceleration(), &state.qdd),
            };
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
                .forward_dynamics(&base, &state.q, &state.qd, &forces, loads, &mut recovered)
                .unwrap();
            assert_slice_close(&recovered, &expected_acceleration, DYNAMICS, &context);
        }
    }
}
