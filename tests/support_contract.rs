mod support;

use std::panic::{AssertUnwindSafe, catch_unwind};

use dynibo::{BaseMode, BaseState, Twist, Wrench};
use nalgebra::Vector3;
use support::{
    context::TestContext,
    fixtures::{LoadSpec, MIXED_ARM},
    model_gen::{ModelGenOptions, corpus_model_seeds, generate_model},
    numeric::{Tolerance, assert_scalar_close, assert_slice_close},
    states::{deterministic_joint_state, generalized_acceleration},
};

#[test]
fn numeric_contract_combines_absolute_and_relative_tolerances() {
    let context = TestContext::new("numeric-contract", "support").seed(7);
    assert_scalar_close(1.0e-13, 0.0, Tolerance::new(2.0e-13, 0.0), &context);
    assert_scalar_close(1.0e9 + 0.5, 1.0e9, Tolerance::new(0.0, 1.0e-9), &context);
    assert_slice_close(
        &[1.0e-13, 1.0e9 + 0.5],
        &[0.0, 1.0e9],
        Tolerance::new(2.0e-13, 1.0e-9),
        &context,
    );
}

#[test]
fn numeric_contract_rejects_non_finite_values_and_reports_context() {
    let context = TestContext::new("nan-check", "support").seed(19).sample(3);
    let panic = catch_unwind(AssertUnwindSafe(|| {
        assert_slice_close(&[f64::NAN], &[f64::NAN], Tolerance::new(1.0, 1.0), &context);
    }))
    .expect_err("NaN values must fail even when both sides contain NaN");
    let message = if let Some(message) = panic.downcast_ref::<String>() {
        message.clone()
    } else if let Some(message) = panic.downcast_ref::<&str>() {
        (*message).to_owned()
    } else {
        String::new()
    };
    assert!(message.contains("operation=nan-check"));
    assert!(message.contains("seed=19"));
    assert!(message.contains("sample=3"));
    assert!(message.contains("index=0"));
}

#[test]
fn deterministic_state_and_generalized_order_are_stable() {
    let first = deterministic_joint_state(4, 11);
    let second = deterministic_joint_state(4, 11);
    assert_eq!(first.q, second.q);
    assert_eq!(first.qd, second.qd);
    assert_eq!(first.qdd, second.qdd);
    assert_eq!(first.tau, second.tau);

    let acceleration = generalized_acceleration(
        Twist::new(Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0)),
        &[7.0, 8.0],
    );
    assert_eq!(acceleration, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]);
}

#[test]
fn load_specs_resolve_to_model_scoped_handles_shared_by_forks() {
    let robot = MIXED_ARM.robot(BaseMode::Fixed);
    let fork = robot.fork();
    let spec = LoadSpec::new(
        "tool",
        Wrench::new(Vector3::new(0.1, 0.2, 0.3), Vector3::new(1.0, 2.0, 3.0)),
    );
    assert_eq!(spec.resolve(&robot).link, spec.resolve(&fork).link);
}

#[test]
fn generated_model_is_reproducible_and_loadable() {
    let options = ModelGenOptions {
        active_joints: 8,
        branched: true,
        include_fixed_joints: true,
        base_mode: BaseMode::Floating,
    };
    let first = generate_model(23, options);
    let second = generate_model(23, options);
    assert_eq!(first.urdf, second.urdf);
    let robot = first.robot();
    assert_eq!(robot.joint_count(), 8);
    assert_eq!(robot.base_mode(), BaseMode::Floating);
    assert!(!first.metadata.branch_targets.is_empty());

    let base = BaseState::fixed();
    assert!(
        base.frame()
            .translation
            .vector
            .iter()
            .all(|value| *value == 0.0)
    );
}

#[test]
fn default_corpus_uses_stable_stratified_pseudo_random_seeds() {
    let seeds = corpus_model_seeds(16);
    assert_eq!(seeds.len(), 16);
    assert_eq!(seeds[0], 0x1ea5_9f28_78e5_1fb4);
    assert_eq!(seeds[15], 0xf04b_5a7e_31a6_709f);
    for (index, &seed) in seeds.iter().enumerate() {
        assert_eq!(seed % 12, index as u64 % 12);
    }
    let unique: std::collections::HashSet<_> = seeds.iter().copied().collect();
    assert_eq!(unique.len(), seeds.len());
}
