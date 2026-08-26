mod support;

use dynibo::{BaseMode, BaseState, Robot, Wrench};
use nalgebra::Vector3;
use support::{
    fixtures::{FLOATING_ARM, LoadSpec, TREE_ARM},
    matrix::MatrixCase,
    operations::{deterministic_operation_sequence, run_workspace_sequence},
    states::{deterministic_base_state, deterministic_joint_state},
};

#[test]
fn fixed_and_floating_workspace_sequences_match_clean_forks_step_by_step() {
    for (fixture, base_mode, seed) in [
        (TREE_ARM, BaseMode::Fixed, 101_u64),
        (FLOATING_ARM, BaseMode::Floating, 202_u64),
    ] {
        let prototype = fixture.robot(base_mode);
        let target_names: Vec<_> = fixture
            .targets
            .iter()
            .map(|name| (*name).to_owned())
            .collect();
        let load = LoadSpec::new(
            target_names[0].clone(),
            Wrench::new(Vector3::new(0.3, -0.2, 0.4), Vector3::new(1.0, 0.5, -0.7)),
        );
        let cases: Vec<_> = (0..4)
            .map(|sample| {
                let mut state = deterministic_joint_state(prototype.joint_count(), sample);
                if base_mode == BaseMode::Floating {
                    let base_tau = (0..6).map(|index| 0.3 * (sample + index + 1) as f64);
                    state.tau = base_tau.chain(state.tau).collect();
                }
                MatrixCase {
                    state,
                    base: match base_mode {
                        BaseMode::Fixed => BaseState::fixed(),
                        BaseMode::Floating => deterministic_base_state(sample),
                    },
                    loads: match sample {
                        0 => vec![load.clone()],
                        1 => vec![],
                        2 => vec![load.clone(), load.clone()],
                        _ => vec![load.clone()],
                    },
                    load_case: match sample {
                        0 => "single",
                        1 => "none",
                        2 => "duplicate",
                        _ => "single-alternate",
                    }
                    .to_owned(),
                }
            })
            .collect();
        let operations = deterministic_operation_sequence(cases.len(), &target_names, seed);

        let foreign = Robot::from_urdf(fixture.path()).unwrap();
        let foreign_link = foreign.link_id(fixture.targets[0]).unwrap();
        run_workspace_sequence(
            &prototype,
            &cases,
            &operations,
            foreign_link,
            fixture.name,
            seed,
        );
    }
}
