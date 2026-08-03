use std::path::PathBuf;

use approx::assert_relative_eq;
use dyno::{Frame, IndexedLoad, Robot, Twist, Wrench};
use nalgebra::Vector3;

fn tree_arm() -> Robot {
    Robot::from_urdf(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("benches/data/test_tree_7.urdf"),
    )
    .expect("tree fixture must load")
}

fn deterministic_state(sample: usize, phase: f64, amplitude: f64) -> [f64; 7] {
    std::array::from_fn(|joint| {
        let argument = (sample + 1) as f64 * (joint + 3) as f64 * 0.731 + phase;
        amplitude * argument.sin()
    })
}

fn assert_slice_close(actual: &[f64], expected: &[f64], epsilon: f64) {
    assert_eq!(actual.len(), expected.len());
    for (&actual, &expected) in actual.iter().zip(expected) {
        assert_relative_eq!(actual, expected, epsilon = epsilon);
    }
}

#[test]
fn deterministic_tree_jacobians_match_finite_difference() {
    let robot = tree_arm();
    let epsilon = 1.0e-7;

    for target_name in ["left_tool", "right_tool"] {
        let target = robot.link_id(target_name).unwrap();
        let mut workspace = robot.workspace();
        for sample in 0..16 {
            let q = deterministic_state(sample, 0.0, 0.8);
            let mut jacobian = [0.0; 42];
            robot
                .jacobian(&q, target, &mut workspace, &mut jacobian)
                .unwrap();

            for joint in 0..7 {
                let mut plus_q = q;
                let mut minus_q = q;
                plus_q[joint] += epsilon;
                minus_q[joint] -= epsilon;
                let plus = robot
                    .forward_kinematics(&plus_q, target, &mut workspace)
                    .unwrap();
                let minus = robot
                    .forward_kinematics(&minus_q, target, &mut workspace)
                    .unwrap();
                let angular =
                    (plus.rotation * minus.rotation.inverse()).scaled_axis() / (2.0 * epsilon);
                let linear = (plus.translation.vector - minus.translation.vector) / (2.0 * epsilon);
                for row in 0..3 {
                    assert_relative_eq!(jacobian[6 * joint + row], angular[row], epsilon = 3.0e-8);
                    assert_relative_eq!(
                        jacobian[6 * joint + row + 3],
                        linear[row],
                        epsilon = 3.0e-8
                    );
                }
            }
        }
    }
}

#[test]
fn deterministic_dynamics_preserve_gravity_and_load_invariants() {
    let robot = tree_arm();
    let left = robot.link_id("left_tool").unwrap();
    let right = robot.link_id("right_tool").unwrap();
    let left_load = IndexedLoad {
        link: left,
        wrench: Wrench::new(Vector3::new(0.3, -0.2, 0.4), Vector3::new(1.0, 0.5, -0.7)),
    };
    let right_load = IndexedLoad {
        link: right,
        wrench: Wrench::new(Vector3::new(-0.4, 0.1, 0.2), Vector3::new(-0.6, 0.8, 0.3)),
    };
    let zero = [0.0; 7];
    let mut workspace = robot.workspace();

    for sample in 0..32 {
        let q = deterministic_state(sample, 0.2, 0.85);
        let base = Frame::rotation(Vector3::new(
            0.2 * (sample as f64 * 0.31).sin(),
            -0.15 * (sample as f64 * 0.47).cos(),
            0.1 * (sample as f64 * 0.23).sin(),
        ));
        let mut gravity = [0.0; 7];
        let mut inverse = [0.0; 7];
        robot
            .gravity(&q, &base, &[], &mut workspace, &mut gravity)
            .unwrap();
        robot
            .inverse_dynamics(
                &q,
                &zero,
                &zero,
                &base,
                Twist::zeros(),
                Twist::zeros(),
                &[],
                &mut workspace,
                &mut inverse,
            )
            .unwrap();
        assert_slice_close(&gravity, &inverse, 3.0e-12);

        let mut left_only = [0.0; 7];
        let mut right_only = [0.0; 7];
        let mut both = [0.0; 7];
        robot
            .gravity(&q, &base, &[left_load], &mut workspace, &mut left_only)
            .unwrap();
        robot
            .gravity(&q, &base, &[right_load], &mut workspace, &mut right_only)
            .unwrap();
        robot
            .gravity(
                &q,
                &base,
                &[left_load, right_load],
                &mut workspace,
                &mut both,
            )
            .unwrap();
        let expected: [f64; 7] =
            std::array::from_fn(|joint| left_only[joint] + right_only[joint] - gravity[joint]);
        assert_slice_close(&both, &expected, 4.0e-12);
    }
}
