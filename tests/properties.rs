use std::path::PathBuf;

use approx::assert_relative_eq;
use dynibo::{Frame, IndexedLoad, Robot, Workspace, Wrench};
use nalgebra::{DMatrix, Vector3};

fn tree_arm() -> Robot {
    Robot::from_urdf(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf"))
        .expect("tree fixture must load")
}

fn mixed_oracle_arm() -> Robot {
    Robot::from_urdf(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/oracle_mixed.urdf"))
        .expect("mixed oracle fixture must load")
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

/// Extracts the mass matrix column-wise from Newton-Euler evaluations:
/// `M e_j = RNEA(q, 0, e_j) - RNEA(q, 0, 0)`.
fn rnea_mass_matrix(robot: &Robot, q: &[f64], workspace: &mut Workspace) -> Vec<f64> {
    let joint_count = robot.joint_count();
    let zero = vec![0.0; joint_count];
    let mut bias = vec![0.0; joint_count];
    robot
        .inverse_dynamics(
            &dynibo::BaseState::fixed(),
            q,
            &zero,
            &zero,
            &[],
            workspace,
            &mut bias,
        )
        .unwrap();
    let mut mass = vec![0.0; joint_count * joint_count];
    for column in 0..joint_count {
        let mut unit_acceleration = vec![0.0; joint_count];
        unit_acceleration[column] = 1.0;
        let mut torque = vec![0.0; joint_count];
        robot
            .inverse_dynamics(
                &dynibo::BaseState::fixed(),
                q,
                &zero,
                &unit_acceleration,
                &[],
                workspace,
                &mut torque,
            )
            .unwrap();
        for row in 0..joint_count {
            mass[column * joint_count + row] = torque[row] - bias[row];
        }
    }
    mass
}

#[test]
fn mass_matrix_matches_rnea_columns_and_keeps_structural_guarantees() {
    for robot in [tree_arm(), mixed_oracle_arm()] {
        let joint_count = robot.joint_count();
        let mut workspace = robot.workspace();
        for sample in 0..16 {
            let q: Vec<f64> = (0..joint_count)
                .map(|joint| {
                    let argument = (sample + 1) as f64 * (joint + 2) as f64 * 0.531 + 0.2;
                    (0.9 + 0.1 * joint as f64) * argument.sin()
                })
                .collect();
            let mut mass = vec![f64::NAN; joint_count * joint_count];
            robot
                .mass_matrix(&dynibo::BaseState::fixed(), &q, &mut workspace, &mut mass)
                .unwrap();
            let expected = rnea_mass_matrix(&robot, &q, &mut workspace);
            assert_slice_close(&mass, &expected, 2.0e-11);

            for row in 0..joint_count {
                for column in 0..joint_count {
                    assert_relative_eq!(
                        mass[column * joint_count + row],
                        mass[row * joint_count + column],
                        epsilon = 1.0e-12
                    );
                }
            }
            for index in 0..joint_count {
                assert!(
                    mass[index * joint_count + index] > 0.0,
                    "mass matrix diagonal must be positive: sample={sample}, joint={index}"
                );
            }
            assert!(
                DMatrix::from_column_slice(joint_count, joint_count, &mass)
                    .cholesky()
                    .is_some(),
                "mass matrix must be positive definite: sample={sample}"
            );
        }
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
                .jacobian(
                    &dynibo::BaseState::fixed(),
                    &q,
                    target,
                    &mut workspace,
                    &mut jacobian,
                )
                .unwrap();

            for joint in 0..7 {
                let mut plus_q = q;
                let mut minus_q = q;
                plus_q[joint] += epsilon;
                minus_q[joint] -= epsilon;
                let plus = robot
                    .forward_kinematics(
                        &dynibo::BaseState::fixed(),
                        &plus_q,
                        target,
                        &mut workspace,
                    )
                    .unwrap();
                let minus = robot
                    .forward_kinematics(
                        &dynibo::BaseState::fixed(),
                        &minus_q,
                        target,
                        &mut workspace,
                    )
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

fn sample_state(joint_count: usize, sample: usize, phase: f64, amplitude: f64) -> Vec<f64> {
    (0..joint_count)
        .map(|joint| {
            let argument = (sample + 1) as f64 * (joint + 2) as f64 * 0.531 + phase;
            amplitude * (1.0 + 0.1 * joint as f64) * argument.sin()
        })
        .collect()
}

#[test]
fn velocity_product_matches_rnea_minus_gravity() {
    for robot in [tree_arm(), mixed_oracle_arm()] {
        let joint_count = robot.joint_count();
        let zero = vec![0.0; joint_count];
        let mut workspace = robot.workspace();
        for sample in 0..16 {
            let q = sample_state(joint_count, sample, 0.2, 0.9);
            let qd = sample_state(joint_count, sample, 0.9, 0.7);
            let mut velocity_product = vec![f64::NAN; robot.generalized_count()];
            robot
                .velocity_product_forces(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    &mut workspace,
                    &mut velocity_product,
                )
                .unwrap();

            let mut gravity = vec![0.0; robot.generalized_count()];
            robot
                .gravity(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &[],
                    &mut workspace,
                    &mut gravity,
                )
                .unwrap();
            let mut bias = vec![0.0; robot.generalized_count()];
            robot
                .inverse_dynamics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &qd,
                    &zero,
                    &[],
                    &mut workspace,
                    &mut bias,
                )
                .unwrap();
            let expected: Vec<f64> = bias
                .iter()
                .zip(&gravity)
                .map(|(bias, gravity)| bias - gravity)
                .collect();
            assert_slice_close(&velocity_product, &expected, 2.0e-11);

            let mut zero_product = vec![f64::NAN; robot.generalized_count()];
            robot
                .velocity_product_forces(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &zero,
                    &mut workspace,
                    &mut zero_product,
                )
                .unwrap();
            assert_slice_close(
                &zero_product,
                &vec![0.0; robot.generalized_count()],
                1.0e-14,
            );
        }
    }
}

#[test]
fn jacobian_derivative_matches_zero_acceleration_and_finite_difference() {
    let epsilon = 1.0e-7;
    for robot in [tree_arm(), mixed_oracle_arm()] {
        let joint_count = robot.joint_count();
        let leaf_names: Vec<String> = robot
            .leaf_links()
            .map(|link| link.name().to_owned())
            .collect();
        for leaf_name in leaf_names {
            let target = robot.link_id(&leaf_name).unwrap();
            let mut workspace = robot.workspace();
            for sample in 0..8 {
                let q = sample_state(joint_count, sample, 0.3, 0.85);
                let qd = sample_state(joint_count, sample, 1.2, 0.75);
                let zero = vec![0.0; joint_count];

                let mut derivative = vec![f64::NAN; 6 * joint_count];
                robot
                    .jacobian_derivative(
                        &dynibo::BaseState::fixed(),
                        &q,
                        &qd,
                        target,
                        &mut workspace,
                        &mut derivative,
                    )
                    .unwrap();

                // J_dot qd == forward_acceleration(q, qd, 0).
                let acceleration = robot
                    .forward_acceleration_kinematics(
                        &dynibo::BaseState::fixed(),
                        &q,
                        &qd,
                        &zero,
                        target,
                        &mut workspace,
                    )
                    .unwrap();
                let contracted: Vec<f64> = (0..6)
                    .map(|row| {
                        (0..joint_count)
                            .map(|joint| derivative[6 * joint + row] * qd[joint])
                            .sum()
                    })
                    .collect();
                assert_slice_close(&contracted, acceleration.to_vector().as_slice(), 2.0e-11);

                // Central finite difference of J along qd.
                let plus_q: Vec<f64> = q.iter().zip(&qd).map(|(q, qd)| q + epsilon * qd).collect();
                let minus_q: Vec<f64> = q.iter().zip(&qd).map(|(q, qd)| q - epsilon * qd).collect();
                let mut plus = vec![0.0; 6 * joint_count];
                let mut minus = vec![0.0; 6 * joint_count];
                robot
                    .jacobian(
                        &dynibo::BaseState::fixed(),
                        &plus_q,
                        target,
                        &mut workspace,
                        &mut plus,
                    )
                    .unwrap();
                robot
                    .jacobian(
                        &dynibo::BaseState::fixed(),
                        &minus_q,
                        target,
                        &mut workspace,
                        &mut minus,
                    )
                    .unwrap();
                for index in 0..6 * joint_count {
                    assert_relative_eq!(
                        derivative[index],
                        (plus[index] - minus[index]) / (2.0 * epsilon),
                        epsilon = 1.0e-6
                    );
                }
            }
        }

        // The root link has no moving ancestor joints.
        let root = robot.link_id(robot.root_link().name()).unwrap();
        let mut workspace = robot.workspace();
        let q = sample_state(joint_count, 0, 0.3, 0.85);
        let qd = sample_state(joint_count, 0, 1.2, 0.75);
        let mut derivative = vec![f64::NAN; 6 * joint_count];
        robot
            .jacobian_derivative(
                &dynibo::BaseState::fixed(),
                &q,
                &qd,
                root,
                &mut workspace,
                &mut derivative,
            )
            .unwrap();
        assert_slice_close(&derivative, &vec![0.0; 6 * joint_count], 0.0);
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
        let base = dynibo::BaseState::fixed_at(base).unwrap();
        let mut gravity = [0.0; 7];
        let mut inverse = [0.0; 7];
        robot
            .gravity(&base, &q, &[], &mut workspace, &mut gravity)
            .unwrap();
        robot
            .inverse_dynamics(&base, &q, &zero, &zero, &[], &mut workspace, &mut inverse)
            .unwrap();
        assert_slice_close(&gravity, &inverse, 3.0e-12);

        let mut left_only = [0.0; 7];
        let mut right_only = [0.0; 7];
        let mut both = [0.0; 7];
        robot
            .gravity(&base, &q, &[left_load], &mut workspace, &mut left_only)
            .unwrap();
        robot
            .gravity(&base, &q, &[right_load], &mut workspace, &mut right_only)
            .unwrap();
        robot
            .gravity(
                &base,
                &q,
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

#[test]
fn mixed_joint_mass_matrix_is_symmetric_and_positive_on_moving_coordinates() {
    let robot = mixed_oracle_arm();
    let mut workspace = robot.workspace();
    let zero = [0.0; 3];

    for sample in 0..16 {
        let q = [
            1.4 * ((sample + 1) as f64 * 0.71).sin(),
            0.3 * ((sample + 1) as f64 * 0.37).sin(),
            2.5 * ((sample + 1) as f64 * 0.29).sin(),
        ];
        let mut bias = [0.0; 3];
        robot
            .inverse_dynamics(
                &dynibo::BaseState::fixed(),
                &q,
                &zero,
                &zero,
                &[],
                &mut workspace,
                &mut bias,
            )
            .unwrap();

        let mut mass = DMatrix::<f64>::zeros(3, 3);
        for column in 0..3 {
            let mut unit_acceleration = [0.0; 3];
            unit_acceleration[column] = 1.0;
            let mut torque = [0.0; 3];
            robot
                .inverse_dynamics(
                    &dynibo::BaseState::fixed(),
                    &q,
                    &zero,
                    &unit_acceleration,
                    &[],
                    &mut workspace,
                    &mut torque,
                )
                .unwrap();
            for row in 0..3 {
                mass[(row, column)] = torque[row] - bias[row];
            }
        }

        assert_relative_eq!(mass, mass.transpose(), epsilon = 2.0e-11);
        for direction in [[1.0, -0.4, 0.7], [-0.3, 1.2, -0.8], [0.6, 0.2, 1.4]] {
            let direction = nalgebra::DVector::from_column_slice(&direction);
            let energy = direction.dot(&(&mass * &direction));
            assert!(
                energy > 1.0e-8,
                "mass matrix is not positive on moving coordinates: sample={sample}, energy={energy}, mass={mass:?}"
            );
        }
    }
}
