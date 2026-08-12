use std::path::PathBuf;

use approx::assert_relative_eq;
use dynibo::{Frame, IndexedLoad, JointType, Robot, Twist, Workspace, Wrench};
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
            q,
            &zero,
            &zero,
            &Frame::identity(),
            Twist::zeros(),
            Twist::zeros(),
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
                q,
                &zero,
                &unit_acceleration,
                &Frame::identity(),
                Twist::zeros(),
                Twist::zeros(),
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
            robot.mass_matrix(&q, &mut workspace, &mut mass).unwrap();
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
            let mut moving = Vec::new();
            for (index, joint) in robot.joints().iter().enumerate() {
                if joint.joint_type() == JointType::Fixed {
                    for other in 0..joint_count {
                        assert_eq!(mass[index * joint_count + other], 0.0);
                        assert_eq!(mass[other * joint_count + index], 0.0);
                    }
                } else {
                    assert!(
                        mass[index * joint_count + index] > 0.0,
                        "mass matrix diagonal must be positive on moving joints: sample={sample}, joint={index}"
                    );
                    moving.push(index);
                }
            }
            let moving_mass = DMatrix::from_fn(moving.len(), moving.len(), |row, column| {
                mass[moving[column] * joint_count + moving[row]]
            });
            assert!(
                moving_mass.cholesky().is_some(),
                "moving-joint mass submatrix must be positive definite: sample={sample}"
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

/// Polarization extraction of the Coriolis matrix from public bias-force
/// evaluations: `C e_j = [b(qd+e_j) - b(qd) - b(e_j) + b(0)] / 2` with
/// `b(v) = RNEA(q, v, 0)`. Slow but obviously correct cross-check oracle.
fn polarization_coriolis_matrix(
    robot: &Robot,
    q: &[f64],
    qd: &[f64],
    workspace: &mut Workspace,
) -> Vec<f64> {
    let joint_count = robot.joint_count();
    let zero = vec![0.0; joint_count];
    let bias = |velocity: &[f64], workspace: &mut Workspace| {
        let mut output = vec![0.0; joint_count];
        robot
            .inverse_dynamics(
                q,
                velocity,
                &zero,
                &Frame::identity(),
                Twist::zeros(),
                Twist::zeros(),
                &[],
                workspace,
                &mut output,
            )
            .unwrap();
        output
    };
    let base = bias(qd, workspace);
    let zero_bias = bias(&zero, workspace);
    let mut coriolis = vec![0.0; joint_count * joint_count];
    let mut perturbed = qd.to_vec();
    let mut unit = vec![0.0; joint_count];
    for column in 0..joint_count {
        perturbed[column] += 1.0;
        unit[column] = 1.0;
        let plus = bias(&perturbed, workspace);
        let unit_bias = bias(&unit, workspace);
        perturbed[column] -= 1.0;
        unit[column] = 0.0;
        for row in 0..joint_count {
            coriolis[column * joint_count + row] =
                0.5 * (plus[row] - base[row] - unit_bias[row] + zero_bias[row]);
        }
    }
    coriolis
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
fn coriolis_matrix_matches_bias_identity_and_polarization() {
    for robot in [tree_arm(), mixed_oracle_arm()] {
        let joint_count = robot.joint_count();
        let mut workspace = robot.workspace();
        for sample in 0..16 {
            let q = sample_state(joint_count, sample, 0.2, 0.9);
            let qd = sample_state(joint_count, sample, 0.9, 0.7);
            let mut coriolis = vec![f64::NAN; joint_count * joint_count];
            robot
                .coriolis_matrix(&q, &qd, &mut workspace, &mut coriolis)
                .unwrap();

            // C(q, qd) qd + g(q) == RNEA(q, qd, 0).
            let zero = vec![0.0; joint_count];
            let mut gravity = vec![0.0; joint_count];
            robot
                .gravity(&q, &Frame::identity(), &[], &mut workspace, &mut gravity)
                .unwrap();
            let mut bias = vec![0.0; joint_count];
            robot
                .inverse_dynamics(
                    &q,
                    &qd,
                    &zero,
                    &Frame::identity(),
                    Twist::zeros(),
                    Twist::zeros(),
                    &[],
                    &mut workspace,
                    &mut bias,
                )
                .unwrap();
            let reconstructed: Vec<f64> = (0..joint_count)
                .map(|row| {
                    gravity[row]
                        + (0..joint_count)
                            .map(|column| coriolis[column * joint_count + row] * qd[column])
                            .sum::<f64>()
                })
                .collect();
            assert_slice_close(&reconstructed, &bias, 2.0e-11);

            let expected = polarization_coriolis_matrix(&robot, &q, &qd, &mut workspace);
            assert_slice_close(&coriolis, &expected, 1.0e-8);

            for (index, joint) in robot.joints().iter().enumerate() {
                if joint.joint_type() == JointType::Fixed {
                    for other in 0..joint_count {
                        assert_eq!(coriolis[index * joint_count + other], 0.0);
                        assert_eq!(coriolis[other * joint_count + index], 0.0);
                    }
                }
            }

            let mut zero_coriolis = vec![f64::NAN; joint_count * joint_count];
            robot
                .coriolis_matrix(&q, &zero, &mut workspace, &mut zero_coriolis)
                .unwrap();
            assert_slice_close(
                &zero_coriolis,
                &vec![0.0; joint_count * joint_count],
                1.0e-14,
            );
        }
    }
}

#[test]
fn coriolis_matrix_makes_mass_rate_minus_twice_c_skew_symmetric() {
    let epsilon = 1.0e-6;
    for robot in [tree_arm(), mixed_oracle_arm()] {
        let joint_count = robot.joint_count();
        let mut workspace = robot.workspace();
        for sample in 0..8 {
            let q = sample_state(joint_count, sample, 0.4, 0.8);
            let qd = sample_state(joint_count, sample, 1.1, 0.6);
            let mut coriolis = vec![0.0; joint_count * joint_count];
            robot
                .coriolis_matrix(&q, &qd, &mut workspace, &mut coriolis)
                .unwrap();
            let plus_q: Vec<f64> = q.iter().zip(&qd).map(|(q, qd)| q + epsilon * qd).collect();
            let minus_q: Vec<f64> = q.iter().zip(&qd).map(|(q, qd)| q - epsilon * qd).collect();
            let mut plus_mass = vec![0.0; joint_count * joint_count];
            let mut minus_mass = vec![0.0; joint_count * joint_count];
            robot
                .mass_matrix(&plus_q, &mut workspace, &mut plus_mass)
                .unwrap();
            robot
                .mass_matrix(&minus_q, &mut workspace, &mut minus_mass)
                .unwrap();
            for row in 0..joint_count {
                for column in 0..joint_count {
                    let mass_rate = (plus_mass[column * joint_count + row]
                        - minus_mass[column * joint_count + row])
                        / (2.0 * epsilon);
                    let skew = mass_rate - 2.0 * coriolis[column * joint_count + row];
                    let skew_transpose = (plus_mass[row * joint_count + column]
                        - minus_mass[row * joint_count + column])
                        / (2.0 * epsilon)
                        - 2.0 * coriolis[row * joint_count + column];
                    assert_relative_eq!(skew, -skew_transpose, epsilon = 1.0e-6);
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

#[test]
fn mixed_joint_mass_matrix_is_symmetric_and_positive_on_moving_coordinates() {
    let robot = mixed_oracle_arm();
    let mut workspace = robot.workspace();
    let zero = [0.0; 4];

    for sample in 0..16 {
        let q = [
            1.4 * ((sample + 1) as f64 * 0.71).sin(),
            3.0 * ((sample + 1) as f64 * 0.53).sin(),
            0.3 * ((sample + 1) as f64 * 0.37).sin(),
            2.5 * ((sample + 1) as f64 * 0.29).sin(),
        ];
        let mut bias = [0.0; 4];
        robot
            .inverse_dynamics(
                &q,
                &zero,
                &zero,
                &Frame::identity(),
                Twist::zeros(),
                Twist::zeros(),
                &[],
                &mut workspace,
                &mut bias,
            )
            .unwrap();

        let mut mass = DMatrix::<f64>::zeros(4, 4);
        for column in 0..4 {
            let mut unit_acceleration = [0.0; 4];
            unit_acceleration[column] = 1.0;
            let mut torque = [0.0; 4];
            robot
                .inverse_dynamics(
                    &q,
                    &zero,
                    &unit_acceleration,
                    &Frame::identity(),
                    Twist::zeros(),
                    Twist::zeros(),
                    &[],
                    &mut workspace,
                    &mut torque,
                )
                .unwrap();
            for row in 0..4 {
                mass[(row, column)] = torque[row] - bias[row];
            }
        }

        assert_relative_eq!(mass, mass.transpose(), epsilon = 2.0e-11);
        // Joint 1 is fixed and intentionally occupies a zero row and column.
        for direction in [
            [1.0, 0.0, -0.4, 0.7],
            [-0.3, 0.0, 1.2, -0.8],
            [0.6, 0.0, 0.2, 1.4],
        ] {
            let direction = nalgebra::DVector::from_column_slice(&direction);
            let energy = direction.dot(&(&mass * &direction));
            assert!(
                energy > 1.0e-8,
                "mass matrix is not positive on moving coordinates: sample={sample}, energy={energy}, mass={mass:?}"
            );
        }
    }
}
