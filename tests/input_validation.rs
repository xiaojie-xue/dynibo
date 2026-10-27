use dynibo::{
    BaseState, Error, ErrorCategory, FloatingRobot, Frame, IndexedLoad, InverseKinematicsOptions,
    Robot, Wrench,
};
use nalgebra::Vector3;
use std::path::PathBuf;

fn path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf")
}

#[test]
fn invalid_loads_forces_and_tools_preserve_outputs_and_allow_reuse() {
    let mut fixed = Robot::from_urdf(path()).unwrap();
    let mut floating = FloatingRobot::from_urdf(path()).unwrap();
    let base = BaseState::stationary(Frame::identity()).unwrap();
    let q = vec![0.0; fixed.joint_count()];
    for is_floating in [false, true] {
        let g = if is_floating {
            floating.generalized_count()
        } else {
            fixed.generalized_count()
        };
        let id = if is_floating {
            floating.link_id_at(1)
        } else {
            fixed.link_id_at(1)
        }
        .unwrap();
        let mut output = vec![123.0; g];
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for component in 0..6 {
                let mut wrench = Wrench::zeros();
                if component < 3 {
                    wrench.torque[component] = invalid;
                } else {
                    wrench.force[component - 3] = invalid;
                }
                let loads = [IndexedLoad { link: id, wrench }];
                for operation in 0..3 {
                    output.fill(123.0);
                    let forces = vec![0.0; g];
                    let result = match (is_floating, operation) {
                        (false, 0) => fixed.gravity(&q, &loads, &mut output),
                        (false, 1) => fixed.inverse_dynamics(&q, &q, &q, &loads, &mut output),
                        (false, _) => fixed.forward_dynamics(&q, &q, &forces, &loads, &mut output),
                        (true, 0) => floating.gravity(&base, &q, &loads, &mut output),
                        (true, 1) => {
                            floating.inverse_dynamics(&base, &q, &q, &q, &loads, &mut output)
                        }
                        (true, _) => {
                            floating.forward_dynamics(&base, &q, &q, &forces, &loads, &mut output)
                        }
                    };
                    assert!(matches!(
                        result,
                        Err(Error::NonFiniteInput { input: "load" })
                    ));
                    assert!(output.iter().all(|v| *v == 123.0));
                }
            }
            let forces = vec![invalid; g];
            let result = if is_floating {
                floating.forward_dynamics(&base, &q, &q, &forces, &[], &mut output)
            } else {
                fixed.forward_dynamics(&q, &q, &forces, &[], &mut output)
            };
            assert_eq!(result.unwrap_err().category(), ErrorCategory::InvalidInput);
            assert!(output.iter().all(|v| *v == 123.0));
            let tool = Frame::translation(invalid, 0.0, 0.0);
            let result = if is_floating {
                floating.forward_velocity_kinematics(&base, &q, &q, id, &tool)
            } else {
                fixed.forward_velocity_kinematics(&q, &q, id, &tool)
            };
            assert!(matches!(
                result,
                Err(Error::NonFiniteInput {
                    input: "tool frame"
                })
            ));
        }
        let huge = IndexedLoad {
            link: id,
            wrench: Wrench::new(Vector3::repeat(f64::MAX), Vector3::zeros()),
        };
        let result = if is_floating {
            floating.gravity(&base, &q, &[huge, huge], &mut output)
        } else {
            fixed.gravity(&q, &[huge, huge], &mut output)
        };
        assert!(matches!(
            result,
            Err(Error::NumericalFailure {
                operation: "load aggregation"
            })
        ));
        assert!(output.iter().all(|v| *v == 123.0));
        let mut expected = vec![0.0; g];
        if is_floating {
            floating
                .fork()
                .gravity(&base, &q, &[], &mut expected)
                .unwrap();
            floating.gravity(&base, &q, &[], &mut output).unwrap();
        } else {
            fixed.fork().gravity(&q, &[], &mut expected).unwrap();
            fixed.gravity(&q, &[], &mut output).unwrap();
        }
        assert_eq!(output, expected);
    }
}

#[test]
fn ik_limit_error_reports_an_active_dof_index_after_fixed_joints() {
    let mut robot = Robot::from_urdf(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf"),
    )
    .unwrap();
    // Select a DOF after a fixed joint, and use its current FK pose so the
    // solver reaches the limit check without needing an iterative solve.
    for dof in 0..robot.joint_count() {
        let mut q = vec![0.0; robot.joint_count()];
        q[dof] = robot.joint_upper_limit(dof).unwrap() + 1.0;
        if !q[dof].is_finite() {
            continue;
        }
        let target = robot.root_link_id();
        let desired = robot.forward_kinematics(&q, target).unwrap();
        let mut output = vec![123.0; q.len()];
        let error = robot
            .inverse_kinematics(
                &q,
                target,
                &desired,
                InverseKinematicsOptions::default(),
                &mut output,
            )
            .unwrap_err();
        match error {
            Error::IkJointLimitViolation {
                joint_index, joint, ..
            } => {
                assert_eq!(joint_index, dof);
                assert_eq!(joint, robot.joint_name(dof).unwrap());
            }
            other => panic!("unexpected error: {other}"),
        }
        assert!(output.iter().all(|v| *v == 123.0));
    }
}

#[test]
fn finite_velocity_overflow_is_a_solver_error_and_workspace_recovers() {
    let mut fixed = Robot::from_urdf(path()).unwrap();
    let mut floating = FloatingRobot::from_urdf(path()).unwrap();
    let base = BaseState::stationary(Frame::identity()).unwrap();
    let q = vec![0.0; fixed.joint_count()];
    let huge_velocity = vec![1.0e308; q.len()];
    for is_floating in [false, true] {
        let g = if is_floating {
            floating.generalized_count()
        } else {
            fixed.generalized_count()
        };
        let target = if is_floating {
            floating.link_id("test_link_4")
        } else {
            fixed.link_id("test_link_4")
        }
        .unwrap();
        let mut output = vec![0.0; g];
        let forces = vec![0.0; g];
        for operation in ["velocity product", "inverse dynamics", "forward dynamics"] {
            let calculate = |fixed: &mut Robot,
                             floating: &mut FloatingRobot,
                             velocity: &[f64],
                             output: &mut [f64]| match (
                is_floating,
                operation,
            ) {
                (false, "velocity product") => fixed.velocity_product_forces(&q, velocity, output),
                (false, "inverse dynamics") => {
                    fixed.inverse_dynamics(&q, velocity, &q, &[], output)
                }
                (false, _) => fixed.forward_dynamics(&q, velocity, &forces, &[], output),
                (true, "velocity product") => {
                    floating.velocity_product_forces(&base, &q, velocity, output)
                }
                (true, "inverse dynamics") => {
                    floating.inverse_dynamics(&base, &q, velocity, &q, &[], output)
                }
                (true, _) => floating.forward_dynamics(&base, &q, velocity, &forces, &[], output),
            };
            let result = calculate(&mut fixed, &mut floating, &huge_velocity, &mut output);
            assert!(
                matches!(result, Err(Error::NumericalFailure { .. })),
                "floating={is_floating} {operation}: {result:?}, output={output:?}"
            );
            let mut expected = vec![0.0; g];
            calculate(&mut fixed.fork(), &mut floating.fork(), &q, &mut expected).unwrap();
            calculate(&mut fixed, &mut floating, &q, &mut output).unwrap();
            assert_eq!(output, expected);
        }
        let result = if is_floating {
            floating.forward_acceleration_kinematics(&base, &q, &huge_velocity, &q, target)
        } else {
            fixed.forward_acceleration_kinematics(&q, &huge_velocity, &q, target)
        };
        assert!(
            matches!(result, Err(Error::NumericalFailure { .. })),
            "{result:?}"
        );
        let recovered = if is_floating {
            floating.forward_acceleration_kinematics(&base, &q, &q, &q, target)
        } else {
            fixed.forward_acceleration_kinematics(&q, &q, &q, target)
        }
        .unwrap();
        assert_eq!(recovered, dynibo::Twist::zeros());
    }
}

#[test]
fn finite_tool_velocity_overflow_is_a_solver_error() {
    let mut fixed = Robot::from_urdf(path()).unwrap();
    let mut floating = FloatingRobot::from_urdf(path()).unwrap();
    let base = BaseState::stationary(Frame::identity()).unwrap();
    let q = vec![0.0; fixed.joint_count()];
    let qd = vec![100.0; q.len()];
    let tool = Frame::translation(1.0e308, 1.0e308, 1.0e308);
    for is_floating in [false, true] {
        let target = if is_floating {
            floating.link_id("test_link_4")
        } else {
            fixed.link_id("test_link_4")
        }
        .unwrap();
        let result = if is_floating {
            floating.forward_velocity_kinematics(&base, &q, &qd, target, &tool)
        } else {
            fixed.forward_velocity_kinematics(&q, &qd, target, &tool)
        };
        assert!(
            matches!(result, Err(Error::NumericalFailure { .. })),
            "{result:?}"
        );
        let recovered = if is_floating {
            floating.forward_velocity_kinematics(&base, &q, &q, target, &Frame::identity())
        } else {
            fixed.forward_velocity_kinematics(&q, &q, target, &Frame::identity())
        }
        .unwrap();
        assert_eq!(recovered, dynibo::Twist::zeros());
    }
}

#[test]
fn finite_prismatic_configuration_overflow_is_a_solver_error() {
    let model = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/floating_arm.urdf");
    let mut fixed = Robot::from_urdf(&model).unwrap();
    let mut floating = FloatingRobot::from_urdf(&model).unwrap();
    let base_frame = Frame::translation(1.0e308, 1.0e308, 1.0e308);
    fixed.set_base_frame(base_frame).unwrap();
    let base = BaseState::stationary(base_frame).unwrap();
    let huge_q = [0.0, f64::MAX];
    let normal_q = [0.0, 0.0];
    for is_floating in [false, true] {
        let (target, g, links) = if is_floating {
            (
                floating.link_id("tool").unwrap(),
                floating.generalized_count(),
                floating.link_count(),
            )
        } else {
            (
                fixed.link_id("tool").unwrap(),
                fixed.generalized_count(),
                fixed.link_count(),
            )
        };
        let result = if is_floating {
            floating.forward_kinematics(&base, &huge_q, target)
        } else {
            fixed.forward_kinematics(&huge_q, target)
        };
        assert!(
            matches!(result, Err(Error::NumericalFailure { .. })),
            "{result:?}"
        );
        let mut poses = vec![Frame::identity(); links];
        let result = if is_floating {
            floating.forward_kinematics_all(&base, &huge_q, &mut poses)
        } else {
            fixed.forward_kinematics_all(&huge_q, &mut poses)
        };
        assert!(
            matches!(result, Err(Error::NumericalFailure { .. })),
            "{result:?}"
        );
        let mut matrix = vec![0.0; g * g];
        let result = if is_floating {
            floating.mass_matrix(&base, &huge_q, &mut matrix)
        } else {
            fixed.mass_matrix(&huge_q, &mut matrix)
        };
        assert!(
            matches!(result, Err(Error::NumericalFailure { .. })),
            "{result:?}, {matrix:?}"
        );
        if is_floating {
            floating
                .forward_kinematics_all(&base, &normal_q, &mut poses)
                .unwrap();
            floating.mass_matrix(&base, &normal_q, &mut matrix).unwrap();
        } else {
            fixed.forward_kinematics_all(&normal_q, &mut poses).unwrap();
            fixed.mass_matrix(&normal_q, &mut matrix).unwrap();
        }
        assert!(matrix.iter().all(|value| value.is_finite()));
    }
}

#[test]
fn finite_jacobian_derivative_and_load_overflow_are_solver_errors() {
    let mut fixed = Robot::from_urdf(path()).unwrap();
    let mut floating = FloatingRobot::from_urdf(path()).unwrap();
    let base = BaseState::stationary(Frame::identity()).unwrap();
    let q = vec![0.0; fixed.joint_count()];
    let qd = vec![1.0e308; q.len()];
    for is_floating in [false, true] {
        let (target, g) = if is_floating {
            (
                floating.link_id("test_link_4").unwrap(),
                floating.generalized_count(),
            )
        } else {
            (
                fixed.link_id("test_link_4").unwrap(),
                fixed.generalized_count(),
            )
        };
        let mut derivative = vec![0.0; 6 * g];
        let result = if is_floating {
            floating.jacobian_derivative(&base, &q, &qd, target, &mut derivative)
        } else {
            fixed.jacobian_derivative(&q, &qd, target, &mut derivative)
        };
        assert!(
            matches!(result, Err(Error::NumericalFailure { .. })),
            "{result:?}, {derivative:?}"
        );
        let mut expected = vec![0.0; 6 * g];
        if is_floating {
            floating
                .fork()
                .jacobian_derivative(&base, &q, &q, target, &mut expected)
                .unwrap();
            floating
                .jacobian_derivative(&base, &q, &q, target, &mut derivative)
                .unwrap();
        } else {
            fixed
                .fork()
                .jacobian_derivative(&q, &q, target, &mut expected)
                .unwrap();
            fixed
                .jacobian_derivative(&q, &q, target, &mut derivative)
                .unwrap();
        }
        assert_eq!(derivative, expected);
        let loads = [IndexedLoad {
            link: target,
            wrench: Wrench::new(Vector3::repeat(f64::MAX), Vector3::repeat(f64::MAX)),
        }];
        let mut output = vec![0.0; g];
        let result = if is_floating {
            floating.gravity(&base, &q, &loads, &mut output)
        } else {
            fixed.gravity(&q, &loads, &mut output)
        };
        assert!(
            matches!(result, Err(Error::NumericalFailure { .. })),
            "{result:?}, {output:?}"
        );
        let mut expected = vec![0.0; g];
        if is_floating {
            floating
                .fork()
                .gravity(&base, &q, &[], &mut expected)
                .unwrap();
            floating.gravity(&base, &q, &[], &mut output).unwrap();
        } else {
            fixed.fork().gravity(&q, &[], &mut expected).unwrap();
            fixed.gravity(&q, &[], &mut output).unwrap();
        }
        assert_eq!(output, expected);
    }
}
