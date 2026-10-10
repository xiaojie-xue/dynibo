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
