use std::path::PathBuf;

use approx::assert_relative_eq;
use dynibo::{
    BaseMode, BaseState, Error, Frame, IndexedLoad, InverseKinematicsOptions, Robot, Twist, Wrench,
};
use nalgebra::{DMatrix, DVector, Translation3, UnitQuaternion, Vector3};

fn robot() -> Robot {
    Robot::from_urdf_with_base(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/floating_arm.urdf"),
        BaseMode::Floating,
    )
    .unwrap()
}

fn base_frame() -> Frame {
    Frame::from_parts(
        Translation3::new(0.4, -0.3, 0.7),
        UnitQuaternion::from_euler_angles(0.3, -0.25, 0.17),
    )
}

fn base_velocity() -> Twist {
    Twist::new(
        Vector3::new(0.21, -0.17, 0.13),
        Vector3::new(-0.3, 0.2, 0.1),
    )
}

fn base_acceleration() -> Twist {
    Twist::new(
        Vector3::new(-0.11, 0.14, 0.09),
        Vector3::new(0.35, -0.22, 0.18),
    )
}

fn assert_slice_close(actual: &[f64], expected: &[f64], tolerance: f64) {
    assert_eq!(actual.len(), expected.len());
    for (index, (&actual, &expected)) in actual.iter().zip(expected).enumerate() {
        assert!(
            (actual - expected).abs() <= tolerance,
            "element {index}: actual={actual:.16e}, expected={expected:.16e}"
        );
    }
}

#[test]
fn base_mode_state_dimensions_and_ik_contract_are_explicit() {
    let fixed = Robot::from_urdf(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/floating_arm.urdf"),
    )
    .unwrap();
    assert_eq!(fixed.base_mode(), BaseMode::Fixed);
    assert_eq!(fixed.base_dof_count(), 0);
    assert_eq!(fixed.generalized_count(), fixed.joint_count());
    let zero = BaseState::fixed();
    assert_eq!(zero.frame(), &Frame::identity());
    assert_eq!(zero.velocity(), Twist::zeros());
    assert_eq!(zero.acceleration(), Twist::zeros());
    assert!(matches!(
        BaseState::fixed_at(Frame::translation(f64::NAN, 0.0, 0.0)),
        Err(Error::InvalidBaseState { field: "frame", .. })
    ));
    let fixed_state = BaseState::fixed_at(base_frame()).unwrap();
    let fixed_target = fixed.link_id("tool").unwrap();
    let mut fixed_workspace = fixed.workspace();
    let known_q = [0.2, 0.1];
    let moving_state = BaseState::new(
        Frame::identity(),
        Twist::new(Vector3::x(), Vector3::zeros()),
        Twist::zeros(),
    )
    .unwrap();
    assert!(matches!(
        fixed.forward_kinematics(&moving_state, &known_q, fixed_target, &mut fixed_workspace,),
        Err(Error::InvalidBaseState {
            field: "velocity",
            ..
        })
    ));
    let desired = fixed
        .forward_kinematics(&fixed_state, &known_q, fixed_target, &mut fixed_workspace)
        .unwrap();
    let mut solved = [0.0; 2];
    fixed
        .inverse_kinematics(
            &fixed_state,
            &[0.0; 2],
            fixed_target,
            &desired,
            InverseKinematicsOptions::default(),
            &mut fixed_workspace,
            &mut solved,
        )
        .unwrap();
    assert_slice_close(&solved, &known_q, 2.0e-5);

    let floating = robot();
    assert_eq!(floating.base_dof_count(), 6);
    assert_eq!(floating.generalized_count(), floating.joint_count() + 6);
    let floating_state =
        BaseState::new(base_frame(), base_velocity(), base_acceleration()).unwrap();
    assert_eq!(floating_state.frame(), &base_frame());
    assert_eq!(floating_state.velocity(), base_velocity());
    assert_eq!(floating_state.acceleration(), base_acceleration());
    assert!(matches!(
        BaseState::new(
            base_frame(),
            Twist::new(Vector3::new(f64::NAN, 0.0, 0.0), Vector3::zeros()),
            Twist::zeros(),
        ),
        Err(Error::InvalidBaseState {
            field: "velocity",
            ..
        })
    ));
    assert!(matches!(
        BaseState::new(
            base_frame(),
            Twist::zeros(),
            Twist::new(Vector3::zeros(), Vector3::new(0.0, f64::INFINITY, 0.0),),
        ),
        Err(Error::InvalidBaseState {
            field: "acceleration",
            ..
        })
    ));
    let mut workspace = floating.workspace();
    assert_eq!(workspace.generalized_count(), 8);
    let target = floating.link_id("tool").unwrap();
    let mut short_jacobian = [0.0; 47];
    assert!(matches!(
        floating.jacobian(
            &floating_state,
            &[0.0; 2],
            target,
            &mut workspace,
            &mut short_jacobian,
        ),
        Err(Error::WrongSliceLength {
            slice: "jacobian output",
            expected: 48,
            actual: 47,
        })
    ));
    let mut output = [0.0; 2];
    assert!(matches!(
        floating.inverse_kinematics(
            &floating_state,
            &[0.0; 2],
            target,
            &Frame::identity(),
            InverseKinematicsOptions::default(),
            &mut workspace,
            &mut output,
        ),
        Err(Error::FloatingBaseIkUnsupported)
    ));
}

#[test]
fn generalized_jacobian_and_derivative_match_forward_motion() {
    let robot = robot();
    let base = BaseState::new(base_frame(), base_velocity(), base_acceleration()).unwrap();
    let target = robot.link_id("tool").unwrap();
    let mut workspace = robot.workspace();
    let q = [0.37, 0.18];
    let qd = [-0.42, 0.31];
    let qdd = [0.27, -0.19];
    let generalized_velocity =
        DVector::from_column_slice(&[0.21, -0.17, 0.13, -0.3, 0.2, 0.1, qd[0], qd[1]]);
    let generalized_acceleration =
        DVector::from_column_slice(&[-0.11, 0.14, 0.09, 0.35, -0.22, 0.18, qdd[0], qdd[1]]);
    let mut jacobian = vec![0.0; 6 * robot.generalized_count()];
    let mut derivative = vec![0.0; jacobian.len()];
    robot
        .jacobian(&base, &q, target, &mut workspace, &mut jacobian)
        .unwrap();
    robot
        .jacobian_derivative(&base, &q, &qd, target, &mut workspace, &mut derivative)
        .unwrap();
    let jacobian = DMatrix::from_column_slice(6, robot.generalized_count(), &jacobian);
    let derivative = DMatrix::from_column_slice(6, robot.generalized_count(), &derivative);
    let velocity = robot
        .forward_velocity_kinematics(&base, &q, &qd, target, &Frame::identity(), &mut workspace)
        .unwrap();
    assert_slice_close(
        velocity.to_vector().as_slice(),
        (jacobian.clone() * &generalized_velocity).as_slice(),
        2.0e-11,
    );
    let acceleration = robot
        .forward_acceleration_kinematics(&base, &q, &qd, &qdd, target, &mut workspace)
        .unwrap();
    let expected = jacobian * generalized_acceleration + derivative * generalized_velocity;
    assert_slice_close(
        acceleration.to_vector().as_slice(),
        expected.as_slice(),
        3.0e-10,
    );
}

#[test]
fn floating_mass_velocity_product_gravity_and_rnea_obey_manipulator_identities() {
    let robot = robot();
    let moving_base = BaseState::new(base_frame(), base_velocity(), Twist::zeros()).unwrap();
    let mut workspace = robot.workspace();
    let q = [0.37, 0.18];
    let qd = [-0.42, 0.31];
    let n = robot.generalized_count();
    let mut mass = vec![0.0; n * n];
    robot
        .mass_matrix(&moving_base, &q, &mut workspace, &mut mass)
        .unwrap();
    let mass_matrix = DMatrix::from_column_slice(n, n, &mass);
    assert_relative_eq!(
        mass_matrix.clone(),
        mass_matrix.transpose(),
        epsilon = 2.0e-11
    );

    let stationary_base = BaseState::new(base_frame(), Twist::zeros(), Twist::zeros()).unwrap();
    let mut gravity = vec![0.0; n];
    robot
        .gravity(&stationary_base, &q, &[], &mut workspace, &mut gravity)
        .unwrap();
    for column in 0..n {
        let mut qdd = [0.0; 2];
        let mut base_acceleration = Twist::zeros();
        if column < 3 {
            base_acceleration.angular[column] = 1.0;
        } else if column < 6 {
            base_acceleration.linear[column - 3] = 1.0;
        } else {
            qdd[column - 6] = 1.0;
        }
        let accelerated_base =
            BaseState::new(base_frame(), Twist::zeros(), base_acceleration).unwrap();
        let mut inverse = vec![0.0; n];
        robot
            .inverse_dynamics(
                &accelerated_base,
                &q,
                &[0.0; 2],
                &qdd,
                &[],
                &mut workspace,
                &mut inverse,
            )
            .unwrap();
        for row in 0..n {
            assert_relative_eq!(
                inverse[row] - gravity[row],
                mass_matrix[(row, column)],
                epsilon = 3.0e-10
            );
        }
    }

    let mut velocity_product = vec![0.0; n];
    robot
        .velocity_product_forces(&moving_base, &q, &qd, &mut workspace, &mut velocity_product)
        .unwrap();
    let expected_bias =
        DVector::from_column_slice(&velocity_product) + DVector::from_column_slice(&gravity);
    let mut inverse = vec![0.0; n];
    robot
        .inverse_dynamics(
            &moving_base,
            &q,
            &qd,
            &[0.0; 2],
            &[],
            &mut workspace,
            &mut inverse,
        )
        .unwrap();
    assert_slice_close(&inverse, expected_bias.as_slice(), 3.0e-9);
}

#[test]
fn root_load_is_returned_as_a_world_base_wrench() {
    let robot = robot();
    let base = BaseState::new(base_frame(), Twist::zeros(), Twist::zeros()).unwrap();
    let root = robot.link_id("base").unwrap();
    let load = Wrench::new(Vector3::new(0.3, -0.2, 0.4), Vector3::new(1.0, 0.5, -0.7));
    let mut workspace = robot.workspace();
    let mut baseline = vec![0.0; robot.generalized_count()];
    let mut loaded = baseline.clone();
    robot
        .gravity(&base, &[0.2, 0.1], &[], &mut workspace, &mut baseline)
        .unwrap();
    robot
        .gravity(
            &base,
            &[0.2, 0.1],
            &[IndexedLoad {
                link: root,
                wrench: load,
            }],
            &mut workspace,
            &mut loaded,
        )
        .unwrap();
    let expected_torque = base_frame().rotation * load.torque;
    let expected_force = base_frame().rotation * load.force;
    let expected_torque = std::array::from_fn::<_, 3, _>(|i| baseline[i] + expected_torque[i]);
    let expected_force = std::array::from_fn::<_, 3, _>(|i| baseline[i + 3] + expected_force[i]);
    assert_slice_close(&loaded[..3], &expected_torque, 2.0e-12);
    assert_slice_close(&loaded[3..6], &expected_force, 2.0e-12);
    assert_slice_close(&loaded[6..], &baseline[6..], 2.0e-12);
}
