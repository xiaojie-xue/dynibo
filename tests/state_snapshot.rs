use std::path::PathBuf;

use approx::assert_relative_eq;
use dynibo::{BaseState, Error, FloatingRobot, Frame, IndexedLoad, Robot, Twist, Wrench};
use nalgebra::{Translation3, UnitQuaternion, Vector3};

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

fn assert_frame_eq(actual: &Frame, expected: &Frame, eps: f64) {
    assert_relative_eq!(
        actual.translation.vector,
        expected.translation.vector,
        epsilon = eps
    );
    // Quaternions q and -q represent the same rotation.
    let dot = actual.rotation.coords.dot(&expected.rotation.coords);
    let aligned = if dot < 0.0 {
        -expected.rotation.coords
    } else {
        expected.rotation.coords
    };
    assert_relative_eq!(actual.rotation.coords, aligned, epsilon = eps);
}

fn assert_twist_eq(actual: Twist, expected: Twist, eps: f64) {
    assert_relative_eq!(actual.angular, expected.angular, epsilon = eps);
    assert_relative_eq!(actual.linear, expected.linear, epsilon = eps);
}

fn assert_slice_eq(actual: &[f64], expected: &[f64], eps: f64) {
    assert_eq!(actual.len(), expected.len());
    for (a, e) in actual.iter().zip(expected) {
        assert_relative_eq!(a, e, epsilon = eps);
    }
}

#[test]
fn fixed_robot_state_matches_standalone_methods_across_all_links_and_custom_settings() {
    let mut robot = Robot::from_urdf(fixture_path("test_arm.urdf")).unwrap();
    let base_frame = Frame::from_parts(
        Translation3::new(0.4, -0.25, 0.7),
        UnitQuaternion::from_euler_angles(0.2, -0.35, 0.5),
    );
    robot.set_base_frame(base_frame).unwrap();
    robot.set_gravity(Vector3::new(0.5, -1.2, -8.7)).unwrap();

    let mut reference_robot = robot.fork();
    let q = [0.25, -0.6, 0.9, -0.35];
    let qd = [-0.4, 0.7, -0.2, 0.55];
    let qdd = [0.6, -0.3, 0.25, -0.8];
    let tool = Frame::from_parts(
        Translation3::new(0.08, -0.04, 0.12),
        UnitQuaternion::from_euler_angles(-0.1, 0.2, 0.3),
    );
    let target = robot.link_id("test_link_4").unwrap();
    let loads = [IndexedLoad {
        link: target,
        wrench: Wrench::new(Vector3::new(0.2, -0.1, 0.3), Vector3::new(-0.5, 0.8, -1.2)),
    }];

    let n = robot.generalized_count();
    let link_count = robot.link_count();
    let mut ref_poses = vec![Frame::identity(); link_count];
    let mut state_poses = vec![Frame::identity(); link_count];
    let mut ref_j = vec![0.0; 6 * n];
    let mut state_j = vec![0.0; 6 * n];
    let mut ref_jd = vec![0.0; 6 * n];
    let mut state_jd = vec![0.0; 6 * n];
    let mut ref_m = vec![0.0; n * n];
    let mut state_m = vec![0.0; n * n];
    let mut ref_g = vec![0.0; n];
    let mut state_g = vec![0.0; n];
    let mut ref_c = vec![0.0; n];
    let mut state_c = vec![0.0; n];
    let mut ref_tau = vec![0.0; n];
    let mut state_tau = vec![0.0; n];
    let mut ref_aba = vec![0.0; n];
    let mut state_aba = vec![0.0; n];

    let mut state = robot.state(&q, &qd).unwrap();

    // Run dynamics BEFORE kinematics to verify scratch writes never clobber the cached snapshot.
    state.mass_matrix(&mut state_m).unwrap();
    state.gravity(&loads, &mut state_g).unwrap();
    state.velocity_product_forces(&mut state_c).unwrap();
    state
        .inverse_dynamics(&qdd, &loads, &mut state_tau)
        .unwrap();
    state
        .forward_dynamics(&state_tau, &loads, &mut state_aba)
        .unwrap();
    state.forward_kinematics_all(&mut state_poses).unwrap();

    reference_robot.mass_matrix(&q, &mut ref_m).unwrap();
    reference_robot.gravity(&q, &loads, &mut ref_g).unwrap();
    reference_robot
        .velocity_product_forces(&q, &qd, &mut ref_c)
        .unwrap();
    reference_robot
        .inverse_dynamics(&q, &qd, &qdd, &loads, &mut ref_tau)
        .unwrap();
    reference_robot
        .forward_dynamics(&q, &qd, &ref_tau, &loads, &mut ref_aba)
        .unwrap();
    reference_robot
        .forward_kinematics_all(&q, &mut ref_poses)
        .unwrap();

    assert_slice_eq(&state_m, &ref_m, 1e-12);
    assert_slice_eq(&state_g, &ref_g, 1e-12);
    assert_slice_eq(&state_c, &ref_c, 1e-12);
    assert_slice_eq(&state_tau, &ref_tau, 1e-12);
    assert_slice_eq(&state_aba, &ref_aba, 1e-12);
    assert_slice_eq(&state_aba, &qdd, 1e-10);
    for (a, e) in state_poses.iter().zip(&ref_poses) {
        assert_frame_eq(a, e, 1e-12);
    }

    for i in 0..link_count {
        let link = reference_robot.link_id_at(i).unwrap();
        assert_frame_eq(
            &state.forward_kinematics(link).unwrap(),
            &reference_robot.forward_kinematics(&q, link).unwrap(),
            1e-12,
        );
        assert_twist_eq(
            state.forward_velocity_kinematics(link, &tool).unwrap(),
            reference_robot
                .forward_velocity_kinematics(&q, &qd, link, &tool)
                .unwrap(),
            1e-12,
        );
        assert_twist_eq(
            state.forward_acceleration_kinematics(&qdd, link).unwrap(),
            reference_robot
                .forward_acceleration_kinematics(&q, &qd, &qdd, link)
                .unwrap(),
            1e-12,
        );
        state.jacobian(link, &mut state_j).unwrap();
        reference_robot.jacobian(&q, link, &mut ref_j).unwrap();
        assert_slice_eq(&state_j, &ref_j, 1e-12);

        state.jacobian_derivative(link, &mut state_jd).unwrap();
        reference_robot
            .jacobian_derivative(&q, &qd, link, &mut ref_jd)
            .unwrap();
        assert_slice_eq(&state_jd, &ref_jd, 1e-12);
    }

    // Re-running dynamics after kinematics must produce identical results.
    let mut state_tau_again = vec![0.0; n];
    state
        .inverse_dynamics(&qdd, &loads, &mut state_tau_again)
        .unwrap();
    assert_slice_eq(&state_tau_again, &ref_tau, 1e-12);
}

#[test]
fn floating_robot_state_matches_standalone_methods_with_base_motion_and_loads() {
    let mut robot = FloatingRobot::from_urdf(fixture_path("floating_arm.urdf")).unwrap();
    robot.set_gravity(Vector3::new(-0.3, 0.8, -9.5)).unwrap();
    let mut reference_robot = robot.fork();

    let base = BaseState::new(
        Frame::from_parts(
            Translation3::new(-0.5, 0.3, 1.1),
            UnitQuaternion::from_euler_angles(0.3, -0.2, 0.7),
        ),
        Twist::new(
            Vector3::new(0.15, -0.25, 0.35),
            Vector3::new(-0.4, 0.2, 0.6),
        ),
        Twist::new(Vector3::new(-0.1, 0.3, -0.2), Vector3::new(0.5, -0.7, 0.25)),
    )
    .unwrap();
    let q = [0.35, -0.45];
    let qd = [-0.6, 0.8];
    let qdd = [0.9, -0.5];
    let tool = Frame::from_parts(
        Translation3::new(0.05, 0.03, -0.07),
        UnitQuaternion::from_euler_angles(0.1, -0.15, 0.25),
    );
    let target = robot.link_id("tool").unwrap();
    let loads = [IndexedLoad {
        link: target,
        wrench: Wrench::new(Vector3::new(0.4, -0.2, 0.1), Vector3::new(1.5, -0.6, 0.9)),
    }];

    let g = robot.generalized_count();
    let link_count = robot.link_count();
    let mut ref_poses = vec![Frame::identity(); link_count];
    let mut state_poses = vec![Frame::identity(); link_count];
    let mut ref_j = vec![0.0; 6 * g];
    let mut state_j = vec![0.0; 6 * g];
    let mut ref_jd = vec![0.0; 6 * g];
    let mut state_jd = vec![0.0; 6 * g];
    let mut ref_m = vec![0.0; g * g];
    let mut state_m = vec![0.0; g * g];
    let mut ref_g = vec![0.0; g];
    let mut state_g = vec![0.0; g];
    let mut ref_c = vec![0.0; g];
    let mut state_c = vec![0.0; g];
    let mut ref_tau = vec![0.0; g];
    let mut state_tau = vec![0.0; g];
    let mut ref_aba = vec![0.0; g];
    let mut state_aba = vec![0.0; g];

    let mut state = robot.state(&base, &q, &qd).unwrap();
    state
        .forward_dynamics(&vec![0.1; g], &loads, &mut state_aba)
        .unwrap();
    state.mass_matrix(&mut state_m).unwrap();
    state.gravity(&loads, &mut state_g).unwrap();
    state.velocity_product_forces(&mut state_c).unwrap();
    state
        .inverse_dynamics(&qdd, &loads, &mut state_tau)
        .unwrap();
    state
        .forward_dynamics(&state_tau, &loads, &mut state_aba)
        .unwrap();
    state.forward_kinematics_all(&mut state_poses).unwrap();

    reference_robot.mass_matrix(&base, &q, &mut ref_m).unwrap();
    reference_robot
        .gravity(&base, &q, &loads, &mut ref_g)
        .unwrap();
    reference_robot
        .velocity_product_forces(&base, &q, &qd, &mut ref_c)
        .unwrap();
    reference_robot
        .inverse_dynamics(&base, &q, &qd, &qdd, &loads, &mut ref_tau)
        .unwrap();
    reference_robot
        .forward_dynamics(&base, &q, &qd, &ref_tau, &loads, &mut ref_aba)
        .unwrap();
    reference_robot
        .forward_kinematics_all(&base, &q, &mut ref_poses)
        .unwrap();

    assert_slice_eq(&state_m, &ref_m, 1e-12);
    assert_slice_eq(&state_g, &ref_g, 1e-12);
    assert_slice_eq(&state_c, &ref_c, 1e-12);
    assert_slice_eq(&state_tau, &ref_tau, 1e-12);
    assert_slice_eq(&state_aba, &ref_aba, 1e-11);
    for (a, e) in state_poses.iter().zip(&ref_poses) {
        assert_frame_eq(a, e, 1e-12);
    }

    for i in 0..link_count {
        let link = reference_robot.link_id_at(i).unwrap();
        assert_frame_eq(
            &state.forward_kinematics(link).unwrap(),
            &reference_robot.forward_kinematics(&base, &q, link).unwrap(),
            1e-12,
        );
        assert_twist_eq(
            state.forward_velocity_kinematics(link, &tool).unwrap(),
            reference_robot
                .forward_velocity_kinematics(&base, &q, &qd, link, &tool)
                .unwrap(),
            1e-12,
        );
        assert_twist_eq(
            state.forward_acceleration_kinematics(&qdd, link).unwrap(),
            reference_robot
                .forward_acceleration_kinematics(&base, &q, &qd, &qdd, link)
                .unwrap(),
            1e-12,
        );
        state.jacobian(link, &mut state_j).unwrap();
        reference_robot
            .jacobian(&base, &q, link, &mut ref_j)
            .unwrap();
        assert_slice_eq(&state_j, &ref_j, 1e-12);

        state.jacobian_derivative(link, &mut state_jd).unwrap();
        reference_robot
            .jacobian_derivative(&base, &q, &qd, link, &mut ref_jd)
            .unwrap();
        assert_slice_eq(&state_jd, &ref_jd, 1e-12);
    }
}

#[test]
fn state_validates_inputs() {
    let mut robot = Robot::from_urdf(fixture_path("test_arm.urdf")).unwrap();
    assert!(matches!(
        robot.state(&[0.0; 3], &[0.0; 4]),
        Err(Error::WrongSliceLength { slice: "q", .. })
    ));
    assert!(matches!(
        robot.state(&[0.0; 4], &[f64::NAN, 0.0, 0.0, 0.0]),
        Err(Error::NonFiniteInput { input: "qd" })
    ));

    let target = robot.link_id("test_link_4").unwrap();
    let state = robot.state(&[0.0; 4], &[0.0; 4]).unwrap();
    let bad_tool = Frame::from_parts(
        Translation3::new(f64::NAN, 0.0, 0.0),
        UnitQuaternion::identity(),
    );
    assert!(matches!(
        state.forward_velocity_kinematics(target, &bad_tool),
        Err(Error::NonFiniteInput {
            input: "tool frame"
        })
    ));
    assert!(matches!(
        state.forward_kinematics_all(&mut []),
        Err(Error::WrongSliceLength { .. })
    ));
    assert!(matches!(
        state.jacobian(target, &mut [0.0; 1]),
        Err(Error::WrongSliceLength { .. })
    ));
    assert!(matches!(
        state.jacobian_derivative(target, &mut [0.0; 1]),
        Err(Error::WrongSliceLength { .. })
    ));
}

#[test]
fn state_matches_standalone_on_mixed_revolute_prismatic_and_fixed_joints() {
    let mut robot = Robot::from_urdf(fixture_path("oracle_mixed.urdf")).unwrap();
    let mut reference_robot = robot.fork();
    let n = robot.joint_count();
    let g = robot.generalized_count();
    let q: Vec<f64> = (0..n).map(|i| 0.15 * (i as f64 + 1.0)).collect();
    let qd: Vec<f64> = (0..n).map(|i| -0.2 + 0.12 * (i as f64)).collect();
    let qdd: Vec<f64> = (0..n).map(|i| 0.3 - 0.1 * (i as f64)).collect();
    let tool = Frame::from_parts(
        Translation3::new(0.05, -0.02, 0.04),
        UnitQuaternion::identity(),
    );

    let mut state = robot.state(&q, &qd).unwrap();
    let mut ref_j = vec![0.0; 6 * g];
    let mut state_j = vec![0.0; 6 * g];
    let mut ref_jd = vec![0.0; 6 * g];
    let mut state_jd = vec![0.0; 6 * g];
    let mut ref_m = vec![0.0; g * g];
    let mut state_m = vec![0.0; g * g];
    let mut ref_tau = vec![0.0; g];
    let mut state_tau = vec![0.0; g];
    state.mass_matrix(&mut state_m).unwrap();
    reference_robot.mass_matrix(&q, &mut ref_m).unwrap();
    assert_slice_eq(&state_m, &ref_m, 1e-12);
    state.inverse_dynamics(&qdd, &[], &mut state_tau).unwrap();
    reference_robot
        .inverse_dynamics(&q, &qd, &qdd, &[], &mut ref_tau)
        .unwrap();
    assert_slice_eq(&state_tau, &ref_tau, 1e-12);

    for i in 0..reference_robot.link_count() {
        let link = reference_robot.link_id_at(i).unwrap();
        assert_frame_eq(
            &state.forward_kinematics(link).unwrap(),
            &reference_robot.forward_kinematics(&q, link).unwrap(),
            1e-12,
        );
        assert_twist_eq(
            state.forward_velocity_kinematics(link, &tool).unwrap(),
            reference_robot
                .forward_velocity_kinematics(&q, &qd, link, &tool)
                .unwrap(),
            1e-12,
        );
        assert_twist_eq(
            state.forward_acceleration_kinematics(&qdd, link).unwrap(),
            reference_robot
                .forward_acceleration_kinematics(&q, &qd, &qdd, link)
                .unwrap(),
            1e-12,
        );
        state.jacobian(link, &mut state_j).unwrap();
        reference_robot.jacobian(&q, link, &mut ref_j).unwrap();
        assert_slice_eq(&state_j, &ref_j, 1e-12);
        state.jacobian_derivative(link, &mut state_jd).unwrap();
        reference_robot
            .jacobian_derivative(&q, &qd, link, &mut ref_jd)
            .unwrap();
        assert_slice_eq(&state_jd, &ref_jd, 1e-12);
    }

    // Overflow in velocity/acceleration kinematics must be reported as NumericalFailure.
    let target = reference_robot.link_id("tool").unwrap();
    let huge_base = Frame::from_parts(
        Translation3::new(1.0e308, 1.0e308, 1.0e308),
        UnitQuaternion::from_euler_angles(0.5, 0.5, 0.5),
    );
    robot.set_base_frame(huge_base).unwrap();
    let huge_q = vec![0.0, 1.0e308, 0.0];
    let huge_qd = vec![1.0e308; n];
    let mut overflow_state = robot.state(&huge_q, &huge_qd).unwrap();
    assert!(matches!(
        overflow_state.forward_kinematics(target),
        Err(Error::NumericalFailure { .. })
    ));
    assert!(matches!(
        overflow_state.forward_velocity_kinematics(target, &tool),
        Err(Error::NumericalFailure { .. })
    ));
    assert!(matches!(
        overflow_state.forward_acceleration_kinematics(&qdd, target),
        Err(Error::NumericalFailure { .. })
    ));
}
