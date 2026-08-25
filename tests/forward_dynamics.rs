use std::path::PathBuf;

use approx::assert_relative_eq;
use dynibo::{BaseMode, BaseState, Error, Frame, IndexedLoad, Robot, Twist, Wrench};
use nalgebra::{Translation3, UnitQuaternion, Vector3};

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

fn assert_close(actual: &[f64], expected: &[f64], epsilon: f64) {
    assert_eq!(actual.len(), expected.len());
    for (&actual, &expected) in actual.iter().zip(expected) {
        assert_relative_eq!(actual, expected, epsilon = epsilon, max_relative = epsilon);
    }
}

#[test]
fn fixed_forward_dynamics_inverts_inverse_dynamics_for_tree_and_mixed_joints() {
    for (model, target_name) in [
        ("test_tree_7.urdf", "right_tool"),
        ("oracle_mixed.urdf", "tool"),
    ] {
        let mut robot = Robot::from_urdf(fixture(model)).unwrap();
        let joint_count = robot.joint_count();
        let load = IndexedLoad {
            link: robot.link_id(target_name).unwrap(),
            wrench: Wrench::new(Vector3::new(0.2, -0.1, 0.3), Vector3::new(-0.7, 0.4, -0.2)),
        };

        for sample in 0..24 {
            let state = |phase: f64, amplitude: f64| -> Vec<f64> {
                (0..joint_count)
                    .map(|joint| {
                        amplitude * ((sample + 1) as f64 * (joint + 2) as f64 * 0.37 + phase).sin()
                    })
                    .collect()
            };
            let q = state(0.1, 0.6);
            let qd = state(0.7, 0.8);
            let expected_qdd = state(1.1, 0.9);
            let mut generalized_forces = vec![0.0; joint_count];
            robot
                .inverse_dynamics(
                    &BaseState::fixed(),
                    &q,
                    &qd,
                    &expected_qdd,
                    &[load],
                    &mut generalized_forces,
                )
                .unwrap();

            let mut actual_qdd = vec![f64::NAN; joint_count];
            robot
                .forward_dynamics(
                    &BaseState::fixed(),
                    &q,
                    &qd,
                    &generalized_forces,
                    &[load],
                    &mut actual_qdd,
                )
                .unwrap();
            assert_close(&actual_qdd, &expected_qdd, 2.0e-10);
        }
    }
}

#[test]
fn floating_forward_dynamics_inverts_moving_base_inverse_dynamics() {
    let mut robot =
        Robot::from_urdf_with_base(fixture("floating_arm.urdf"), BaseMode::Floating).unwrap();
    let load = IndexedLoad {
        link: robot.link_id("tool").unwrap(),
        wrench: Wrench::new(
            Vector3::new(-0.13, 0.21, 0.08),
            Vector3::new(0.5, -0.4, 0.3),
        ),
    };
    let q = [0.31, -0.27];
    let qd = [-0.24, 0.35];
    let qdd = [0.42, -0.28];
    let frame = Frame::from_parts(
        Translation3::new(0.2, -0.3, 0.5),
        UnitQuaternion::from_euler_angles(0.27, -0.19, 0.31),
    );
    let velocity = Twist::new(
        Vector3::new(0.23, -0.17, 0.11),
        Vector3::new(-0.32, 0.26, 0.14),
    );
    let expected_base_acceleration = Twist::new(
        Vector3::new(-0.12, 0.15, 0.09),
        Vector3::new(0.38, -0.25, 0.17),
    );
    let inverse_base = BaseState::new(frame, velocity, expected_base_acceleration).unwrap();
    let mut generalized_forces = vec![0.0; robot.generalized_count()];
    robot
        .inverse_dynamics(
            &inverse_base,
            &q,
            &qd,
            &qdd,
            &[load],
            &mut generalized_forces,
        )
        .unwrap();

    // The acceleration component is deliberately different: forward dynamics
    // consumes only the floating-base pose and velocity.
    let forward_base = BaseState::new(
        frame,
        velocity,
        Twist::new(Vector3::repeat(4.0), Vector3::repeat(-3.0)),
    )
    .unwrap();
    let mut actual = vec![f64::NAN; robot.generalized_count()];
    robot
        .forward_dynamics(
            &forward_base,
            &q,
            &qd,
            &generalized_forces,
            &[load],
            &mut actual,
        )
        .unwrap();

    let expected = [
        expected_base_acceleration.angular.x,
        expected_base_acceleration.angular.y,
        expected_base_acceleration.angular.z,
        expected_base_acceleration.linear.x,
        expected_base_acceleration.linear.y,
        expected_base_acceleration.linear.z,
        qdd[0],
        qdd[1],
    ];
    assert_close(&actual, &expected, 3.0e-10);
}

#[test]
fn forward_dynamics_validates_dimensions_base_state_and_load_ids() {
    let mut robot = Robot::from_urdf(fixture("floating_arm.urdf")).unwrap();
    let other = Robot::from_urdf(fixture("floating_arm.urdf")).unwrap();
    let foreign_load = IndexedLoad {
        link: other.link_id("tool").unwrap(),
        wrench: Wrench::zeros(),
    };
    let q = [0.0; 2];
    let mut output = [0.0; 2];

    assert!(matches!(
        robot.forward_dynamics(&BaseState::fixed(), &q[..1], &q, &q, &[], &mut output),
        Err(Error::WrongSliceLength { slice: "q", .. })
    ));
    assert!(matches!(
        robot.forward_dynamics(&BaseState::fixed(), &q, &q[..1], &q, &[], &mut output),
        Err(Error::WrongSliceLength { slice: "qd", .. })
    ));
    assert!(matches!(
        robot.forward_dynamics(&BaseState::fixed(), &q, &q, &q[..1], &[], &mut output),
        Err(Error::WrongSliceLength {
            slice: "forward dynamics generalized forces",
            ..
        })
    ));
    assert!(matches!(
        robot.forward_dynamics(&BaseState::fixed(), &q, &q, &q, &[], &mut output[..1]),
        Err(Error::WrongSliceLength {
            slice: "forward dynamics output",
            ..
        })
    ));
    assert!(matches!(
        robot.forward_dynamics(
            &BaseState::fixed(),
            &q,
            &q,
            &q,
            &[foreign_load],
            &mut output
        ),
        Err(Error::InvalidLinkId)
    ));

    let invalid_fixed_base = BaseState::new(
        Frame::identity(),
        Twist::zeros(),
        Twist::new(Vector3::x(), Vector3::zeros()),
    )
    .unwrap();
    assert!(matches!(
        robot.forward_dynamics(&invalid_fixed_base, &q, &q, &q, &[], &mut output),
        Err(Error::InvalidBaseState {
            field: "acceleration",
            ..
        })
    ));
}

#[test]
fn forward_dynamics_reports_singular_joint_and_floating_base_inertia() {
    let mut singular_joint = Robot::from_urdf(fixture("singular_arm.urdf")).unwrap();
    let mut joint_output = [0.0];
    assert!(matches!(
        singular_joint.forward_dynamics(
            &BaseState::fixed(),
            &[0.0],
            &[0.0],
            &[0.0],
            &[],
            &mut joint_output,
        ),
        Err(Error::ForwardDynamicsSingularJointInertia { joint_index: 0 })
    ));

    let mut singular_base =
        Robot::from_urdf_with_base(fixture("singular_floating_base.urdf"), BaseMode::Floating)
            .unwrap();
    let base = BaseState::new(Frame::identity(), Twist::zeros(), Twist::zeros()).unwrap();
    let mut base_output = [0.0; 6];
    let result = singular_base.forward_dynamics(&base, &[], &[], &[0.0; 6], &[], &mut base_output);
    assert!(
        matches!(result, Err(Error::ForwardDynamicsSingularBaseInertia)),
        "unexpected result: {result:?}, output: {base_output:?}"
    );
}
