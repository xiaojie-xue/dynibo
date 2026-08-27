mod support;

use support::context::TestRootType as RootType;

use dynibo::{BaseState, Error, FloatingRobot, Frame, IndexedLoad, Robot, Twist, Wrench};
use nalgebra::{Translation3, UnitQuaternion, Vector3};
use support::{
    context::TestContext,
    fixtures::{FLOATING_ARM, MIXED_ARM, TREE_ARM, fixture_path},
    numeric::{Tolerance, assert_slice_close},
    states::deterministic_joint_state,
};

#[test]
fn fixed_forward_dynamics_inverts_inverse_dynamics_for_tree_and_mixed_joints() {
    for (fixture, target_name) in [(TREE_ARM, "right_tool"), (MIXED_ARM, "tool")] {
        let mut robot = fixture.robot(RootType::Fixed);
        let joint_count = robot.joint_count();
        let load = IndexedLoad {
            link: robot.link_id(target_name).unwrap(),
            wrench: Wrench::new(Vector3::new(0.2, -0.1, 0.3), Vector3::new(-0.7, 0.4, -0.2)),
        };

        for sample in 0..24 {
            let state = deterministic_joint_state(joint_count, sample);
            let q = state.q;
            let qd = state.qd;
            let expected_qdd = state.qdd;
            let mut generalized_forces = vec![0.0; joint_count];
            robot
                .inverse_dynamics(&q, &qd, &expected_qdd, &[load], &mut generalized_forces)
                .unwrap();

            let mut actual_qdd = vec![f64::NAN; joint_count];
            robot
                .forward_dynamics(&q, &qd, &generalized_forces, &[load], &mut actual_qdd)
                .unwrap();
            let context = TestContext::new("rnea-aba-round-trip", fixture.name)
                .sample(sample)
                .target(target_name)
                .load_case("single");
            assert_slice_close(
                &actual_qdd,
                &expected_qdd,
                Tolerance::new(2.0e-10, 2.0e-10),
                &context,
            );
        }
    }
}

#[test]
fn floating_forward_dynamics_inverts_moving_base_inverse_dynamics() {
    let mut robot = FloatingRobot::from_urdf(FLOATING_ARM.path()).unwrap();
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
    assert_slice_close(
        &actual,
        &expected,
        Tolerance::new(3.0e-10, 3.0e-10),
        &TestContext::new("floating-rnea-aba-round-trip", FLOATING_ARM.name)
            .base_mode(RootType::Floating)
            .target("tool")
            .load_case("single"),
    );
}

#[test]
fn forward_dynamics_validates_dimensions_base_state_and_load_ids() {
    let mut robot = FLOATING_ARM.robot(RootType::Fixed);
    let other = FLOATING_ARM.robot(RootType::Fixed);
    let foreign_load = IndexedLoad {
        link: other.link_id("tool").unwrap(),
        wrench: Wrench::zeros(),
    };
    let q = [0.0; 2];
    let mut output = [0.0; 2];

    assert!(matches!(
        robot.forward_dynamics(&q[..1], &q, &q, &[], &mut output),
        Err(Error::WrongSliceLength { slice: "q", .. })
    ));
    assert!(matches!(
        robot.forward_dynamics(&q, &q[..1], &q, &[], &mut output),
        Err(Error::WrongSliceLength { slice: "qd", .. })
    ));
    assert!(matches!(
        robot.forward_dynamics(&q, &q, &q[..1], &[], &mut output),
        Err(Error::WrongSliceLength {
            slice: "forward dynamics generalized forces",
            ..
        })
    ));
    assert!(matches!(
        robot.forward_dynamics(&q, &q, &q, &[], &mut output[..1]),
        Err(Error::WrongSliceLength {
            slice: "forward dynamics output",
            ..
        })
    ));
    assert!(matches!(
        robot.forward_dynamics(&q, &q, &q, &[foreign_load], &mut output),
        Err(Error::InvalidLinkId)
    ));
}

#[test]
fn forward_dynamics_reports_singular_joint_and_floating_base_inertia() {
    let mut singular_joint = Robot::from_urdf(fixture_path("singular_arm.urdf")).unwrap();
    let mut joint_output = [0.0];
    assert!(matches!(
        singular_joint.forward_dynamics(&[0.0], &[0.0], &[0.0], &[], &mut joint_output,),
        Err(Error::ForwardDynamicsSingularJointInertia { joint_index: 0 })
    ));

    let mut singular_base =
        FloatingRobot::from_urdf(fixture_path("singular_floating_base.urdf")).unwrap();
    let base = BaseState::new(Frame::identity(), Twist::zeros(), Twist::zeros()).unwrap();
    let mut base_output = [0.0; 6];
    let result = singular_base.forward_dynamics(&base, &[], &[], &[0.0; 6], &[], &mut base_output);
    assert!(
        matches!(result, Err(Error::ForwardDynamicsSingularBaseInertia)),
        "unexpected result: {result:?}, output: {base_output:?}"
    );
}
