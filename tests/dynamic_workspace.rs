use std::path::PathBuf;

use approx::assert_relative_eq;
use dynibo::{Error, Frame, IndexedLoad, InverseKinematicsOptions, Robot, Twist, Wrench};
use nalgebra::{Matrix3, SMatrix, SVector, Translation3, UnitQuaternion, Vector3};

fn urdf_path(file_name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(file_name)
}

fn test_arm() -> Robot {
    Robot::from_urdf(urdf_path("test_arm.urdf")).expect("test URDF must load")
}

fn tree_arm() -> Robot {
    Robot::from_urdf(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf"))
        .expect("tree URDF must load")
}

fn assert_slice_close(actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len());
    for (actual, expected) in actual.iter().zip(expected) {
        assert_relative_eq!(actual, expected, epsilon = 2.0e-12);
    }
}

fn assert_wrong_length<T: std::fmt::Debug>(
    result: dynibo::Result<T>,
    expected_slice: &'static str,
) {
    match result {
        Err(Error::WrongSliceLength { slice, .. }) => assert_eq!(slice, expected_slice),
        other => panic!("expected WrongSliceLength for {expected_slice}, found {other:?}"),
    }
}

fn assert_invalid_workspace<T: std::fmt::Debug>(result: dynibo::Result<T>) {
    assert!(matches!(result, Err(Error::InvalidWorkspace)));
}

fn assert_invalid_link<T: std::fmt::Debug>(result: dynibo::Result<T>) {
    assert!(matches!(result, Err(Error::InvalidLinkId)));
}

#[test]
fn all_dynamic_calculations_match_pinocchio_references() {
    let robot = test_arm();
    let target_id = robot.link_id("test_link_4").unwrap();
    let mut workspace = robot.workspace();
    assert_eq!(workspace.joint_count(), 4);

    let q = [0.2, 1.0, -0.7, 0.4];
    let qd = [-0.3, 0.5, -0.2, 0.8];
    let qdd = [0.7, -0.4, 0.1, 0.3];
    // These references are also checked directly against Pinocchio in
    // `pinocchio_oracle::serial_arm_calculations_match_pinocchio`.
    let base = Frame::from_parts(
        Translation3::new(0.3, -0.2, 0.5),
        UnitQuaternion::from_euler_angles(0.2, -0.4, 0.1),
    );
    let tool = Frame::translation(0.1, -0.03, 0.2);

    let frame = robot
        .forward_kinematics(&q, target_id, &mut workspace)
        .unwrap();
    let expected_rotation = Matrix3::from_column_slice(&[
        0.7495962650805186,
        0.15195068551164034,
        0.6442176872376912,
        -0.6313762241158434,
        -0.127986296809854,
        0.7648421872844885,
        0.19866933079506122,
        -0.9800665778412416,
        2.220446049250313e-16,
    ]);
    assert_relative_eq!(
        frame.rotation.to_rotation_matrix().matrix(),
        &expected_rotation,
        epsilon = 2.0e-12
    );
    assert_relative_eq!(
        frame.translation.vector,
        Vector3::new(0.450338323287074, 0.09128809750443889, 0.46592677713692876,),
        epsilon = 2.0e-12
    );

    let mut jacobian = vec![f64::NAN; 24];
    robot
        .jacobian(&q, target_id, &mut workspace, &mut jacobian)
        .unwrap();
    let expected_jacobian = SMatrix::<f64, 6, 4>::from_column_slice(&[
        0.0,
        0.0,
        1.0,
        -0.09128809750443889,
        0.450338323287074,
        0.0,
        0.19866933079506122,
        -0.9800665778412416,
        2.220446049250313e-16,
        -0.3507920715863345,
        -0.07110907328742656,
        0.45949768461548657,
        0.19866933079506122,
        -0.9800665778412416,
        2.220446049250313e-16,
        -0.08688884328765467,
        -0.0176132405081479,
        0.2866009467376818,
        0.19866933079506122,
        -0.9800665778412416,
        2.220446049250313e-16,
        0.0,
        0.0,
        0.0,
    ]);
    assert_relative_eq!(
        SMatrix::<f64, 6, 4>::from_column_slice(&jacobian),
        expected_jacobian,
        epsilon = 2.0e-12
    );

    let velocity = robot
        .forward_velocity_kinematics(&q, &qd, target_id, &base, &tool, &mut workspace)
        .unwrap();
    let mut tool_jacobian = expected_jacobian;
    let offset_world = frame.rotation * tool.translation.vector;
    for column in 0..4 {
        let angular = tool_jacobian.fixed_view::<3, 1>(0, column).into_owned();
        let shifted =
            tool_jacobian.fixed_view::<3, 1>(3, column).into_owned() + angular.cross(&offset_world);
        tool_jacobian
            .fixed_view_mut::<3, 1>(3, column)
            .copy_from(&shifted);
    }
    let expected_velocity = tool_jacobian * SVector::<f64, 4>::from(qd);
    assert_relative_eq!(
        velocity.to_vector(),
        SVector::<f64, 6>::from_iterator(
            (base.rotation * expected_velocity.fixed_rows::<3>(0).into_owned())
                .iter()
                .chain((base.rotation * expected_velocity.fixed_rows::<3>(3).into_owned()).iter())
                .copied()
        ),
        epsilon = 2.0e-12
    );

    let acceleration = robot
        .forward_acceleration_kinematics(&q, &qd, &qdd, target_id, &mut workspace)
        .unwrap();
    assert_relative_eq!(
        acceleration.to_vector(),
        SVector::<f64, 6>::new(
            -0.32342197068760986,
            -0.06556087916237024,
            0.7000000000000001,
            -0.05966580553815265,
            0.4148023496219572,
            -0.2304357035369144,
        ),
        epsilon = 2.0e-12
    );

    let mut gravity = vec![f64::NAN; 4];
    robot
        .gravity(&q, &Frame::identity(), &[], &mut workspace, &mut gravity)
        .unwrap();
    assert_relative_eq!(
        SVector::<f64, 4>::from_column_slice(&gravity),
        SVector::<f64, 4>::new(
            1.7763568394002505e-15,
            39.629058959145354,
            17.60815765611755,
            0.053134179784508524,
        ),
        epsilon = 2.0e-11
    );

    let mut dynamics = vec![f64::NAN; 4];
    robot
        .inverse_dynamics(
            &q,
            &qd,
            &qdd,
            &Frame::identity(),
            Twist::zeros(),
            Twist::zeros(),
            &[],
            &mut workspace,
            &mut dynamics,
        )
        .unwrap();
    assert_relative_eq!(
        SVector::<f64, 4>::from_column_slice(&dynamics),
        SVector::<f64, 4>::new(
            1.7649236924309104,
            38.319908179086525,
            17.136450444507805,
            0.05169960944426318,
        ),
        epsilon = 2.0e-11
    );

    let desired_q = [0.2, 1.0, -1.2, 0.45];
    let desired = robot
        .forward_kinematics(&desired_q, target_id, &mut workspace)
        .unwrap();
    let mut solution = vec![f64::NAN; 4];
    robot
        .inverse_kinematics(
            &[0.0; 4],
            target_id,
            &desired,
            InverseKinematicsOptions::default(),
            &mut workspace,
            &mut solution,
        )
        .unwrap();
    let solved = robot
        .forward_kinematics(&solution, target_id, &mut workspace)
        .unwrap();
    assert_relative_eq!(solved, desired, epsilon = 1.0e-6);
}

#[test]
fn workspace_reuse_clears_jacobian_load_and_solver_state() {
    let robot = tree_arm();
    let mut workspace = robot.workspace();
    let left_id = robot.link_id("left_tool").unwrap();
    let right_id = robot.link_id("right_tool").unwrap();
    let q_a = [0.2, -0.3, 0.4, -0.5, 0.6, -0.7, 0.8];
    let q_b = [-0.4, 0.2, -0.1, 0.7, -0.3, 0.5, -0.6];
    let load = IndexedLoad {
        link: left_id,
        wrench: Wrench::new(Vector3::new(0.3, -0.2, 0.4), Vector3::new(1.0, 0.5, -0.7)),
    };

    let mut first = vec![0.0; 7];
    let mut middle = vec![0.0; 7];
    let mut third = vec![0.0; 7];
    robot
        .gravity(
            &q_a,
            &Frame::identity(),
            &[load],
            &mut workspace,
            &mut first,
        )
        .unwrap();
    robot
        .gravity(&q_b, &Frame::identity(), &[], &mut workspace, &mut middle)
        .unwrap();
    robot
        .gravity(
            &q_a,
            &Frame::identity(),
            &[load],
            &mut workspace,
            &mut third,
        )
        .unwrap();
    assert_slice_close(&first, &third);
    let mut expected_middle = vec![0.0; 7];
    let mut clean_workspace = robot.workspace();
    robot
        .gravity(
            &q_b,
            &Frame::identity(),
            &[],
            &mut clean_workspace,
            &mut expected_middle,
        )
        .unwrap();
    assert_slice_close(&middle, &expected_middle);

    let zero = [0.0; 7];
    let mut first_rnea = vec![0.0; 7];
    let mut middle_rnea = vec![0.0; 7];
    let mut third_rnea = vec![0.0; 7];
    robot
        .inverse_dynamics(
            &q_a,
            &zero,
            &zero,
            &Frame::identity(),
            Twist::zeros(),
            Twist::zeros(),
            &[load, load],
            &mut workspace,
            &mut first_rnea,
        )
        .unwrap();
    robot
        .inverse_dynamics(
            &q_b,
            &zero,
            &zero,
            &Frame::identity(),
            Twist::zeros(),
            Twist::zeros(),
            &[],
            &mut workspace,
            &mut middle_rnea,
        )
        .unwrap();
    robot
        .inverse_dynamics(
            &q_a,
            &zero,
            &zero,
            &Frame::identity(),
            Twist::zeros(),
            Twist::zeros(),
            &[load, load],
            &mut workspace,
            &mut third_rnea,
        )
        .unwrap();
    assert_slice_close(&first_rnea, &third_rnea);
    let mut expected_middle_rnea = vec![0.0; 7];
    robot
        .inverse_dynamics(
            &q_b,
            &zero,
            &zero,
            &Frame::identity(),
            Twist::zeros(),
            Twist::zeros(),
            &[],
            &mut clean_workspace,
            &mut expected_middle_rnea,
        )
        .unwrap();
    assert_slice_close(&middle_rnea, &expected_middle_rnea);

    let mut jacobian = vec![f64::NAN; 42];
    robot
        .jacobian(&q_a, left_id, &mut workspace, &mut jacobian)
        .unwrap();
    robot
        .jacobian(&q_b, right_id, &mut workspace, &mut jacobian)
        .unwrap();
    let mut expected_jacobian = vec![0.0; 42];
    robot
        .jacobian(&q_b, right_id, &mut clean_workspace, &mut expected_jacobian)
        .unwrap();
    assert_slice_close(&jacobian, &expected_jacobian);
    robot
        .jacobian(&q_a, left_id, &mut workspace, &mut jacobian)
        .unwrap();
    robot
        .jacobian(&q_a, left_id, &mut clean_workspace, &mut expected_jacobian)
        .unwrap();
    assert_slice_close(&jacobian, &expected_jacobian);

    let desired = robot
        .forward_kinematics(&q_a, left_id, &mut workspace)
        .unwrap();
    let mut solution = vec![0.0; 7];
    robot
        .inverse_kinematics(
            &[0.0; 7],
            left_id,
            &desired,
            InverseKinematicsOptions::default(),
            &mut workspace,
            &mut solution,
        )
        .unwrap();
    assert_relative_eq!(
        robot
            .forward_kinematics(&solution, left_id, &mut workspace)
            .unwrap(),
        desired,
        epsilon = 1.0e-6
    );
}

#[test]
fn dynamic_api_rejects_wrong_models_and_lengths() {
    let robot_a = test_arm();
    let robot_b = test_arm();
    let target_a = robot_a.link_id("test_link_4").unwrap();
    let target_b = robot_b.link_id("test_link_4").unwrap();
    let mut workspace_a = robot_a.workspace();
    let mut workspace_b = robot_b.workspace();
    let q = [0.0; 4];

    assert!(matches!(
        robot_a.forward_kinematics(&q, target_b, &mut workspace_a),
        Err(Error::InvalidLinkId)
    ));
    assert!(matches!(
        robot_a.forward_kinematics(&q, target_a, &mut workspace_b),
        Err(Error::InvalidWorkspace)
    ));
    assert!(matches!(
        robot_a.forward_kinematics(&q[..3], target_a, &mut workspace_a),
        Err(Error::WrongSliceLength {
            slice: "q",
            expected: 4,
            actual: 3
        })
    ));
    let mut wrong_jacobian = [0.0; 23];
    assert!(matches!(
        robot_a.jacobian(&q, target_a, &mut workspace_a, &mut wrong_jacobian),
        Err(Error::WrongSliceLength {
            slice: "jacobian output",
            expected: 24,
            actual: 23
        })
    ));
    let invalid_load = IndexedLoad {
        link: target_b,
        wrench: Wrench::zeros(),
    };
    let mut output = [0.0; 4];
    assert!(matches!(
        robot_a.gravity(
            &q,
            &Frame::identity(),
            &[invalid_load],
            &mut workspace_a,
            &mut output,
        ),
        Err(Error::InvalidLinkId)
    ));

    let clone = robot_a.clone();
    assert!(
        clone
            .forward_kinematics(&q, target_a, &mut workspace_a)
            .is_ok()
    );
}

#[test]
fn every_dynamic_api_validates_its_workspace_link_and_slices() {
    let robot = test_arm();
    let other = test_arm();
    let target = robot.link_id("test_link_4").unwrap();
    let foreign_target = other.link_id("test_link_4").unwrap();
    let q = [0.0; 4];
    let short = [0.0; 3];
    let mut output = [0.0; 4];
    let mut short_output = [0.0; 3];
    let mut jacobian = [0.0; 24];
    let options = InverseKinematicsOptions::default();
    let identity = Frame::identity();

    let mut foreign_workspace = other.workspace();
    assert_invalid_workspace(robot.jacobian(&q, target, &mut foreign_workspace, &mut jacobian));
    assert_invalid_workspace(robot.inverse_kinematics(
        &q,
        target,
        &identity,
        options,
        &mut foreign_workspace,
        &mut output,
    ));
    assert_invalid_workspace(robot.forward_velocity_kinematics(
        &q,
        &q,
        target,
        &identity,
        &identity,
        &mut foreign_workspace,
    ));
    assert_invalid_workspace(robot.forward_acceleration_kinematics(
        &q,
        &q,
        &q,
        target,
        &mut foreign_workspace,
    ));
    assert_invalid_workspace(robot.gravity(
        &q,
        &identity,
        &[],
        &mut foreign_workspace,
        &mut output,
    ));
    assert_invalid_workspace(robot.inverse_dynamics(
        &q,
        &q,
        &q,
        &identity,
        Twist::zeros(),
        Twist::zeros(),
        &[],
        &mut foreign_workspace,
        &mut output,
    ));

    let mut workspace = robot.workspace();
    assert_wrong_length(
        robot.jacobian(&short, target, &mut workspace, &mut jacobian),
        "q",
    );
    assert_invalid_link(robot.jacobian(&q, foreign_target, &mut workspace, &mut jacobian));

    assert_wrong_length(
        robot.inverse_kinematics(
            &short,
            target,
            &identity,
            options,
            &mut workspace,
            &mut output,
        ),
        "initial_q",
    );
    assert_wrong_length(
        robot.inverse_kinematics(
            &q,
            target,
            &identity,
            options,
            &mut workspace,
            &mut short_output,
        ),
        "inverse kinematics output",
    );
    assert_invalid_link(robot.inverse_kinematics(
        &q,
        foreign_target,
        &identity,
        options,
        &mut workspace,
        &mut output,
    ));

    assert_wrong_length(
        robot.forward_velocity_kinematics(&short, &q, target, &identity, &identity, &mut workspace),
        "q",
    );
    assert_wrong_length(
        robot.forward_velocity_kinematics(&q, &short, target, &identity, &identity, &mut workspace),
        "qd",
    );
    assert_invalid_link(robot.forward_velocity_kinematics(
        &q,
        &q,
        foreign_target,
        &identity,
        &identity,
        &mut workspace,
    ));

    assert_wrong_length(
        robot.forward_acceleration_kinematics(&short, &q, &q, target, &mut workspace),
        "q",
    );
    assert_wrong_length(
        robot.forward_acceleration_kinematics(&q, &short, &q, target, &mut workspace),
        "qd",
    );
    assert_wrong_length(
        robot.forward_acceleration_kinematics(&q, &q, &short, target, &mut workspace),
        "qdd",
    );
    assert_invalid_link(robot.forward_acceleration_kinematics(
        &q,
        &q,
        &q,
        foreign_target,
        &mut workspace,
    ));

    assert_wrong_length(
        robot.gravity(&short, &identity, &[], &mut workspace, &mut output),
        "q",
    );
    assert_wrong_length(
        robot.gravity(&q, &identity, &[], &mut workspace, &mut short_output),
        "gravity output",
    );

    assert_wrong_length(
        robot.inverse_dynamics(
            &short,
            &q,
            &q,
            &identity,
            Twist::zeros(),
            Twist::zeros(),
            &[],
            &mut workspace,
            &mut output,
        ),
        "q",
    );
    assert_wrong_length(
        robot.inverse_dynamics(
            &q,
            &short,
            &q,
            &identity,
            Twist::zeros(),
            Twist::zeros(),
            &[],
            &mut workspace,
            &mut output,
        ),
        "qd",
    );
    assert_wrong_length(
        robot.inverse_dynamics(
            &q,
            &q,
            &short,
            &identity,
            Twist::zeros(),
            Twist::zeros(),
            &[],
            &mut workspace,
            &mut output,
        ),
        "qdd",
    );
    assert_wrong_length(
        robot.inverse_dynamics(
            &q,
            &q,
            &q,
            &identity,
            Twist::zeros(),
            Twist::zeros(),
            &[],
            &mut workspace,
            &mut short_output,
        ),
        "inverse dynamics output",
    );
    let invalid_load = IndexedLoad {
        link: foreign_target,
        wrench: Wrench::zeros(),
    };
    assert_invalid_link(robot.inverse_dynamics(
        &q,
        &q,
        &q,
        &identity,
        Twist::zeros(),
        Twist::zeros(),
        &[invalid_load],
        &mut workspace,
        &mut output,
    ));
}

#[test]
fn dynamic_root_results_are_zero_or_identity() {
    let robot = test_arm();
    let root = robot.link_id("test_base_link").unwrap();
    let mut workspace = robot.workspace();
    let q = [0.1, -0.2, 0.3, -0.4];
    let frame = robot.forward_kinematics(&q, root, &mut workspace).unwrap();
    assert_relative_eq!(frame, Frame::identity(), epsilon = 2.0e-12);
    let mut jacobian = [f64::NAN; 24];
    robot
        .jacobian(&q, root, &mut workspace, &mut jacobian)
        .unwrap();
    assert_eq!(jacobian, [0.0; 24]);
    let acceleration = robot
        .forward_acceleration_kinematics(&q, &[0.2; 4], &[0.3; 4], root, &mut workspace)
        .unwrap();
    assert_eq!(acceleration, Twist::zeros());

    let root_load = IndexedLoad {
        link: root,
        wrench: Wrench::new(Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0)),
    };
    let mut baseline = [0.0; 4];
    let mut loaded = [f64::NAN; 4];
    robot
        .gravity(&q, &Frame::identity(), &[], &mut workspace, &mut baseline)
        .unwrap();
    robot
        .gravity(
            &q,
            &Frame::identity(),
            &[root_load],
            &mut workspace,
            &mut loaded,
        )
        .unwrap();
    assert_slice_close(&loaded, &baseline);
}
