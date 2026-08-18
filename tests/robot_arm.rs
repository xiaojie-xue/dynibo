// Several regression inputs deliberately preserve the decimal constants used
// by the original C++ tests instead of replacing them with exact PI fractions.
#![allow(clippy::approx_constant)]

use std::{
    f64::consts::{FRAC_PI_2, PI},
    path::PathBuf,
};

use approx::{assert_abs_diff_eq, assert_relative_eq};
use dynibo::{
    Error, Frame, IndexedLoad, InverseKinematicsOptions, Joint, JointType, Link, Robot, Twist,
    Wrench,
};
use nalgebra::{Isometry3, Matrix3, SMatrix, SVector, Translation3, UnitQuaternion, Vector3};

type JointVector<const N: usize> = SVector<f64, N>;
type Jacobian<const N: usize> = SMatrix<f64, 6, N>;

const STANDARD_GRAVITY: f64 = 9.80665;

#[derive(Clone, Copy)]
struct Load<'a> {
    link: &'a Link,
    wrench: Wrench,
}

trait DynamicTestApi {
    fn test_forward_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: &Link,
    ) -> dynibo::Result<Frame>;
    fn test_jacobian<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: &Link,
    ) -> dynibo::Result<Jacobian<N>>;
    fn test_inverse_kinematics<const N: usize>(
        &self,
        initial_q: &JointVector<N>,
        target: &Link,
        desired: &Frame,
        options: InverseKinematicsOptions,
    ) -> dynibo::Result<JointVector<N>>;
    fn test_forward_velocity_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        target: &Link,
        base: &Frame,
        tool: &Frame,
    ) -> dynibo::Result<Twist>;
    fn test_forward_acceleration_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        target: &Link,
    ) -> dynibo::Result<Twist>;
    fn test_gravity<const N: usize>(
        &self,
        q: &JointVector<N>,
        base: &Frame,
        loads: &[Load<'_>],
    ) -> dynibo::Result<JointVector<N>>;
    fn test_inverse_dynamics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        loads: &[Load<'_>],
    ) -> dynibo::Result<JointVector<N>>;
}

impl DynamicTestApi for Robot {
    fn test_forward_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: &Link,
    ) -> dynibo::Result<Frame> {
        let mut workspace = self.workspace();
        self.forward_kinematics(q.as_slice(), self.link_id(target.name())?, &mut workspace)
    }

    fn test_jacobian<const N: usize>(
        &self,
        q: &JointVector<N>,
        target: &Link,
    ) -> dynibo::Result<Jacobian<N>> {
        let mut workspace = self.workspace();
        let mut output = Jacobian::<N>::zeros();
        self.jacobian(
            q.as_slice(),
            self.link_id(target.name())?,
            &mut workspace,
            output.as_mut_slice(),
        )?;
        Ok(output)
    }

    fn test_inverse_kinematics<const N: usize>(
        &self,
        initial_q: &JointVector<N>,
        target: &Link,
        desired: &Frame,
        options: InverseKinematicsOptions,
    ) -> dynibo::Result<JointVector<N>> {
        let mut workspace = self.workspace();
        let mut output = JointVector::<N>::zeros();
        self.inverse_kinematics(
            initial_q.as_slice(),
            self.link_id(target.name())?,
            desired,
            options,
            &mut workspace,
            output.as_mut_slice(),
        )?;
        Ok(output)
    }

    fn test_forward_velocity_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        target: &Link,
        base: &Frame,
        tool: &Frame,
    ) -> dynibo::Result<Twist> {
        let mut robot = self.clone();
        robot.set_base_frame(*base)?;
        let mut workspace = robot.workspace();
        robot.forward_velocity_kinematics(
            q.as_slice(),
            qd.as_slice(),
            robot.link_id(target.name())?,
            tool,
            &mut workspace,
        )
    }

    fn test_forward_acceleration_kinematics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        target: &Link,
    ) -> dynibo::Result<Twist> {
        let mut workspace = self.workspace();
        self.forward_acceleration_kinematics(
            q.as_slice(),
            qd.as_slice(),
            qdd.as_slice(),
            self.link_id(target.name())?,
            &mut workspace,
        )
    }

    fn test_gravity<const N: usize>(
        &self,
        q: &JointVector<N>,
        base: &Frame,
        loads: &[Load<'_>],
    ) -> dynibo::Result<JointVector<N>> {
        let loads = loads
            .iter()
            .map(|load| {
                Ok(IndexedLoad {
                    link: self.link_id(load.link.name())?,
                    wrench: load.wrench,
                })
            })
            .collect::<dynibo::Result<Vec<_>>>()?;
        let mut robot = self.clone();
        robot.set_base_frame(*base)?;
        let mut workspace = robot.workspace();
        let mut output = JointVector::<N>::zeros();
        robot.gravity(q.as_slice(), &loads, &mut workspace, output.as_mut_slice())?;
        Ok(output)
    }

    fn test_inverse_dynamics<const N: usize>(
        &self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        loads: &[Load<'_>],
    ) -> dynibo::Result<JointVector<N>> {
        let loads = loads
            .iter()
            .map(|load| {
                Ok(IndexedLoad {
                    link: self.link_id(load.link.name())?,
                    wrench: load.wrench,
                })
            })
            .collect::<dynibo::Result<Vec<_>>>()?;
        let mut workspace = self.workspace();
        let mut output = JointVector::<N>::zeros();
        self.inverse_dynamics(
            q.as_slice(),
            qd.as_slice(),
            qdd.as_slice(),
            &loads,
            &mut workspace,
            output.as_mut_slice(),
        )?;
        Ok(output)
    }
}

fn test_arm() -> Robot {
    Robot::from_urdf(urdf_path("test_arm.urdf")).expect("test URDF must load")
}

fn end_link(arm: &Robot) -> &Link {
    arm.leaf_links()
        .next()
        .expect("robot must have a leaf link")
}

fn urdf_path(file_name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(file_name)
}

fn tree_arm() -> Robot {
    Robot::from_urdf(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf"))
        .expect("tree URDF must load")
}

#[allow(clippy::too_many_arguments)]
fn joint(name: &str, joint_type: JointType, xyz: [f64; 3], rpy: [f64; 3], axis: [f64; 3]) -> Joint {
    Joint::new(
        name,
        joint_type,
        Isometry3::from_parts(
            Translation3::new(xyz[0], xyz[1], xyz[2]),
            UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
        ),
        Vector3::new(axis[0], axis[1], axis[2]),
        -10.0,
        10.0,
        100.0,
    )
    .unwrap()
}

#[test]
fn joint_and_loaded_link_preserve_their_parameters() {
    let mut joint = Joint::new(
        "joint_1",
        JointType::Revolute,
        Isometry3::from_parts(
            Translation3::new(2.0, 3.0, 4.0),
            UnitQuaternion::from_euler_angles(0.0, 2.0, 1.0),
        ),
        Vector3::z(),
        -3.14,
        3.14,
        100.0,
    )
    .unwrap();
    let arm = test_arm();
    let link = arm.link("test_link_2").unwrap();

    assert_eq!(joint.name(), "joint_1");
    assert_eq!(link.name(), "test_link_2");
    assert_eq!(joint.joint_type(), JointType::Revolute);
    assert_abs_diff_eq!(joint.lower_limit(), -3.14);
    assert_abs_diff_eq!(joint.upper_limit(), 3.14);
    assert_abs_diff_eq!(joint.velocity_limit(), 100.0);
    assert_eq!(link.mass(), 7.016);
    assert_abs_diff_eq!(joint.origin().translation.vector.x, 2.0);
    assert_abs_diff_eq!(link.center_of_mass().z, 0.129994);
    assert_eq!(
        link.inertia(),
        &Matrix3::new(
            0.016533114,
            0.000002097,
            0.006290504,
            0.000002097,
            0.096957399,
            -0.000005506,
            0.006290504,
            -0.000005506,
            0.093672064,
        )
    );
    assert_relative_eq!(joint.axis().as_ref(), &Vector3::z(), epsilon = 1.0e-12);
    assert!(joint.is_over_limit(4.0));
    assert!(!joint.is_over_limit(0.0));
    assert_abs_diff_eq!(joint.set_position(10.0), 3.14);
    assert_abs_diff_eq!(joint.position(), 3.14);
    assert_abs_diff_eq!(joint.set_velocity(-200.0), -100.0);
    assert_abs_diff_eq!(joint.velocity(), -100.0);
    assert_abs_diff_eq!(joint.set_acceleration(12.0), 12.0);
    assert_abs_diff_eq!(joint.acceleration(), 12.0);
    assert_abs_diff_eq!(joint.set_home_offset(-0.25), -0.25);
    assert_abs_diff_eq!(joint.home_offset(), -0.25);
    assert_abs_diff_eq!(
        joint.active_force(Wrench::new(Vector3::new(1.0, 2.0, 3.0), Vector3::zeros())),
        3.0
    );
}

#[test]
fn fixed_joint_ignores_its_axis_but_moving_joints_validate_theirs() {
    let fixed = Joint::new(
        "fixed",
        JointType::Fixed,
        Frame::identity(),
        Vector3::zeros(),
        0.0,
        0.0,
        0.0,
    )
    .expect("a fixed joint does not need a motion axis");
    assert_eq!(fixed.joint_type(), JointType::Fixed);

    for joint_type in [JointType::Revolute, JointType::Prismatic] {
        let error = Joint::new(
            "moving",
            joint_type,
            Frame::identity(),
            Vector3::zeros(),
            -1.0,
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(
            error,
            Error::InvalidJointAxis { ref joint } if joint == "moving"
        ));
    }
}

#[test]
fn revolute_and_prismatic_joint_frames_match_urdf_semantics() {
    let revolute = joint(
        "revolute",
        JointType::Revolute,
        [0.0, 0.0, 0.226],
        [0.0, 0.0, FRAC_PI_2],
        [0.0, 0.0, 1.0],
    );
    let frame = revolute.frame(0.3 * PI);
    let expected = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.8 * PI);
    assert_relative_eq!(frame.rotation, expected, epsilon = 1.0e-12);
    assert_abs_diff_eq!(frame.translation.vector.z, 0.226);

    let prismatic = joint(
        "slide",
        JointType::Prismatic,
        [1.0, 0.0, 0.0],
        [0.0; 3],
        [0.0, 1.0, 0.0],
    );
    assert_relative_eq!(
        prismatic.frame(0.25).translation.vector,
        Vector3::new(1.0, 0.25, 0.0),
        epsilon = 1.0e-12
    );
    assert_abs_diff_eq!(
        prismatic.active_force(Wrench::new(Vector3::zeros(), Vector3::new(2.0, 3.0, 4.0))),
        3.0
    );

    let fixed = joint(
        "fixed",
        JointType::Fixed,
        [0.1, 0.2, 0.3],
        [0.0; 3],
        [0.0; 3],
    );
    assert_relative_eq!(fixed.frame(123.0), *fixed.origin(), epsilon = 1.0e-12);
    assert_abs_diff_eq!(
        fixed.active_force(Wrench::new(Vector3::z(), Vector3::x())),
        0.0
    );
}

#[test]
fn urdf_rs_loads_test_arm_and_checks_calculation_size() {
    let arm = test_arm();
    assert_eq!(arm.name(), "test_arm");
    assert_eq!(arm.links().len(), 5);
    assert_eq!(arm.links()[0].name(), "test_base_link");
    assert_eq!(arm.links()[1].name(), "test_link_1");
    assert_eq!(arm.joints().len(), 4);
    assert_eq!(arm.joints()[0].name(), "test_joint_1");
    assert_eq!(arm.link_count(), 5);
    assert_eq!(arm.joint_count(), 4);
    assert_eq!(arm.link("test_link_1").unwrap().name(), "test_link_1");
    assert!(matches!(
        arm.link("missing_link"),
        Err(Error::UnknownLink { name }) if name == "missing_link"
    ));
    let link_id = arm.link_id("test_link_1").unwrap();
    let mut workspace = arm.workspace();
    arm.forward_kinematics(&[0.0; 4], link_id, &mut workspace)
        .expect("a model-owned link ID remains valid");
    let other_arm = test_arm();
    let other_link_id = other_arm.link_id("test_link_1").unwrap();
    assert!(matches!(
        arm.forward_kinematics(&[0.0; 4], other_link_id, &mut workspace),
        Err(Error::InvalidLinkId)
    ));
    assert_abs_diff_eq!(arm.links()[2].mass(), 7.016);
    assert_abs_diff_eq!(arm.joints()[1].origin().translation.vector.z, 0.108);

    let wrong_size = arm
        .forward_kinematics(&[0.0; 3], link_id, &mut workspace)
        .unwrap_err();
    assert!(matches!(
        wrong_size,
        Error::WrongSliceLength {
            slice: "q",
            expected: 4,
            actual: 3
        }
    ));
}

#[test]
fn urdf_loading_reports_missing_and_malformed_files() {
    for path in [urdf_path("does_not_exist.urdf"), urdf_path("invalid.urdf")] {
        let error = Robot::from_urdf(path).expect_err("invalid input must not load");
        assert!(matches!(error, Error::Urdf(_)));
        assert!(error.to_string().starts_with("failed to parse URDF:"));
        assert!(std::error::Error::source(&error).is_some());
    }
}

#[test]
fn jacobian_agrees_with_forward_kinematics_finite_difference() {
    let arm = test_arm();
    let q = JointVector::<4>::new(0.147607, 1.014764, -1.840751, 0.825987);
    let jacobian = arm.test_jacobian(&q, end_link(&arm)).unwrap();
    let epsilon = 1.0e-7;

    for joint in 0..4 {
        let mut q_plus = q;
        let mut q_minus = q;
        q_plus[joint] += epsilon;
        q_minus[joint] -= epsilon;
        let plus = arm
            .test_forward_kinematics(&q_plus, end_link(&arm))
            .unwrap();
        let minus = arm
            .test_forward_kinematics(&q_minus, end_link(&arm))
            .unwrap();
        let linear = (plus.translation.vector - minus.translation.vector) / (2.0 * epsilon);
        let angular = (plus.rotation * minus.rotation.inverse()).scaled_axis() / (2.0 * epsilon);
        assert_relative_eq!(
            jacobian.fixed_view::<3, 1>(0, joint).into_owned(),
            angular,
            epsilon = 2.0e-8
        );
        assert_relative_eq!(
            jacobian.fixed_view::<3, 1>(3, joint).into_owned(),
            linear,
            epsilon = 2.0e-8
        );
    }
}

#[test]
fn test_arm_jacobian_matches_numeric_reference() {
    let arm = test_arm();
    let q = JointVector::<4>::new(0.205506, 1.443005, -2.645997, 1.202992);
    let expected = nalgebra::SMatrix::<f64, 6, 4>::from_row_slice(&[
        -0.0000, 0.2041, 0.2041, 0.2041, -0.0000, -0.9790, -0.9790, -0.9790, 1.0000, 0.0000,
        0.0000, 0.0000, -0.0303, -0.0367, 0.2740, 0.0, 0.1455, -0.0076, 0.0571, 0.0, 0.0, 0.1487,
        0.1079, 0.0,
    ]);
    assert_relative_eq!(
        arm.test_jacobian(&q, end_link(&arm)).unwrap(),
        expected,
        epsilon = 5.0e-4
    );

    let q = JointVector::<4>::new(0.147607, 1.014764, -1.840751, 0.825987);
    let expected = nalgebra::SMatrix::<f64, 6, 4>::from_row_slice(&[
        0.0000, 0.1471, 0.1471, 0.1471, -0.0000, -0.9891, -0.9891, -0.9891, 1.0000, 0.0000, 0.0000,
        0.0000, -0.0547, -0.0507, 0.2182, 0.0, 0.3682, -0.0075, 0.0324, 0.0, 0.0, 0.3723, 0.2033,
        0.0,
    ]);
    assert_relative_eq!(
        arm.test_jacobian(&q, end_link(&arm)).unwrap(),
        expected,
        epsilon = 5.0e-4
    );
}

#[test]
fn damped_inverse_kinematics_reaches_a_known_pose() {
    let arm = test_arm();
    let target = end_link(&arm);
    let expected_q = JointVector::<4>::new(0.2, 1.0, -1.2, 0.45);
    let desired = arm.test_forward_kinematics(&expected_q, target).unwrap();
    let initial_q = JointVector::<4>::zeros();

    let solved_q = arm
        .test_inverse_kinematics(
            &initial_q,
            target,
            &desired,
            InverseKinematicsOptions::default(),
        )
        .unwrap();
    let solved = arm.test_forward_kinematics(&solved_q, target).unwrap();

    assert!(
        (solved.translation.vector - desired.translation.vector).norm() <= 1.0e-6,
        "position did not converge: solved_q={solved_q:?}"
    );
    assert!(
        (desired.rotation * solved.rotation.inverse())
            .scaled_axis()
            .norm()
            <= 1.0e-6,
        "orientation did not converge: solved_q={solved_q:?}"
    );
}

#[test]
fn inverse_kinematics_reports_specific_solver_errors() {
    let arm = test_arm();
    let initial_q = JointVector::<4>::zeros();
    let unreachable =
        Frame::from_parts(Translation3::new(1.0, 0.0, 0.0), UnitQuaternion::identity());
    let options = InverseKinematicsOptions {
        max_iterations: 2,
        ..InverseKinematicsOptions::default()
    };
    let error = arm
        .test_inverse_kinematics(&initial_q, arm.root_link(), &unreachable, options)
        .unwrap_err();
    assert!(matches!(
        error,
        Error::IkNotConverged {
            iterations: 2,
            translation_error,
            rotation_error,
        } if (translation_error - 1.0).abs() <= 1.0e-12 && rotation_error <= 1.0e-12
    ));

    let invalid_options = InverseKinematicsOptions {
        damping: 0.0,
        ..InverseKinematicsOptions::default()
    };
    let error = arm
        .test_inverse_kinematics(
            &initial_q,
            end_link(&arm),
            &Frame::identity(),
            invalid_options,
        )
        .unwrap_err();
    assert!(matches!(
        error,
        Error::InvalidIkOptions {
            option: "damping",
            reason: "must be finite and greater than zero",
        }
    ));

    let mut non_finite_q = initial_q;
    non_finite_q[0] = f64::NAN;
    let error = arm
        .test_inverse_kinematics(
            &non_finite_q,
            end_link(&arm),
            &Frame::identity(),
            InverseKinematicsOptions::default(),
        )
        .unwrap_err();
    assert!(matches!(
        error,
        Error::NonFiniteIkInput {
            input: "initial joint vector"
        }
    ));

    let outside_q = JointVector::<4>::new(0.8, 0.0, 0.0, 0.0);
    let outside_target = arm
        .test_forward_kinematics(&outside_q, end_link(&arm))
        .unwrap();
    let error = arm
        .test_inverse_kinematics(
            &outside_q,
            end_link(&arm),
            &outside_target,
            InverseKinematicsOptions::default(),
        )
        .unwrap_err();
    assert!(matches!(
        error,
        Error::IkJointLimitViolation {
            joint_index: 0,
            ref joint,
            position,
            lower,
            upper,
        } if joint == "test_joint_1"
            && (position - 0.8).abs() <= 1.0e-12
            && (lower + 0.610865238198015).abs() <= 1.0e-12
            && (upper - 0.610865238198015).abs() <= 1.0e-12
    ));
}

#[test]
fn inverse_kinematics_validates_every_option_and_target_component() {
    let arm = test_arm();
    let target = end_link(&arm);
    let initial_q = JointVector::<4>::zeros();
    let desired = Frame::identity();
    let invalid_options = [
        InverseKinematicsOptions {
            max_iterations: 0,
            ..InverseKinematicsOptions::default()
        },
        InverseKinematicsOptions {
            translation_tolerance: 0.0,
            ..InverseKinematicsOptions::default()
        },
        InverseKinematicsOptions {
            rotation_tolerance: -1.0,
            ..InverseKinematicsOptions::default()
        },
        InverseKinematicsOptions {
            damping: f64::NAN,
            ..InverseKinematicsOptions::default()
        },
        InverseKinematicsOptions {
            max_step_norm: f64::INFINITY,
            ..InverseKinematicsOptions::default()
        },
    ];
    for options in invalid_options {
        assert!(matches!(
            arm.test_inverse_kinematics(&initial_q, target, &desired, options),
            Err(Error::InvalidIkOptions { .. })
        ));
    }

    let mut non_finite_target = desired;
    non_finite_target.translation.vector.y = f64::NAN;
    assert!(matches!(
        arm.test_inverse_kinematics(
            &initial_q,
            target,
            &non_finite_target,
            InverseKinematicsOptions::default(),
        ),
        Err(Error::NonFiniteIkInput {
            input: "target frame"
        })
    ));

    let unreachable = Frame::translation(1.0, 0.0, 0.0);
    let numerically_singular = InverseKinematicsOptions {
        damping: f64::MIN_POSITIVE,
        ..InverseKinematicsOptions::default()
    };
    assert!(matches!(
        arm.test_inverse_kinematics(
            &initial_q,
            arm.root_link(),
            &unreachable,
            numerically_singular,
        ),
        Err(Error::IkNumericalFailure { iteration: 1 })
    ));

    let rotation_only = Frame::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.25),
    );
    let one_iteration = InverseKinematicsOptions {
        max_iterations: 1,
        ..InverseKinematicsOptions::default()
    };
    assert!(matches!(
        arm.test_inverse_kinematics(
            &initial_q,
            arm.root_link(),
            &rotation_only,
            one_iteration,
        ),
        Err(Error::IkNotConverged {
            iterations: 1,
            translation_error,
            rotation_error,
        }) if translation_error <= 1.0e-12 && (rotation_error - 0.25).abs() <= 1.0e-12
    ));

    let overflowing_target = Frame::translation(f64::MAX, f64::MAX, f64::MAX);
    assert!(matches!(
        arm.test_inverse_kinematics(
            &initial_q,
            target,
            &overflowing_target,
            InverseKinematicsOptions::default(),
        ),
        Err(Error::IkNumericalFailure { iteration: 1 })
    ));
}

#[test]
fn velocity_is_jacobian_times_joint_velocity() {
    let arm = test_arm();
    let q = JointVector::<4>::new(PI / 12.0, PI / 3.0, -PI / 2.0, PI / 6.0);
    let qd = q;
    let velocity = arm
        .test_forward_velocity_kinematics(
            &q,
            &qd,
            end_link(&arm),
            &Frame::identity(),
            &Frame::identity(),
        )
        .unwrap();
    assert_relative_eq!(
        velocity.to_vector(),
        arm.test_jacobian(&q, end_link(&arm)).unwrap() * qd,
        epsilon = 1.0e-12
    );
    let expected = nalgebra::SVector::<f64, 6>::new(
        -2.88805923e-17,
        3.20702034e-17,
        2.61799388e-1,
        -3.84628546e-1,
        1.07215119e-2,
        3.15166559e-2,
    );
    assert_relative_eq!(velocity.to_vector(), expected, epsilon = 1.0e-8);

    let base = Isometry3::from_parts(
        Translation3::new(0.3, -0.2, 0.5),
        UnitQuaternion::from_euler_angles(0.2, -0.4, 0.1),
    );
    let tool = Isometry3::translation(0.1, -0.03, 0.2);
    let end = arm.test_forward_kinematics(&q, end_link(&arm)).unwrap();
    let mut tool_jacobian = arm.test_jacobian(&q, end_link(&arm)).unwrap();
    let offset_world = end.rotation * tool.translation.vector;
    for i in 0..4 {
        let angular = tool_jacobian.fixed_view::<3, 1>(0, i).into_owned();
        let shifted =
            tool_jacobian.fixed_view::<3, 1>(3, i).into_owned() + angular.cross(&offset_world);
        tool_jacobian
            .fixed_view_mut::<3, 1>(3, i)
            .copy_from(&shifted);
    }
    let tool_jacobian_velocity = tool_jacobian * qd;
    let expected_with_frames = Twist::new(
        base.rotation * tool_jacobian_velocity.fixed_rows::<3>(0).into_owned(),
        base.rotation * tool_jacobian_velocity.fixed_rows::<3>(3).into_owned(),
    );
    assert_relative_eq!(
        arm.test_forward_velocity_kinematics(&q, &qd, end_link(&arm), &base, &tool)
            .unwrap()
            .to_vector(),
        expected_with_frames.to_vector(),
        epsilon = 1.0e-12
    );
}

#[test]
fn forward_acceleration_matches_finite_difference() {
    let arm = test_arm();
    let q = JointVector::<4>::new(0.2, 1.1, -0.7, 0.4);
    let qd = JointVector::<4>::new(-0.3, 0.5, -0.2, 0.8);
    let epsilon = 1.0e-7;
    let numerical = (arm
        .test_jacobian(&(q + epsilon * qd), end_link(&arm))
        .unwrap()
        - arm
            .test_jacobian(&(q - epsilon * qd), end_link(&arm))
            .unwrap())
        / (2.0 * epsilon);
    let qdd = JointVector::<4>::new(0.7, -0.4, 0.1, 0.3);
    assert_relative_eq!(
        arm.test_forward_acceleration_kinematics(&q, &qd, &qdd, end_link(&arm))
            .unwrap()
            .to_vector(),
        arm.test_jacobian(&q, end_link(&arm)).unwrap() * qdd + numerical * qd,
        epsilon = 2.0e-8
    );

    let mixed_arm = Robot::from_urdf(urdf_path("mixed_arm.urdf")).unwrap();
    let mixed_q = JointVector::<2>::new(0.4, 0.2);
    let mixed_qd = JointVector::<2>::new(-0.3, 0.5);
    let mixed_numerical = (mixed_arm
        .test_jacobian(&(mixed_q + epsilon * mixed_qd), end_link(&mixed_arm))
        .unwrap()
        - mixed_arm
            .test_jacobian(&(mixed_q - epsilon * mixed_qd), end_link(&mixed_arm))
            .unwrap())
        / (2.0 * epsilon);
    let mixed_qdd = JointVector::<2>::new(0.7, -0.4);
    assert_relative_eq!(
        mixed_arm
            .test_forward_acceleration_kinematics(
                &mixed_q,
                &mixed_qd,
                &mixed_qdd,
                end_link(&mixed_arm),
            )
            .unwrap()
            .to_vector(),
        mixed_arm
            .test_jacobian(&mixed_q, end_link(&mixed_arm))
            .unwrap()
            * mixed_qdd
            + mixed_numerical * mixed_qd,
        epsilon = 2.0e-8
    );

    for q in [
        JointVector::<4>::new(0.0, FRAC_PI_2, 0.0, 0.0),
        JointVector::<4>::new(1.5708, 1.0472, -1.0472, 0.5236),
    ] {
        let numerical_jacobian_dot = (arm
            .test_jacobian(&(q + epsilon * q), end_link(&arm))
            .unwrap()
            - arm
                .test_jacobian(&(q - epsilon * q), end_link(&arm))
                .unwrap())
            / (2.0 * epsilon);
        let expected =
            arm.test_jacobian(&q, end_link(&arm)).unwrap() * q + numerical_jacobian_dot * q;
        let acceleration = arm
            .test_forward_acceleration_kinematics(&q, &q, &q, end_link(&arm))
            .unwrap();
        assert_relative_eq!(acceleration.to_vector(), expected, epsilon = 2.0e-8);
    }
}

#[test]
fn fixed_joint_has_constant_pose_and_no_motion_or_generalized_force() {
    let arm = Robot::from_urdf(urdf_path("fixed_arm.urdf")).unwrap();
    let target = end_link(&arm);
    let zero = JointVector::<1>::zeros();
    let arbitrary = JointVector::<1>::new(123.0);
    let expected = Isometry3::from_parts(
        Translation3::new(0.2, 0.1, 0.3),
        UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3),
    );

    assert_relative_eq!(
        arm.test_forward_kinematics(&zero, target).unwrap(),
        expected,
        epsilon = 1.0e-12
    );
    assert_relative_eq!(
        arm.test_forward_kinematics(&arbitrary, target).unwrap(),
        expected,
        epsilon = 1.0e-12
    );
    assert_relative_eq!(
        arm.test_jacobian(&arbitrary, target).unwrap(),
        Jacobian::<1>::zeros(),
        epsilon = 1.0e-12
    );
    assert_relative_eq!(
        arm.test_forward_acceleration_kinematics(&arbitrary, &arbitrary, &arbitrary, target)
            .unwrap()
            .to_vector(),
        Twist::zeros().to_vector(),
        epsilon = 1.0e-12
    );

    let torque = arm
        .test_inverse_dynamics(&arbitrary, &arbitrary, &arbitrary, &[])
        .unwrap();
    assert_relative_eq!(torque, zero, epsilon = 1.0e-12);
}

#[test]
fn two_link_gravity_matches_closed_form_oracle() {
    let arm = Robot::from_urdf(urdf_path("gravity_arm.urdf")).unwrap();
    let q = JointVector::<2>::new(FRAC_PI_2, FRAC_PI_2);
    let tau = arm.test_gravity(&q, &Frame::identity(), &[]).unwrap();
    assert_relative_eq!(
        tau,
        JointVector::<2>::new(0.0, -0.5 * STANDARD_GRAVITY),
        epsilon = 2.0e-12
    );

    let vertical_base = Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_euler_angles(FRAC_PI_2, 0.0, 0.0),
    );
    let tau = arm
        .test_gravity(&JointVector::<2>::zeros(), &vertical_base, &[])
        .unwrap();
    assert_relative_eq!(
        tau,
        JointVector::<2>::new(STANDARD_GRAVITY, 0.0),
        epsilon = 2.0e-12
    );
}

#[test]
fn single_revolute_pendulum_matches_closed_form_oracle() {
    let arm = Robot::from_urdf(urdf_path("single_revolute.urdf")).unwrap();
    let target = end_link(&arm);
    let mass = 2.0;
    let center_distance = 0.3;
    let center_inertia = 0.05;
    let joint_inertia = center_inertia + mass * center_distance * center_distance;

    for (q, qd, qdd) in [
        (PI / 3.0, 0.8, 0.7),
        (-PI / 4.0, -0.6, -0.2),
        (0.0, 0.0, 0.0),
    ] {
        let position = JointVector::<1>::new(q);
        let velocity = JointVector::<1>::new(qd);
        let acceleration = JointVector::<1>::new(qdd);

        assert_relative_eq!(
            arm.test_forward_kinematics(&position, target).unwrap(),
            Frame::rotation(Vector3::new(0.0, q, 0.0)),
            epsilon = 2.0e-12
        );
        assert_relative_eq!(
            arm.test_jacobian(&position, target).unwrap(),
            Jacobian::<1>::from_column_slice(&[0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            epsilon = 2.0e-12
        );
        assert_relative_eq!(
            arm.test_forward_velocity_kinematics(
                &position,
                &velocity,
                target,
                &Frame::identity(),
                &Frame::identity(),
            )
            .unwrap()
            .to_vector(),
            SVector::<f64, 6>::new(0.0, qd, 0.0, 0.0, 0.0, 0.0),
            epsilon = 2.0e-12
        );
        assert_relative_eq!(
            arm.test_forward_acceleration_kinematics(&position, &velocity, &acceleration, target,)
                .unwrap()
                .to_vector(),
            SVector::<f64, 6>::new(0.0, qdd, 0.0, 0.0, 0.0, 0.0),
            epsilon = 2.0e-12
        );

        let gravity_torque = -mass * STANDARD_GRAVITY * center_distance * q.cos();
        assert_relative_eq!(
            arm.test_gravity(&position, &Frame::identity(), &[])
                .unwrap(),
            JointVector::<1>::new(gravity_torque),
            epsilon = 2.0e-12
        );
        assert_relative_eq!(
            arm.test_inverse_dynamics(&position, &velocity, &acceleration, &[],)
                .unwrap(),
            JointVector::<1>::new(gravity_torque + joint_inertia * qdd),
            epsilon = 2.0e-12
        );
    }
}

#[test]
fn revolute_prismatic_arm_matches_closed_form_oracle() {
    let arm = Robot::from_urdf(urdf_path("mixed_arm.urdf")).unwrap();
    let target = end_link(&arm);
    let q = JointVector::<2>::new(PI / 6.0, 0.25);
    let qd = JointVector::<2>::new(0.4, -0.3);
    let qdd = JointVector::<2>::new(-0.2, 0.7);
    let (sin_theta, cos_theta) = q[0].sin_cos();
    let target_radius = 1.0 + q[1];

    let expected_frame = Frame::from_parts(
        Translation3::new(target_radius * cos_theta, target_radius * sin_theta, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), q[0]),
    );
    assert_relative_eq!(
        arm.test_forward_kinematics(&q, target).unwrap(),
        expected_frame,
        epsilon = 2.0e-12
    );

    let expected_jacobian = Jacobian::<2>::from_columns(&[
        SVector::<f64, 6>::new(
            0.0,
            0.0,
            1.0,
            -target_radius * sin_theta,
            target_radius * cos_theta,
            0.0,
        ),
        SVector::<f64, 6>::new(0.0, 0.0, 0.0, cos_theta, sin_theta, 0.0),
    ]);
    assert_relative_eq!(
        arm.test_jacobian(&q, target).unwrap(),
        expected_jacobian,
        epsilon = 2.0e-12
    );

    let expected_velocity = Twist::new(
        Vector3::new(0.0, 0.0, qd[0]),
        Vector3::new(
            qd[1] * cos_theta - target_radius * qd[0] * sin_theta,
            qd[1] * sin_theta + target_radius * qd[0] * cos_theta,
            0.0,
        ),
    );
    assert_relative_eq!(
        arm.test_forward_velocity_kinematics(
            &q,
            &qd,
            target,
            &Frame::identity(),
            &Frame::identity(),
        )
        .unwrap()
        .to_vector(),
        expected_velocity.to_vector(),
        epsilon = 2.0e-12
    );

    let radial_acceleration = qdd[1] - target_radius * qd[0].powi(2);
    let tangential_acceleration = target_radius * qdd[0] + 2.0 * qd[0] * qd[1];
    let expected_acceleration = Twist::new(
        Vector3::new(0.0, 0.0, qdd[0]),
        Vector3::new(
            radial_acceleration * cos_theta - tangential_acceleration * sin_theta,
            radial_acceleration * sin_theta + tangential_acceleration * cos_theta,
            0.0,
        ),
    );
    assert_relative_eq!(
        arm.test_forward_acceleration_kinematics(&q, &qd, &qdd, target)
            .unwrap()
            .to_vector(),
        expected_acceleration.to_vector(),
        epsilon = 2.0e-12
    );

    assert_relative_eq!(
        arm.test_gravity(&q, &Frame::identity(), &[]).unwrap(),
        JointVector::<2>::zeros(),
        epsilon = 2.0e-12
    );

    // The first link COM is 0.2 m from the revolute joint. The second link COM
    // is 0.1 m beyond the prismatic joint, so its radius is 1.1 + q[1]. Both
    // links have unit mass and Izz = 0.01 kg m^2.
    let second_com_radius = 1.1 + q[1];
    let rotational_inertia = 0.01 + 0.2_f64.powi(2) + 0.01 + second_com_radius.powi(2);
    let expected_torque = JointVector::<2>::new(
        rotational_inertia * qdd[0] + 2.0 * second_com_radius * qd[0] * qd[1],
        qdd[1] - second_com_radius * qd[0].powi(2),
    );
    assert_relative_eq!(
        arm.test_inverse_dynamics(&q, &qd, &qdd, &[],).unwrap(),
        expected_torque,
        epsilon = 2.0e-12
    );
}

#[test]
fn inverse_dynamics_matches_test_arm_numeric_reference() {
    let arm = test_arm();
    let set_q = JointVector::<4>::new(1.5708, 1.0472, -1.0472, 0.5236);
    let zero = JointVector::<4>::zeros();
    let random = JointVector::<4>::new(-0.2, 0.5, -0.3, 0.8);
    let cases = [
        (
            set_q,
            zero,
            zero,
            JointVector::<4>::new(0.0, 38.8143, 18.4362, 0.0607),
            1.0e-4,
        ),
        (
            zero,
            random,
            zero,
            JointVector::<4>::new(-0.1404, 59.2065, 18.4470, 0.0716),
            1.0e-2,
        ),
        (
            zero,
            zero,
            random,
            JointVector::<4>::new(-0.6787, 60.3962, 18.8600, 0.0733),
            1.0e-2,
        ),
        (
            set_q,
            zero,
            random,
            JointVector::<4>::new(-0.5904, 39.8839, 18.6987, 0.0623),
            1.0e-2,
        ),
        (
            zero,
            random,
            random,
            JointVector::<4>::new(-0.8191, 60.3961, 18.8599, 0.0733),
            1.0e-2,
        ),
        (
            set_q,
            random,
            random,
            JointVector::<4>::new(-0.4478, 39.8180, 18.5676, 0.0621),
            1.0e-2,
        ),
    ];
    for (q, qd, qdd, expected, epsilon) in cases {
        let tau = arm.test_inverse_dynamics(&q, &qd, &qdd, &[]).unwrap();
        assert_relative_eq!(tau, expected, epsilon = epsilon);
    }
}

#[test]
fn prismatic_inverse_dynamics_projects_linear_inertia_onto_its_axis() {
    let arm = Robot::from_urdf(urdf_path("mixed_arm.urdf")).unwrap();
    let zero = JointVector::<2>::zeros();
    let gravity = arm.test_gravity(&zero, &Frame::identity(), &[]).unwrap();
    let accelerated = arm
        .test_inverse_dynamics(&zero, &zero, &JointVector::<2>::new(0.0, 1.0), &[])
        .unwrap();

    assert_relative_eq!(
        accelerated - gravity,
        JointVector::<2>::new(0.0, 1.0),
        epsilon = 1.0e-12
    );
}

#[test]
fn tree_external_loads_are_isolated_and_add_linearly() {
    let arm = tree_arm();
    let q = JointVector::<7>::from_row_slice(&[0.2, -0.3, 0.4, -0.5, 0.6, -0.7, 0.8]);
    let left = Load {
        link: arm.link("left_tool").unwrap(),
        wrench: Wrench::new(Vector3::new(0.3, -0.2, 0.4), Vector3::new(1.0, 0.5, -0.7)),
    };
    let right = Load {
        link: arm.link("right_tool").unwrap(),
        wrench: Wrench::new(Vector3::new(-0.4, 0.1, 0.2), Vector3::new(-0.6, 0.8, 0.3)),
    };

    let baseline = arm.test_gravity(&q, &Frame::identity(), &[]).unwrap();
    let left_only = arm.test_gravity(&q, &Frame::identity(), &[left]).unwrap();
    let right_only = arm.test_gravity(&q, &Frame::identity(), &[right]).unwrap();
    let both = arm
        .test_gravity(&q, &Frame::identity(), &[left, right])
        .unwrap();

    for right_joint in [2, 4, 6] {
        assert_abs_diff_eq!(
            left_only[right_joint],
            baseline[right_joint],
            epsilon = 1.0e-12
        );
    }
    for left_joint in [1, 3, 5] {
        assert_abs_diff_eq!(
            right_only[left_joint],
            baseline[left_joint],
            epsilon = 1.0e-12
        );
    }
    assert!((left_only - baseline).norm() > 1.0e-6);
    assert!((right_only - baseline).norm() > 1.0e-6);
    for (external, with_load) in [(left, left_only), (right, right_only)] {
        let frame = arm.test_forward_kinematics(&q, external.link).unwrap();
        let torque = frame.rotation * external.wrench.torque;
        let force = frame.rotation * external.wrench.force;
        let wrench_in_base =
            SVector::<f64, 6>::from_iterator(torque.iter().chain(force.iter()).copied());
        let expected = arm.test_jacobian(&q, external.link).unwrap().transpose() * wrench_in_base;
        assert_relative_eq!(with_load - baseline, expected, epsilon = 2.0e-12);
    }
    assert_relative_eq!(both, left_only + right_only - baseline, epsilon = 2.0e-12);
}
