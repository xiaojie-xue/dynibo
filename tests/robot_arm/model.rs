use super::*;

#[test]
fn loaded_link_preserves_its_inertial_parameters() {
    let arm = test_arm();
    let link = arm.link_id("test_link_2").unwrap();

    assert_eq!(arm.link_name(link).unwrap(), "test_link_2");
    assert_eq!(arm.link_mass(link).unwrap(), 7.016);
    assert_abs_diff_eq!(arm.link_center_of_mass(link).unwrap().z, 0.129994);
    assert_eq!(
        arm.link_inertia(link).unwrap(),
        Matrix3::new(
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
}

#[test]
fn urdf_rs_loads_test_arm_and_checks_calculation_size() {
    let mut arm = test_arm();
    assert_eq!(arm.name(), "test_arm");
    assert_eq!(arm.link_count(), 5);
    assert_eq!(arm.link_name(arm.root_link_id()).unwrap(), "test_base_link");
    assert_eq!(
        arm.link_name(arm.link_id_at(1).unwrap()).unwrap(),
        "test_link_1"
    );
    assert_eq!(arm.joint_count(), 4);
    assert_eq!(arm.joint_name(0).unwrap(), "test_joint_1");
    assert_eq!(arm.link_count(), 5);
    assert_eq!(arm.joint_count(), 4);
    assert_eq!(
        arm.link_name(arm.link_id("test_link_1").unwrap()).unwrap(),
        "test_link_1"
    );
    assert!(matches!(
        arm.link_id("missing_link"),
        Err(Error::UnknownLink { name }) if name == "missing_link"
    ));
    let link_id = arm.link_id("test_link_1").unwrap();
    arm.forward_kinematics(&[0.0; 4], link_id)
        .expect("a model-owned link ID remains valid");
    let other_arm = test_arm();
    let other_link_id = other_arm.link_id("test_link_1").unwrap();
    assert!(matches!(
        arm.forward_kinematics(&[0.0; 4], other_link_id),
        Err(Error::InvalidLinkId)
    ));
    assert_abs_diff_eq!(arm.link_mass(arm.link_id_at(2).unwrap()).unwrap(), 7.016);

    let wrong_size = arm.forward_kinematics(&[0.0; 3], link_id).unwrap_err();
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
fn fixed_joint_has_constant_pose_and_no_motion_or_generalized_force() {
    let mut arm = Robot::from_urdf(urdf_path("fixed_arm.urdf")).unwrap();
    assert_eq!(arm.joint_count(), 0);
    let target = end_link(&arm);
    let zero = JointVector::<0>::zeros();
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
        arm.test_jacobian(&zero, target).unwrap(),
        Jacobian::<0>::zeros(),
        epsilon = 1.0e-12
    );
    assert_relative_eq!(
        arm.test_forward_velocity_kinematics(
            &zero,
            &zero,
            target,
            &Frame::identity(),
            &Frame::identity(),
        )
        .unwrap()
        .to_vector(),
        Twist::zeros().to_vector(),
        epsilon = 1.0e-12
    );
    assert_relative_eq!(
        arm.test_forward_acceleration_kinematics(&zero, &zero, &zero, target)
            .unwrap()
            .to_vector(),
        Twist::zeros().to_vector(),
        epsilon = 1.0e-12
    );

    let torque = arm.test_inverse_dynamics(&zero, &zero, &zero, &[]).unwrap();
    assert_relative_eq!(torque, zero, epsilon = 1.0e-12);
}
