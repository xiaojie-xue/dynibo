use std::collections::{HashMap, HashSet};

use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};
use urdf_rs::{JointType as UrdfJointType, Pose, Robot};

use crate::{Error, Joint, JointType, Link, Result};

/// Topologically ordered representation produced while importing a URDF tree.
pub(crate) struct TreeModel {
    /// Joints ordered immediately before their corresponding child links.
    pub joints: Vec<Joint>,
    /// Links in root-first topological order.
    pub links: Vec<Link>,
    /// Parent link index for each joint. Joint `i` always connects this parent
    /// to child link `i + 1` in the topologically ordered arrays.
    pub joint_parents: Vec<usize>,
    /// Indices of links that have no child joints.
    pub leaf_links: Vec<usize>,
}

/// Converts a URDF translation and roll-pitch-yaw pose to an isometry.
pub(crate) fn pose_to_frame(pose: &Pose) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose.xyz[0], pose.xyz[1], pose.xyz[2]),
        UnitQuaternion::from_euler_angles(pose.rpy[0], pose.rpy[1], pose.rpy[2]),
    )
}

/// Validates a parsed URDF and converts it to a topologically ordered tree.
pub(crate) fn tree_model(robot: &Robot) -> Result<TreeModel> {
    let link_names: HashSet<&str> = robot.links.iter().map(|link| link.name.as_str()).collect();
    if link_names.len() != robot.links.len() {
        return Err(Error::InvalidModel("link names must be unique".to_owned()));
    }
    let joint_names: HashSet<&str> = robot
        .joints
        .iter()
        .map(|joint| joint.name.as_str())
        .collect();
    if joint_names.len() != robot.joints.len() {
        return Err(Error::InvalidModel("joint names must be unique".to_owned()));
    }

    let children: HashSet<&str> = robot
        .joints
        .iter()
        .map(|joint| joint.child.link.as_str())
        .collect();
    let roots: Vec<&str> = robot
        .links
        .iter()
        .map(|link| link.name.as_str())
        .filter(|name| !children.contains(name))
        .collect();
    if roots.len() != 1 {
        return Err(Error::InvalidModel(format!(
            "expected one root link, found {}",
            roots.len()
        )));
    }

    let mut joints_by_parent: HashMap<&str, Vec<&urdf_rs::Joint>> = HashMap::new();
    for joint in &robot.joints {
        joints_by_parent
            .entry(joint.parent.link.as_str())
            .or_default()
            .push(joint);
    }
    let links_by_name: HashMap<&str, &urdf_rs::Link> = robot
        .links
        .iter()
        .map(|link| (link.name.as_str(), link))
        .collect();

    let mut joints = Vec::with_capacity(robot.joints.len());
    let mut links = Vec::with_capacity(robot.links.len());
    let mut joint_parents = Vec::with_capacity(robot.joints.len());
    let mut topological_names = Vec::with_capacity(robot.links.len());
    let mut discovered: HashMap<&str, usize> = HashMap::with_capacity(robot.links.len());
    let mut has_children = Vec::with_capacity(robot.links.len());

    let root = roots[0];
    links.push(robot_link(links_by_name[root])?);
    topological_names.push(root);
    discovered.insert(root, 0);
    has_children.push(false);

    let mut parent_index = 0;
    while parent_index < topological_names.len() {
        let parent_name = topological_names[parent_index];
        if let Some(child_joints) = joints_by_parent.get(parent_name) {
            has_children[parent_index] = true;
            for joint in child_joints {
                let child_name = joint.child.link.as_str();
                let child = links_by_name.get(child_name).ok_or_else(|| {
                    Error::InvalidModel(format!(
                        "joint {} references a missing child link",
                        joint.name
                    ))
                })?;
                if let Some(&first_index) = discovered.get(child_name) {
                    return Err(Error::InvalidModel(format!(
                        "link {child_name} is reached more than once (first index {first_index})"
                    )));
                }

                let child_index = links.len();
                discovered.insert(child_name, child_index);
                topological_names.push(child_name);
                links.push(robot_link(child)?);
                has_children.push(false);
                joints.push(robot_joint(joint)?);
                joint_parents.push(parent_index);
            }
        }
        parent_index += 1;
    }

    if joints.len() != robot.joints.len() || links.len() != robot.links.len() {
        return Err(Error::InvalidModel(
            "joint graph is disconnected or cyclic".to_owned(),
        ));
    }
    let leaf_links = has_children
        .iter()
        .enumerate()
        .filter_map(|(index, &has_children)| (!has_children).then_some(index))
        .collect();
    Ok(TreeModel {
        joints,
        links,
        joint_parents,
        leaf_links,
    })
}

/// Converts one supported URDF joint into the crate's joint representation.
fn robot_joint(joint: &urdf_rs::Joint) -> Result<Joint> {
    let joint_type = match joint.joint_type {
        UrdfJointType::Revolute | UrdfJointType::Continuous => JointType::Revolute,
        UrdfJointType::Prismatic => JointType::Prismatic,
        UrdfJointType::Fixed => JointType::Fixed,
        _ => {
            return Err(Error::UnsupportedJointType {
                joint: joint.name.clone(),
                joint_type: format!("{:?}", joint.joint_type).to_ascii_lowercase(),
            });
        }
    };
    let (lower_limit, upper_limit) = if joint.joint_type == UrdfJointType::Continuous {
        (f64::NEG_INFINITY, f64::INFINITY)
    } else {
        (joint.limit.lower, joint.limit.upper)
    };
    Joint::new_named(
        joint.name.clone(),
        joint_type,
        pose_to_frame(&joint.origin),
        Vector3::new(joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2]),
        lower_limit,
        upper_limit,
        joint.limit.velocity,
    )
}

/// Converts one URDF link and its inertial block into a [`Link`].
fn robot_link(link: &urdf_rs::Link) -> Result<Link> {
    let inertial = &link.inertial;
    if !inertial.mass.value.is_finite() || inertial.mass.value < 0.0 {
        return Err(Error::InvalidModel(format!(
            "link {} mass must be finite and non-negative",
            link.name
        )));
    }
    if !inertial
        .origin
        .xyz
        .iter()
        .chain(inertial.origin.rpy.iter())
        .all(|value| value.is_finite())
    {
        return Err(Error::InvalidModel(format!(
            "link {} inertial origin must contain only finite values",
            link.name
        )));
    }
    // URDF stores the entries of the symmetric inertia tensor directly.
    let inertia_in_inertial_frame = Matrix3::new(
        inertial.inertia.ixx,
        inertial.inertia.ixy,
        inertial.inertia.ixz,
        inertial.inertia.ixy,
        inertial.inertia.iyy,
        inertial.inertia.iyz,
        inertial.inertia.ixz,
        inertial.inertia.iyz,
        inertial.inertia.izz,
    );
    if !inertia_in_inertial_frame
        .iter()
        .all(|value| value.is_finite())
    {
        return Err(Error::InvalidModel(format!(
            "link {} inertia must contain only finite values",
            link.name
        )));
    }
    let principal_moments = inertia_in_inertial_frame.symmetric_eigen().eigenvalues;
    if principal_moments.iter().any(|value| *value < 0.0) {
        return Err(Error::InvalidModel(format!(
            "link {} inertia must be positive semi-definite",
            link.name
        )));
    }
    let inertial_rotation = UnitQuaternion::from_euler_angles(
        inertial.origin.rpy[0],
        inertial.origin.rpy[1],
        inertial.origin.rpy[2],
    )
    .to_rotation_matrix();
    let inertia = inertial_rotation.matrix()
        * inertia_in_inertial_frame
        * inertial_rotation.matrix().transpose();
    Ok(Link::new(
        link.name.clone(),
        inertial.mass.value,
        Vector3::new(
            inertial.origin.xyz[0],
            inertial.origin.xyz[1],
            inertial.origin.xyz[2],
        ),
        inertia,
    ))
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::Matrix3;
    use urdf_rs::{JointType as UrdfJointType, read_from_string};

    use super::tree_model;
    use crate::{Error, JointType};

    const CHAIN: &str = r#"
        <robot name="chain">
          <link name="base"/>
          <link name="middle"/>
          <link name="tool"/>
          <joint name="shoulder" type="revolute">
            <parent link="base"/><child link="middle"/><axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="2"/>
          </joint>
          <joint name="wrist" type="fixed">
            <parent link="middle"/><child link="tool"/>
          </joint>
        </robot>
    "#;

    fn chain() -> urdf_rs::Robot {
        read_from_string(CHAIN).expect("test URDF must parse")
    }

    fn invalid_model(robot: &urdf_rs::Robot, expected: &str) {
        let error = tree_model(robot).err().expect("model must be rejected");
        assert!(
            matches!(error, Error::InvalidModel(ref message) if message.contains(expected)),
            "unexpected error: {error}"
        );
    }

    #[test]
    fn rejects_duplicate_link_and_joint_names() {
        let mut robot = chain();
        robot.links[1].name = robot.links[0].name.clone();
        invalid_model(&robot, "link names must be unique");

        let mut robot = chain();
        robot.joints[1].name = robot.joints[0].name.clone();
        invalid_model(&robot, "joint names must be unique");
    }

    #[test]
    fn rejects_invalid_root_counts_and_missing_children() {
        let mut robot = chain();
        let mut detached = robot.links[0].clone();
        detached.name = "detached".to_owned();
        robot.links.push(detached);
        invalid_model(&robot, "expected one root link, found 2");

        let mut robot = chain();
        robot.joints.push(robot.joints[0].clone());
        robot.joints[2].name = "cycle".to_owned();
        robot.joints[2].parent.link = "tool".to_owned();
        robot.joints[2].child.link = "base".to_owned();
        invalid_model(&robot, "expected one root link, found 0");

        let mut robot = chain();
        robot.links.pop();
        invalid_model(&robot, "references a missing child link");
    }

    #[test]
    fn rejects_multiply_reached_and_disconnected_links() {
        let mut robot = chain();
        let mut duplicate_edge = robot.joints[1].clone();
        duplicate_edge.name = "second_path".to_owned();
        duplicate_edge.parent.link = "base".to_owned();
        robot.joints.push(duplicate_edge);
        invalid_model(&robot, "is reached more than once");

        let mut robot = chain();
        let mut detached_a = robot.links[0].clone();
        detached_a.name = "detached_a".to_owned();
        let mut detached_b = robot.links[0].clone();
        detached_b.name = "detached_b".to_owned();
        robot.links.extend([detached_a, detached_b]);

        let mut forward = robot.joints[1].clone();
        forward.name = "detached_forward".to_owned();
        forward.parent.link = "detached_a".to_owned();
        forward.child.link = "detached_b".to_owned();
        let mut backward = forward.clone();
        backward.name = "detached_backward".to_owned();
        backward.parent.link = "detached_b".to_owned();
        backward.child.link = "detached_a".to_owned();
        robot.joints.extend([forward, backward]);
        invalid_model(&robot, "joint graph is disconnected or cyclic");
    }

    #[test]
    fn rejects_unsupported_joints_and_accepts_continuous_joints() {
        let mut robot = chain();
        robot.joints[0].joint_type = UrdfJointType::Planar;
        assert!(matches!(
            tree_model(&robot),
            Err(Error::UnsupportedJointType {
                ref joint,
                ref joint_type,
            }) if joint == "shoulder" && joint_type == "planar"
        ));

        let mut robot = chain();
        robot.joints[0].joint_type = UrdfJointType::Continuous;
        let model = tree_model(&robot).expect("continuous joints are supported");
        assert_eq!(model.joints[0].joint_type(), JointType::Revolute);
        assert_eq!(model.joints[0].lower_limit(), f64::NEG_INFINITY);
        assert_eq!(model.joints[0].upper_limit(), f64::INFINITY);
    }

    #[test]
    fn rejects_non_physical_link_inertial_properties() {
        let mut robot = chain();
        robot.links[1].inertial.mass.value = -1.0;
        invalid_model(&robot, "mass must be finite and non-negative");

        let mut robot = chain();
        robot.links[1].inertial.mass.value = f64::NAN;
        invalid_model(&robot, "mass must be finite and non-negative");

        let mut robot = chain();
        robot.links[1].inertial.origin.xyz[0] = f64::INFINITY;
        invalid_model(&robot, "inertial origin must contain only finite values");

        let mut robot = chain();
        robot.links[1].inertial.inertia.ixx = f64::NAN;
        invalid_model(&robot, "inertia must contain only finite values");

        let mut robot = chain();
        robot.links[1].inertial.inertia.ixx = -1.0;
        invalid_model(&robot, "inertia must be positive semi-definite");
    }

    #[test]
    fn rotates_urdf_inertia_from_the_inertial_frame_into_the_link_frame() {
        let robot = read_from_string(
            r#"
            <robot name="rotated_inertia">
              <link name="base"/>
              <link name="body">
                <inertial>
                  <origin xyz="0 0 0" rpy="0 0 1.5707963267948966"/>
                  <mass value="1"/>
                  <inertia ixx="1" ixy="0" ixz="0" iyy="2" iyz="0" izz="3"/>
                </inertial>
              </link>
              <joint name="mount" type="fixed">
                <parent link="base"/><child link="body"/>
              </joint>
            </robot>
            "#,
        )
        .unwrap();
        let model = tree_model(&robot).unwrap();
        assert_relative_eq!(
            model.links[1].inertia(),
            &Matrix3::from_diagonal(&nalgebra::Vector3::new(2.0, 1.0, 3.0)),
            epsilon = 2.0e-12
        );
    }
}
