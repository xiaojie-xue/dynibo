use std::{
    collections::{HashMap, HashSet},
    path::Path,
};

use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};
use urdf_rs::{JointType as UrdfJointType, Pose, Robot};

use super::{Joint, JointType, Link};
use crate::{Error, Result};

/// Format-neutral, topologically ordered data used to construct a [`crate::Robot`].
pub(crate) struct Tree {
    /// Model name supplied by the source description.
    pub name: String,
    /// Joints ordered immediately before their corresponding child links.
    pub joints: Vec<Joint>,
    /// Links in root-first topological order.
    pub links: Vec<Link>,
    /// Parent link index for each joint. Joint `i` always connects this parent
    /// to child link `i + 1` in the topologically ordered arrays.
    pub parent_link_indices: Vec<usize>,
}

/// Loads, validates, and converts a URDF file into the format-neutral model.
pub(crate) fn load_urdf(path: impl AsRef<Path>) -> Result<Tree> {
    let urdf = urdf_rs::read_file(path)?;
    convert_urdf(&urdf)
}

/// Converts a URDF translation and roll-pitch-yaw pose to an isometry.
fn pose_to_frame(pose: &Pose) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose.xyz[0], pose.xyz[1], pose.xyz[2]),
        UnitQuaternion::from_euler_angles(pose.rpy[0], pose.rpy[1], pose.rpy[2]),
    )
}

/// Validates a parsed URDF and converts it to a topologically ordered tree.
fn convert_urdf(robot: &Robot) -> Result<Tree> {
    let mut link_indices = HashMap::with_capacity(robot.links.len());
    for (index, link) in robot.links.iter().enumerate() {
        if link_indices.insert(link.name.as_str(), index).is_some() {
            return Err(Error::InvalidModel("link names must be unique".to_owned()));
        }
    }

    let mut joint_names = HashSet::with_capacity(robot.joints.len());
    let mut has_parent = vec![false; robot.links.len()];
    let mut children_by_parent = vec![Vec::<(usize, usize)>::new(); robot.links.len()];
    for (joint_index, joint) in robot.joints.iter().enumerate() {
        if !joint_names.insert(joint.name.as_str()) {
            return Err(Error::InvalidModel("joint names must be unique".to_owned()));
        }
        let parent_index = *link_indices
            .get(joint.parent.link.as_str())
            .ok_or_else(|| {
                Error::InvalidModel(format!(
                    "joint {} references a missing parent link",
                    joint.name
                ))
            })?;
        let child_index = *link_indices.get(joint.child.link.as_str()).ok_or_else(|| {
            Error::InvalidModel(format!(
                "joint {} references a missing child link",
                joint.name
            ))
        })?;
        if has_parent[child_index] {
            return Err(Error::InvalidModel(format!(
                "link {} is reached more than once",
                joint.child.link
            )));
        }
        has_parent[child_index] = true;
        children_by_parent[parent_index].push((child_index, joint_index));
    }

    let mut root_source_index = None;
    let mut root_count = 0;
    for (index, &has_parent) in has_parent.iter().enumerate() {
        if !has_parent {
            root_source_index = Some(index);
            root_count += 1;
        }
    }
    if root_count != 1 {
        return Err(Error::InvalidModel(format!(
            "expected one root link, found {root_count}"
        )));
    }
    let root_source_index = root_source_index.expect("one root was found");

    let mut joints = Vec::with_capacity(robot.joints.len());
    let mut links = Vec::with_capacity(robot.links.len());
    let mut parent_link_indices = Vec::with_capacity(robot.joints.len());
    let mut topological_source_indices = Vec::with_capacity(robot.links.len());
    let mut discovered = vec![None; robot.links.len()];

    links.push(robot_link(&robot.links[root_source_index])?);
    topological_source_indices.push(root_source_index);
    discovered[root_source_index] = Some(0);

    let mut parent_index = 0;
    while parent_index < topological_source_indices.len() {
        let parent_source_index = topological_source_indices[parent_index];
        for &(child_source_index, joint_source_index) in &children_by_parent[parent_source_index] {
            if let Some(first_index) = discovered[child_source_index] {
                return Err(Error::InvalidModel(format!(
                    "link {} is reached more than once (first index {first_index})",
                    robot.links[child_source_index].name
                )));
            }

            let child_index = links.len();
            discovered[child_source_index] = Some(child_index);
            topological_source_indices.push(child_source_index);
            links.push(robot_link(&robot.links[child_source_index])?);
            joints.push(robot_joint(&robot.joints[joint_source_index])?);
            parent_link_indices.push(parent_index);
        }
        parent_index += 1;
    }

    if joints.len() != robot.joints.len() || links.len() != robot.links.len() {
        return Err(Error::InvalidModel(
            "joint graph is disconnected or cyclic".to_owned(),
        ));
    }
    Ok(Tree {
        name: robot.name.clone(),
        joints,
        links,
        parent_link_indices,
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
    Joint::new(
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

    use super::convert_urdf;
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
        let error = convert_urdf(robot).err().expect("model must be rejected");
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
    fn rejects_invalid_root_counts_and_missing_link_references() {
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

        let mut robot = chain();
        robot.joints[0].parent.link = "missing".to_owned();
        invalid_model(&robot, "references a missing parent link");
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
            convert_urdf(&robot),
            Err(Error::UnsupportedJointType {
                ref joint,
                ref joint_type,
            }) if joint == "shoulder" && joint_type == "planar"
        ));

        let mut robot = chain();
        robot.joints[0].joint_type = UrdfJointType::Continuous;
        let tree = convert_urdf(&robot).expect("continuous joints are supported");
        assert_eq!(tree.joints[0].joint_type(), JointType::Revolute);
        assert_eq!(tree.joints[0].lower_limit(), f64::NEG_INFINITY);
        assert_eq!(tree.joints[0].upper_limit(), f64::INFINITY);
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
        let tree = convert_urdf(&robot).unwrap();
        assert_relative_eq!(
            tree.links[1].inertia(),
            &Matrix3::from_diagonal(&nalgebra::Vector3::new(2.0, 1.0, 3.0)),
            epsilon = 2.0e-12
        );
    }
}
