use std::collections::{HashMap, HashSet};

use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};
use urdf_rs::{JointType, Pose, Robot};

use crate::{Error, JointKind, JointLimit, Result, RobotJoint, RobotLink};

pub(crate) struct TreeModel {
    pub joints: Vec<RobotJoint>,
    pub links: Vec<RobotLink>,
    /// Parent link index for each joint. Joint `i` always connects this parent
    /// to child link `i + 1` in the topologically ordered arrays.
    pub joint_parents: Vec<usize>,
    pub leaf_links: Vec<usize>,
}

pub(crate) fn pose_to_frame(pose: &Pose) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose.xyz[0], pose.xyz[1], pose.xyz[2]),
        UnitQuaternion::from_euler_angles(pose.rpy[0], pose.rpy[1], pose.rpy[2]),
    )
}

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
    links.push(robot_link(links_by_name[root]));
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
                links.push(robot_link(child));
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

fn robot_joint(joint: &urdf_rs::Joint) -> Result<RobotJoint> {
    let kind = match joint.joint_type {
        JointType::Revolute | JointType::Continuous => JointKind::Revolute,
        JointType::Prismatic => JointKind::Prismatic,
        JointType::Fixed => JointKind::Fixed,
        _ => return Err(Error::UnsupportedJoint(joint.name.clone())),
    };
    let limit = if joint.joint_type == JointType::Continuous {
        JointLimit {
            lower: f64::NEG_INFINITY,
            upper: f64::INFINITY,
            velocity: joint.limit.velocity,
        }
    } else {
        JointLimit {
            lower: joint.limit.lower,
            upper: joint.limit.upper,
            velocity: joint.limit.velocity,
        }
    };
    RobotJoint::new_named(
        joint.name.clone(),
        kind,
        pose_to_frame(&joint.origin),
        Vector3::new(joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2]),
        limit,
    )
}

fn robot_link(link: &urdf_rs::Link) -> RobotLink {
    let inertial = &link.inertial;
    // Preserve the compatibility convention: URDF products of inertia are
    // stored with a negative sign in RobotLink.
    let inertia = Matrix3::new(
        inertial.inertia.ixx,
        -inertial.inertia.ixy,
        -inertial.inertia.ixz,
        -inertial.inertia.ixy,
        inertial.inertia.iyy,
        -inertial.inertia.iyz,
        -inertial.inertia.ixz,
        -inertial.inertia.iyz,
        inertial.inertia.izz,
    );
    RobotLink::new(
        link.name.clone(),
        inertial.mass.value,
        Vector3::new(
            inertial.origin.xyz[0],
            inertial.origin.xyz[1],
            inertial.origin.xyz[2],
        ),
        inertia,
    )
}
