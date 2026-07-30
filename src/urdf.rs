use std::collections::{HashMap, HashSet};

use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};
use urdf_rs::{JointType, Pose, Robot};

use crate::{Error, JointKind, JointLimit, Result, RobotJoint, RobotLink};

pub(crate) struct SerialModel {
    pub joints: Vec<RobotJoint>,
    pub links: Vec<RobotLink>,
}

pub(crate) fn pose_to_frame(pose: &Pose) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose.xyz[0], pose.xyz[1], pose.xyz[2]),
        UnitQuaternion::from_euler_angles(pose.rpy[0], pose.rpy[1], pose.rpy[2]),
    )
}

pub(crate) fn serial_model(robot: &Robot) -> Result<SerialModel> {
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
    if let Some((parent, joints)) = joints_by_parent.iter().find(|(_, joints)| joints.len() > 1) {
        return Err(Error::InvalidModel(format!(
            "RobotArm is serial but link {parent} has {} child joints",
            joints.len()
        )));
    }

    let links_by_name: HashMap<&str, &urdf_rs::Link> = robot
        .links
        .iter()
        .map(|link| (link.name.as_str(), link))
        .collect();

    let mut joints = Vec::with_capacity(robot.joints.len());
    let mut links = Vec::with_capacity(robot.links.len());
    let mut current = roots[0];
    links.push(robot_link(links_by_name[current]));
    while let Some(child_joints) = joints_by_parent.get(current) {
        let joint = child_joints[0];
        let child = links_by_name
            .get(joint.child.link.as_str())
            .ok_or_else(|| {
                Error::InvalidModel(format!(
                    "joint {} references a missing child link",
                    joint.name
                ))
            })?;
        joints.push(robot_joint(joint)?);
        links.push(robot_link(child));
        current = child.name.as_str();
    }

    if joints.len() != robot.joints.len() {
        return Err(Error::InvalidModel(
            "joint graph is disconnected or cyclic".to_owned(),
        ));
    }
    Ok(SerialModel { joints, links })
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
