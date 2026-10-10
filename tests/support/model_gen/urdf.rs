//! Deterministic URDF serialization of a generated specification.
use super::spec::*;
use std::fmt::Write as _;

pub fn serialize_urdf(spec: &ModelSpec) -> String {
    let mut urdf = format!("<?xml version=\"1.0\"?>\n<robot name=\"{}\">\n", spec.name);
    for link in &spec.links {
        match &link.inertial {
            Some(inertial) => {
                let [x, y, z] = inertial.origin_xyz;
                let [roll, pitch, yaw] = inertial.origin_rpy;
                let [ixx, ixy, ixz, iyy, iyz, izz] = inertial.inertia;
                writeln!(
                    urdf,
                    "  <link name=\"{}\">\n    <inertial>\n      <origin xyz=\"{x:.6} {y:.6} {z:.6}\" rpy=\"{roll:.6} {pitch:.6} {yaw:.6}\"/>\n      <mass value=\"{:.12}\"/>\n      <inertia ixx=\"{ixx:.12}\" ixy=\"{ixy:.12}\" ixz=\"{ixz:.12}\" iyy=\"{iyy:.12}\" iyz=\"{iyz:.12}\" izz=\"{izz:.12}\"/>\n    </inertial>\n  </link>",
                    link.name, inertial.mass,
                )
                .unwrap();
            }
            None => writeln!(urdf, "  <link name=\"{}\"/>", link.name).unwrap(),
        }
    }
    for joint in &spec.joints {
        let [x, y, z] = joint.origin_xyz;
        let [roll, pitch, yaw] = joint.origin_rpy;
        writeln!(
            urdf,
            "  <joint name=\"{}\" type=\"{}\">\n    <parent link=\"{}\"/><child link=\"{}\"/>\n    <origin xyz=\"{x:.6} {y:.6} {z:.6}\" rpy=\"{roll:.6} {pitch:.6} {yaw:.6}\"/>",
            joint.name,
            joint.kind.urdf_name(),
            joint.parent,
            joint.child,
        )
        .unwrap();
        if let Some([axis_x, axis_y, axis_z]) = joint.axis {
            writeln!(
                urdf,
                "    <axis xyz=\"{axis_x:.12} {axis_y:.12} {axis_z:.12}\"/>"
            )
            .unwrap();
        }
        if let Some(limits) = joint.limits {
            match (limits.lower, limits.upper) {
                (Some(lower), Some(upper)) => writeln!(
                    urdf,
                    "    <limit effort=\"{:.6}\" lower=\"{lower:.6}\" upper=\"{upper:.6}\" velocity=\"{:.6}\"/>",
                    limits.effort, limits.velocity,
                )
                .unwrap(),
                (None, None) => writeln!(
                    urdf,
                    "    <limit effort=\"{:.6}\" velocity=\"{:.6}\"/>",
                    limits.effort, limits.velocity,
                )
                .unwrap(),
                _ => unreachable!("validated joint limits must be complete or absent"),
            }
        }
        writeln!(urdf, "  </joint>").unwrap();
    }
    urdf.push_str("</robot>\n");
    urdf
}
