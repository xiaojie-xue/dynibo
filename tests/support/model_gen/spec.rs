//! Generated model descriptions and structural/physical validation.
use super::super::context::TestRootType as RootType;
use super::GENERATOR_VERSION;
use std::collections::{BTreeSet, HashMap};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TopologyKind {
    Serial,
    SingleBranch,
    Balanced,
    Wide,
    Unbalanced,
}

impl TopologyKind {
    pub(super) const fn label(self) -> &'static str {
        match self {
            Self::Serial => "serial",
            Self::SingleBranch => "single-branch",
            Self::Balanced => "balanced",
            Self::Wide => "wide",
            Self::Unbalanced => "unbalanced",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FixedJointLayout {
    None,
    Interleaved,
    Consecutive,
    ToolFrames,
}

impl FixedJointLayout {
    pub(super) const fn label(self) -> &'static str {
        match self {
            Self::None => "no-fixed",
            Self::Interleaved => "interleaved-fixed",
            Self::Consecutive => "consecutive-fixed",
            Self::ToolFrames => "tool-frames",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointMix {
    RevoluteOnly,
    PrismaticOnly,
    Rotational,
    AllSupported,
}

impl JointMix {
    pub(super) const fn label(self) -> &'static str {
        match self {
            Self::RevoluteOnly => "revolute",
            Self::PrismaticOnly => "prismatic",
            Self::Rotational => "rotational",
            Self::AllSupported => "mixed",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisProfile {
    Cardinal,
    NearCardinal,
    General,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InertialProfile {
    Identity,
    Offset,
    Rotated,
    OffsetRotated,
}

/// Explicit structural choices. Randomness controls physical parameters only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModelGenOptions {
    pub active_joints: usize,
    pub topology: TopologyKind,
    pub fixed_layout: FixedJointLayout,
    pub base_mode: RootType,
    pub joint_mix: JointMix,
    pub axis_profile: AxisProfile,
    pub inertial_profile: InertialProfile,
}

impl ModelGenOptions {
    pub const fn label(self) -> &'static str {
        // The complete label is built by ModelCase, but a stable category is useful in failures.
        match self.base_mode {
            RootType::Fixed => "fixed",
            RootType::Floating => "floating",
        }
    }
}

/// A deterministic test case: its topology is selected independently of the seed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModelCase {
    pub case_index: u64,
    pub seed: u64,
    pub options: ModelGenOptions,
}

impl ModelCase {
    pub fn id(self) -> String {
        format!(
            "v{GENERATOR_VERSION}-{:02}-{}-{}-{}-{}-{}-{}-{}",
            self.case_index,
            self.options.label(),
            self.options.topology.label(),
            self.options.fixed_layout.label(),
            self.options.joint_mix.label(),
            axis_label(self.options.axis_profile),
            inertial_label(self.options.inertial_profile),
            self.options.active_joints,
        )
    }
}

#[derive(Clone, Debug)]
pub struct ModelSpec {
    pub generator_version: u32,
    pub seed: u64,
    pub name: String,
    pub base_mode: RootType,
    pub links: Vec<LinkSpec>,
    pub joints: Vec<JointSpec>,
    pub targets: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct LinkSpec {
    pub name: String,
    pub inertial: Option<InertialSpec>,
}

#[derive(Clone, Debug)]
pub struct InertialSpec {
    pub mass: f64,
    pub origin_xyz: [f64; 3],
    pub origin_rpy: [f64; 3],
    pub inertia: [f64; 6],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
}

impl JointKind {
    pub(super) const fn urdf_name(self) -> &'static str {
        match self {
            Self::Revolute => "revolute",
            Self::Continuous => "continuous",
            Self::Prismatic => "prismatic",
            Self::Fixed => "fixed",
        }
    }

    pub(super) const fn is_active(self) -> bool {
        !matches!(self, Self::Fixed)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct JointLimits {
    pub lower: Option<f64>,
    pub upper: Option<f64>,
    pub effort: f64,
    pub velocity: f64,
}

#[derive(Clone, Debug)]
pub struct JointSpec {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub kind: JointKind,
    pub origin_xyz: [f64; 3],
    pub origin_rpy: [f64; 3],
    pub axis: Option<[f64; 3]>,
    pub limits: Option<JointLimits>,
}

#[derive(Debug)]
pub struct ModelMetadata {
    pub joint_count: usize,
    pub link_names: Vec<String>,
    pub branch_targets: Vec<String>,
    pub base_mode: RootType,
    pub case_id: String,
}

pub fn validate_spec(spec: &ModelSpec) -> Result<(), String> {
    if spec.generator_version != GENERATOR_VERSION {
        return Err(format!(
            "unknown generator version {}",
            spec.generator_version
        ));
    }
    if spec.links.is_empty() || spec.links[0].name != "base" {
        return Err("the first link must be the base link".to_owned());
    }
    let links: BTreeSet<_> = spec.links.iter().map(|link| link.name.as_str()).collect();
    if links.len() != spec.links.len() {
        return Err("link names must be unique".to_owned());
    }
    let joint_names: BTreeSet<_> = spec
        .joints
        .iter()
        .map(|joint| joint.name.as_str())
        .collect();
    if joint_names.len() != spec.joints.len() {
        return Err("joint names must be unique".to_owned());
    }
    let root = &spec.links[0];
    if spec.base_mode == RootType::Floating && root.inertial.is_none() {
        return Err("a floating base requires root inertia".to_owned());
    }
    for link in &spec.links {
        if let Some(inertial) = &link.inertial {
            validate_inertial(&link.name, inertial)?;
        }
    }
    let mut incoming = HashMap::new();
    for joint in &spec.joints {
        if !links.contains(joint.parent.as_str()) || !links.contains(joint.child.as_str()) {
            return Err(format!("joint {} references an unknown link", joint.name));
        }
        if joint.parent == joint.child {
            return Err(format!(
                "joint {} has identical parent and child",
                joint.name
            ));
        }
        if incoming
            .insert(joint.child.as_str(), joint.parent.as_str())
            .is_some()
        {
            return Err(format!("link {} has multiple parents", joint.child));
        }
        validate_joint(joint)?;
    }
    if incoming.contains_key("base") {
        return Err("base link must not have a parent".to_owned());
    }
    if incoming.len() + 1 != spec.links.len() {
        return Err("every non-base link must have exactly one parent".to_owned());
    }
    for link in spec.links.iter().skip(1) {
        let mut current = link.name.as_str();
        for _ in 0..spec.links.len() {
            if current == "base" {
                break;
            }
            current = *incoming
                .get(current)
                .ok_or_else(|| format!("link {} is disconnected", link.name))?;
        }
        if current != "base" {
            return Err(format!("link {} is part of a cycle", link.name));
        }
    }
    if spec.targets.is_empty()
        || spec
            .targets
            .iter()
            .any(|target| !links.contains(target.as_str()))
    {
        return Err("targets must name generated links".to_owned());
    }
    Ok(())
}

fn validate_inertial(name: &str, inertial: &InertialSpec) -> Result<(), String> {
    if !inertial.mass.is_finite() || inertial.mass <= 0.0 {
        return Err(format!("link {name} has non-positive mass"));
    }
    if inertial
        .origin_xyz
        .iter()
        .chain(inertial.origin_rpy.iter())
        .chain(inertial.inertia.iter())
        .any(|value| !value.is_finite())
    {
        return Err(format!("link {name} has non-finite inertia data"));
    }
    let [ixx, ixy, ixz, iyy, iyz, izz] = inertial.inertia;
    if ixx <= 0.0 || iyy <= 0.0 || izz <= 0.0 {
        return Err(format!("link {name} has non-positive principal inertia"));
    }
    if ixy != 0.0 || ixz != 0.0 || iyz != 0.0 {
        return Err(format!(
            "link {name} must use diagonal inertial-frame inertia"
        ));
    }
    if ixx + iyy < izz || ixx + izz < iyy || iyy + izz < ixx {
        return Err(format!(
            "link {name} violates rigid-body inertia inequalities"
        ));
    }
    Ok(())
}

fn validate_joint(joint: &JointSpec) -> Result<(), String> {
    if joint
        .origin_xyz
        .iter()
        .chain(joint.origin_rpy.iter())
        .any(|value| !value.is_finite())
    {
        return Err(format!("joint {} has a non-finite origin", joint.name));
    }
    match joint.kind {
        JointKind::Fixed => {
            if joint.axis.is_some() || joint.limits.is_some() {
                return Err(format!(
                    "fixed joint {} must not specify axis or limits",
                    joint.name
                ));
            }
        }
        _ => {
            let axis = joint
                .axis
                .ok_or_else(|| format!("joint {} is missing an axis", joint.name))?;
            let norm = axis.iter().map(|value| value * value).sum::<f64>().sqrt();
            if !norm.is_finite() || (norm - 1.0).abs() > 1.0e-10 {
                return Err(format!("joint {} axis must be unit length", joint.name));
            }
            let limits = joint
                .limits
                .ok_or_else(|| format!("joint {} is missing limits", joint.name))?;
            if !limits.effort.is_finite()
                || !limits.velocity.is_finite()
                || limits.effort <= 0.0
                || limits.velocity <= 0.0
            {
                return Err(format!(
                    "joint {} has invalid effort or velocity",
                    joint.name
                ));
            }
            if matches!(joint.kind, JointKind::Continuous) {
                if limits.lower.is_some() || limits.upper.is_some() {
                    return Err(format!(
                        "continuous joint {} must not have position limits",
                        joint.name
                    ));
                }
            } else if !matches!((limits.lower, limits.upper), (Some(lower), Some(upper)) if lower.is_finite() && upper.is_finite() && lower < upper)
            {
                return Err(format!("joint {} has invalid position limits", joint.name));
            }
        }
    }
    Ok(())
}

const fn axis_label(profile: AxisProfile) -> &'static str {
    match profile {
        AxisProfile::Cardinal => "cardinal-axis",
        AxisProfile::NearCardinal => "near-axis",
        AxisProfile::General => "general-axis",
    }
}

const fn inertial_label(profile: InertialProfile) -> &'static str {
    match profile {
        InertialProfile::Identity => "identity-inertia",
        InertialProfile::Offset => "offset-inertia",
        InertialProfile::Rotated => "rotated-inertia",
        InertialProfile::OffsetRotated => "offset-rotated-inertia",
    }
}
