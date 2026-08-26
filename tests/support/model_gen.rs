use std::{
    collections::{BTreeSet, HashMap},
    fmt::Write as _,
    fs,
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};

use dynibo::{BaseMode, Robot};

/// Bump this whenever a deliberate incompatibility changes the seed-to-model mapping.
pub const GENERATOR_VERSION: u32 = 2;

static NEXT_FILE_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TopologyKind {
    Serial,
    SingleBranch,
    Balanced,
    Wide,
    Unbalanced,
}

impl TopologyKind {
    const fn label(self) -> &'static str {
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
    const fn label(self) -> &'static str {
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
    const fn label(self) -> &'static str {
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
    pub base_mode: BaseMode,
    pub joint_mix: JointMix,
    pub axis_profile: AxisProfile,
    pub inertial_profile: InertialProfile,
}

impl ModelGenOptions {
    pub const fn label(self) -> &'static str {
        // The complete label is built by ModelCase, but a stable category is useful in failures.
        match self.base_mode {
            BaseMode::Fixed => "fixed",
            BaseMode::Floating => "floating",
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
    pub base_mode: BaseMode,
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
    const fn urdf_name(self) -> &'static str {
        match self {
            Self::Revolute => "revolute",
            Self::Continuous => "continuous",
            Self::Prismatic => "prismatic",
            Self::Fixed => "fixed",
        }
    }

    const fn is_active(self) -> bool {
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
    pub base_mode: BaseMode,
    pub case_id: String,
}

#[derive(Debug)]
pub struct GeneratedModel {
    pub seed: u64,
    pub urdf: String,
    pub spec: ModelSpec,
    pub metadata: ModelMetadata,
    path: PathBuf,
}

impl GeneratedModel {
    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn robot(&self) -> Robot {
        Robot::from_urdf_with_base(&self.path, self.metadata.base_mode).unwrap_or_else(|error| {
            panic!(
                "generated model must load: case={} seed={:#018x} path={} error={error}\n{}",
                self.metadata.case_id,
                self.seed,
                self.path.display(),
                self.urdf
            )
        })
    }

    fn preserve_failure(&self) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("target")
            .join("test-failures");
        let name = format!(
            "generated-v{}-{}-{:#018x}",
            self.spec.generator_version, self.metadata.case_id, self.seed
        );
        let directory = root.join(name);
        if fs::create_dir_all(&directory).is_err() {
            return;
        }
        let _ = fs::write(directory.join("model.urdf"), &self.urdf);
        let _ = fs::write(
            directory.join("model-spec.txt"),
            format!("{:#?}\n", self.spec),
        );
        let reproduce = format!(
            "DYNIBO_TEST_SEED={:#018x} DYNIBO_TEST_CASE_ID={} \\\n+  cargo test --test generated_conformance -- --nocapture\n",
            self.seed, self.metadata.case_id
        );
        let _ = fs::write(directory.join("reproduce.txt"), reproduce);
    }
}

impl Drop for GeneratedModel {
    fn drop(&mut self) {
        if std::thread::panicking() {
            self.preserve_failure();
        }
        if std::env::var_os("DYNIBO_TEST_KEEP_URDF").is_none() {
            let _ = fs::remove_file(&self.path);
        }
    }
}

/// A small stable PRNG used only for test model construction.
#[derive(Clone, Copy, Debug)]
pub struct StableRng {
    state: u64,
}

impl StableRng {
    pub const fn new(seed: u64) -> Self {
        Self {
            state: seed ^ 0x9e37_79b9_7f4a_7c15,
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        mix64(self.state)
    }

    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1_u64 << 53) as f64)
    }

    pub fn signed(&mut self, magnitude: f64) -> f64 {
        magnitude * (2.0 * self.unit() - 1.0)
    }
}

/// Derives independent random streams, so adding one field does not perturb another field's data.
fn domain_rng(seed: u64, domain: u64, index: u64) -> StableRng {
    StableRng::new(mix64(
        seed ^ domain.rotate_left(17) ^ index.wrapping_mul(0x9e37_79b9_7f4a_7c15),
    ))
}

fn mix64(mut value: u64) -> u64 {
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

/// Retained for focused tests that need one explicitly chosen structure.
pub fn generate_model(seed: u64, options: ModelGenOptions) -> GeneratedModel {
    generate_case(&ModelCase {
        case_index: 0,
        seed,
        options,
    })
}

pub fn generate_case(case: &ModelCase) -> GeneratedModel {
    let spec = generate_spec(case);
    validate_spec(&spec).unwrap_or_else(|error| {
        panic!(
            "generated specification must be valid: case={} seed={:#018x}: {error}\n{spec:#?}",
            case.id(),
            case.seed
        )
    });
    let urdf = serialize_urdf(&spec);
    let file_id = NEXT_FILE_ID.fetch_add(1, Ordering::Relaxed);
    let path = std::env::temp_dir().join(format!(
        "dynibo-generated-v{}-{}-{:#018x}-{file_id}.urdf",
        GENERATOR_VERSION, case.case_index, case.seed,
    ));
    fs::write(&path, &urdf).unwrap_or_else(|error| {
        panic!(
            "failed to write generated fixture {}: {error}",
            path.display()
        )
    });
    if std::env::var_os("DYNIBO_TEST_KEEP_URDF").is_some() {
        eprintln!(
            "generated URDF retained: case={} seed={:#018x} path={}",
            case.id(),
            case.seed,
            path.display()
        );
    }
    let link_names = spec.links.iter().map(|link| link.name.clone()).collect();
    let joint_count = spec
        .joints
        .iter()
        .filter(|joint| joint.kind.is_active())
        .count();
    GeneratedModel {
        seed: case.seed,
        urdf,
        metadata: ModelMetadata {
            joint_count,
            link_names,
            branch_targets: spec.targets.clone(),
            base_mode: spec.base_mode,
            case_id: case.id(),
        },
        spec,
        path,
    }
}

pub fn generate_spec(case: &ModelCase) -> ModelSpec {
    let options = case.options;
    assert!((1..=12).contains(&options.active_joints));
    let mut links = vec![LinkSpec {
        name: "base".to_owned(),
        inertial: Some(inertial_link(case.seed, 0, true, options.inertial_profile)),
    }];
    let mut joints = Vec::new();
    let kinds = active_joint_kinds(case.seed, options.active_joints, options.joint_mix);

    for (active, &kind) in kinds.iter().enumerate() {
        let mut parent = active_parent(active, options.topology);
        for mount in fixed_mounts(active, options.fixed_layout) {
            let mount_name = format!("mount_{active}_{mount}");
            links.push(LinkSpec {
                name: mount_name.clone(),
                inertial: None,
            });
            joints.push(fixed_joint(
                case.seed,
                active as u64 * 4 + mount as u64,
                format!("fixed_{active}_{mount}"),
                parent,
                mount_name.clone(),
            ));
            parent = mount_name;
        }

        let child = format!("link_{active}");
        links.push(LinkSpec {
            name: child.clone(),
            inertial: Some(inertial_link(
                case.seed,
                active as u64 + 1,
                false,
                options.inertial_profile,
            )),
        });
        let mut rng = domain_rng(case.seed, 0x4a4f_494e_545f_0001, active as u64);
        joints.push(JointSpec {
            name: format!("joint_{active}"),
            parent,
            child,
            kind,
            origin_xyz: [0.12 + 0.18 * rng.unit(), rng.signed(0.15), rng.signed(0.15)],
            origin_rpy: [rng.signed(0.35), rng.signed(0.35), rng.signed(0.35)],
            axis: Some(sample_axis(case.seed, active as u64, options.axis_profile)),
            limits: Some(limits(kind)),
        });
    }

    let mut targets = leaf_names(&links, &joints);
    if options.fixed_layout == FixedJointLayout::ToolFrames {
        let leaves = targets.clone();
        targets.clear();
        for (index, leaf) in leaves.into_iter().enumerate() {
            let tool = format!("tool_{index}");
            links.push(LinkSpec {
                name: tool.clone(),
                inertial: None,
            });
            joints.push(fixed_joint(
                case.seed,
                0x1_0000 + index as u64,
                format!("tool_fixed_{index}"),
                leaf,
                tool.clone(),
            ));
            targets.push(tool);
        }
    }
    targets.sort();

    ModelSpec {
        generator_version: GENERATOR_VERSION,
        seed: case.seed,
        name: format!("generated_v{GENERATOR_VERSION}_{:016x}", case.seed),
        base_mode: options.base_mode,
        links,
        joints,
        targets,
    }
}

fn active_parent(active: usize, topology: TopologyKind) -> String {
    if active == 0 {
        return "base".to_owned();
    }
    let parent = match topology {
        TopologyKind::Serial => active - 1,
        TopologyKind::SingleBranch if active >= 3 && active.is_multiple_of(3) => active / 2,
        TopologyKind::Balanced => (active - 1) / 2,
        TopologyKind::Wide if active >= 3 => active % 3,
        TopologyKind::Unbalanced if active >= 5 && active.is_multiple_of(5) => 1,
        _ => active - 1,
    };
    format!("link_{parent}")
}

fn fixed_mounts(active: usize, layout: FixedJointLayout) -> std::ops::Range<usize> {
    let count = match layout {
        FixedJointLayout::None | FixedJointLayout::ToolFrames => 0,
        FixedJointLayout::Interleaved if active % 3 == 1 => 1,
        FixedJointLayout::Consecutive if active % 4 == 1 => 2,
        _ => 0,
    };
    0..count
}

fn active_joint_kinds(seed: u64, count: usize, mix: JointMix) -> Vec<JointKind> {
    let mut kinds = match mix {
        JointMix::RevoluteOnly => vec![JointKind::Revolute; count],
        JointMix::PrismaticOnly => vec![JointKind::Prismatic; count],
        JointMix::Rotational => (0..count)
            .map(|index| {
                if index % 2 == 0 {
                    JointKind::Revolute
                } else {
                    JointKind::Continuous
                }
            })
            .collect(),
        JointMix::AllSupported => (0..count)
            .map(|index| match index % 3 {
                0 => JointKind::Revolute,
                1 => JointKind::Continuous,
                _ => JointKind::Prismatic,
            })
            .collect(),
    };
    let mut rng = domain_rng(seed, 0x4a4f_494e_545f_0002, 0);
    for index in (1..kinds.len()).rev() {
        let swap = (rng.next_u64() % (index as u64 + 1)) as usize;
        kinds.swap(index, swap);
    }
    kinds
}

fn limits(kind: JointKind) -> JointLimits {
    match kind {
        JointKind::Revolute => JointLimits {
            lower: Some(-2.8),
            upper: Some(2.8),
            effort: 100.0,
            velocity: 10.0,
        },
        JointKind::Prismatic => JointLimits {
            lower: Some(-0.5),
            upper: Some(0.5),
            effort: 100.0,
            velocity: 10.0,
        },
        JointKind::Continuous | JointKind::Fixed => JointLimits {
            lower: None,
            upper: None,
            effort: 100.0,
            velocity: 10.0,
        },
    }
}

fn sample_axis(seed: u64, index: u64, profile: AxisProfile) -> [f64; 3] {
    let mut rng = domain_rng(seed, 0x4a4f_494e_545f_0003, index);
    let cardinal = match rng.next_u64() % 6 {
        0 => [1.0, 0.0, 0.0],
        1 => [-1.0, 0.0, 0.0],
        2 => [0.0, 1.0, 0.0],
        3 => [0.0, -1.0, 0.0],
        4 => [0.0, 0.0, 1.0],
        _ => [0.0, 0.0, -1.0],
    };
    match profile {
        AxisProfile::Cardinal => cardinal,
        AxisProfile::NearCardinal => normalize([
            cardinal[0] + rng.signed(0.08),
            cardinal[1] + rng.signed(0.08),
            cardinal[2] + rng.signed(0.08),
        ]),
        AxisProfile::General => loop {
            let raw = [rng.signed(1.0), rng.signed(1.0), rng.signed(1.0)];
            let norm = raw.iter().map(|value| value * value).sum::<f64>().sqrt();
            if norm >= 0.2 {
                break normalize(raw);
            }
        },
    }
}

fn normalize(mut value: [f64; 3]) -> [f64; 3] {
    let norm = value
        .iter()
        .map(|component| component * component)
        .sum::<f64>()
        .sqrt();
    for component in &mut value {
        *component /= norm;
    }
    value
}

fn inertial_link(seed: u64, index: u64, root: bool, profile: InertialProfile) -> InertialSpec {
    let mut rng = domain_rng(seed, 0x494e_4552_5449_0001, index);
    // Normal physical range only: a well-conditioned box with positive mass and inertia.
    let mass = if root {
        2.0 + 3.0 * rng.unit()
    } else {
        0.2 + 4.8 * rng.unit()
    };
    let x = 0.08 + 0.25 * rng.unit();
    let y = 0.08 + 0.25 * rng.unit();
    let z = 0.08 + 0.25 * rng.unit();
    let (origin_xyz, origin_rpy) = match profile {
        InertialProfile::Identity => ([0.0; 3], [0.0; 3]),
        InertialProfile::Offset => (
            [rng.signed(0.04), rng.signed(0.04), rng.signed(0.04)],
            [0.0; 3],
        ),
        InertialProfile::Rotated => (
            [0.0; 3],
            [rng.signed(0.4), rng.signed(0.4), rng.signed(0.4)],
        ),
        InertialProfile::OffsetRotated => (
            [rng.signed(0.04), rng.signed(0.04), rng.signed(0.04)],
            [rng.signed(0.4), rng.signed(0.4), rng.signed(0.4)],
        ),
    };
    InertialSpec {
        mass,
        origin_xyz,
        origin_rpy,
        inertia: [
            mass * (y * y + z * z) / 12.0,
            0.0,
            0.0,
            mass * (x * x + z * z) / 12.0,
            0.0,
            mass * (x * x + y * y) / 12.0,
        ],
    }
}

fn fixed_joint(seed: u64, index: u64, name: String, parent: String, child: String) -> JointSpec {
    let mut rng = domain_rng(seed, 0x4649_5845_445f_0001, index);
    JointSpec {
        name,
        parent,
        child,
        kind: JointKind::Fixed,
        origin_xyz: [rng.signed(0.12), rng.signed(0.12), 0.08 + rng.unit() * 0.12],
        origin_rpy: [0.0; 3],
        axis: None,
        limits: None,
    }
}

fn leaf_names(links: &[LinkSpec], joints: &[JointSpec]) -> Vec<String> {
    let parents: BTreeSet<_> = joints.iter().map(|joint| joint.parent.as_str()).collect();
    links
        .iter()
        .filter(|link| link.name != "base" && !parents.contains(link.name.as_str()))
        .map(|link| link.name.clone())
        .collect()
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
    if spec.base_mode == BaseMode::Floating && root.inertial.is_none() {
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

pub fn selected_model_cases(default_cases: u64) -> Vec<ModelCase> {
    if let Some(seed) = std::env::var_os("DYNIBO_TEST_SEED") {
        let case_index = std::env::var_os("DYNIBO_TEST_CASE_ID")
            .map(|value| parse_case_id(&value.to_string_lossy()))
            .unwrap_or(0);
        return vec![ModelCase {
            case_index,
            seed: parse_seed(&seed.to_string_lossy()),
            options: corpus_options(case_index),
        }];
    }
    let cases = requested_case_count(default_cases);
    if std::env::var_os("DYNIBO_TEST_RANDOMIZE").is_some() {
        let master_seed = std::env::var_os("DYNIBO_TEST_MASTER_SEED")
            .map(|seed| parse_seed(&seed.to_string_lossy()))
            .unwrap_or_else(os_random_seed);
        eprintln!("generated URDF random corpus: master_seed={master_seed:#018x} cases={cases}");
        return randomized_model_cases(cases, master_seed);
    }
    corpus_model_cases(cases)
}

fn requested_case_count(default_cases: u64) -> u64 {
    std::env::var_os("DYNIBO_TEST_CASES").map_or(default_cases, |cases| {
        cases
            .to_string_lossy()
            .parse()
            .expect("DYNIBO_TEST_CASES must be an unsigned integer")
    })
}

fn os_random_seed() -> u64 {
    let mut bytes = [0_u8; 8];
    getrandom::fill(&mut bytes).expect("the operating system must provide a random test seed");
    u64::from_le_bytes(bytes)
}

pub fn corpus_model_cases(cases: u64) -> Vec<ModelCase> {
    corpus_model_seeds(cases)
        .into_iter()
        .enumerate()
        .map(|(case_index, seed)| ModelCase {
            case_index: case_index as u64,
            seed,
            options: corpus_options(case_index as u64),
        })
        .collect()
}

/// Creates a reproducible exploration corpus from one OS-generated master seed.
pub fn randomized_model_cases(cases: u64, master_seed: u64) -> Vec<ModelCase> {
    (0..cases)
        .map(|case_index| ModelCase {
            case_index,
            seed: domain_rng(master_seed, 0x4d41_5354_4552_0001, case_index).next_u64(),
            options: corpus_options(case_index),
        })
        .collect()
}

/// Produces reproducible parameter seeds. Structural coverage is handled by `corpus_options`.
pub fn corpus_model_seeds(cases: u64) -> Vec<u64> {
    let mut rng = StableRng::new(0xd1b5_4a32_d192_ed03);
    (0..cases)
        .map(|index| {
            // Preserve the V1 corpus seed sequence. In V2 this residue has no
            // structural meaning: `corpus_options` independently supplies it.
            let random = rng.next_u64();
            let residue = index % 12;
            let base = random - random % 12;
            base.checked_add(residue)
                .unwrap_or_else(|| base - 12 + residue)
        })
        .collect()
}

fn corpus_options(index: u64) -> ModelGenOptions {
    const CASES: [ModelGenOptions; 24] = [
        options(
            1,
            TopologyKind::Serial,
            FixedJointLayout::None,
            BaseMode::Fixed,
            JointMix::RevoluteOnly,
            AxisProfile::Cardinal,
            InertialProfile::Identity,
        ),
        options(
            2,
            TopologyKind::Serial,
            FixedJointLayout::ToolFrames,
            BaseMode::Floating,
            JointMix::AllSupported,
            AxisProfile::NearCardinal,
            InertialProfile::OffsetRotated,
        ),
        options(
            3,
            TopologyKind::SingleBranch,
            FixedJointLayout::Interleaved,
            BaseMode::Fixed,
            JointMix::AllSupported,
            AxisProfile::General,
            InertialProfile::Rotated,
        ),
        options(
            4,
            TopologyKind::SingleBranch,
            FixedJointLayout::Consecutive,
            BaseMode::Floating,
            JointMix::Rotational,
            AxisProfile::Cardinal,
            InertialProfile::Offset,
        ),
        options(
            5,
            TopologyKind::Balanced,
            FixedJointLayout::None,
            BaseMode::Fixed,
            JointMix::PrismaticOnly,
            AxisProfile::NearCardinal,
            InertialProfile::Rotated,
        ),
        options(
            6,
            TopologyKind::Balanced,
            FixedJointLayout::ToolFrames,
            BaseMode::Floating,
            JointMix::AllSupported,
            AxisProfile::General,
            InertialProfile::OffsetRotated,
        ),
        options(
            7,
            TopologyKind::Wide,
            FixedJointLayout::Interleaved,
            BaseMode::Fixed,
            JointMix::AllSupported,
            AxisProfile::Cardinal,
            InertialProfile::Offset,
        ),
        options(
            8,
            TopologyKind::Wide,
            FixedJointLayout::Consecutive,
            BaseMode::Floating,
            JointMix::Rotational,
            AxisProfile::NearCardinal,
            InertialProfile::Identity,
        ),
        options(
            9,
            TopologyKind::Unbalanced,
            FixedJointLayout::None,
            BaseMode::Fixed,
            JointMix::AllSupported,
            AxisProfile::General,
            InertialProfile::OffsetRotated,
        ),
        options(
            10,
            TopologyKind::Unbalanced,
            FixedJointLayout::ToolFrames,
            BaseMode::Floating,
            JointMix::PrismaticOnly,
            AxisProfile::Cardinal,
            InertialProfile::Rotated,
        ),
        options(
            11,
            TopologyKind::Serial,
            FixedJointLayout::Interleaved,
            BaseMode::Fixed,
            JointMix::Rotational,
            AxisProfile::NearCardinal,
            InertialProfile::Offset,
        ),
        options(
            12,
            TopologyKind::Serial,
            FixedJointLayout::Consecutive,
            BaseMode::Floating,
            JointMix::AllSupported,
            AxisProfile::General,
            InertialProfile::Identity,
        ),
        options(
            4,
            TopologyKind::Balanced,
            FixedJointLayout::ToolFrames,
            BaseMode::Fixed,
            JointMix::RevoluteOnly,
            AxisProfile::Cardinal,
            InertialProfile::Rotated,
        ),
        options(
            8,
            TopologyKind::Wide,
            FixedJointLayout::None,
            BaseMode::Floating,
            JointMix::AllSupported,
            AxisProfile::NearCardinal,
            InertialProfile::Offset,
        ),
        options(
            12,
            TopologyKind::Unbalanced,
            FixedJointLayout::Interleaved,
            BaseMode::Fixed,
            JointMix::PrismaticOnly,
            AxisProfile::General,
            InertialProfile::Identity,
        ),
        options(
            2,
            TopologyKind::SingleBranch,
            FixedJointLayout::Consecutive,
            BaseMode::Floating,
            JointMix::Rotational,
            AxisProfile::Cardinal,
            InertialProfile::OffsetRotated,
        ),
        options(
            5,
            TopologyKind::Serial,
            FixedJointLayout::ToolFrames,
            BaseMode::Fixed,
            JointMix::AllSupported,
            AxisProfile::NearCardinal,
            InertialProfile::Rotated,
        ),
        options(
            6,
            TopologyKind::Balanced,
            FixedJointLayout::Interleaved,
            BaseMode::Floating,
            JointMix::RevoluteOnly,
            AxisProfile::General,
            InertialProfile::Offset,
        ),
        options(
            7,
            TopologyKind::Wide,
            FixedJointLayout::Consecutive,
            BaseMode::Fixed,
            JointMix::AllSupported,
            AxisProfile::Cardinal,
            InertialProfile::Identity,
        ),
        options(
            9,
            TopologyKind::Unbalanced,
            FixedJointLayout::None,
            BaseMode::Floating,
            JointMix::PrismaticOnly,
            AxisProfile::NearCardinal,
            InertialProfile::OffsetRotated,
        ),
        options(
            10,
            TopologyKind::SingleBranch,
            FixedJointLayout::ToolFrames,
            BaseMode::Fixed,
            JointMix::Rotational,
            AxisProfile::General,
            InertialProfile::Rotated,
        ),
        options(
            11,
            TopologyKind::Serial,
            FixedJointLayout::Interleaved,
            BaseMode::Floating,
            JointMix::AllSupported,
            AxisProfile::Cardinal,
            InertialProfile::Offset,
        ),
        options(
            12,
            TopologyKind::Balanced,
            FixedJointLayout::Consecutive,
            BaseMode::Fixed,
            JointMix::RevoluteOnly,
            AxisProfile::NearCardinal,
            InertialProfile::Identity,
        ),
        options(
            8,
            TopologyKind::Wide,
            FixedJointLayout::None,
            BaseMode::Floating,
            JointMix::AllSupported,
            AxisProfile::General,
            InertialProfile::OffsetRotated,
        ),
    ];
    CASES[index as usize % CASES.len()]
}

const fn options(
    active_joints: usize,
    topology: TopologyKind,
    fixed_layout: FixedJointLayout,
    base_mode: BaseMode,
    joint_mix: JointMix,
    axis_profile: AxisProfile,
    inertial_profile: InertialProfile,
) -> ModelGenOptions {
    ModelGenOptions {
        active_joints,
        topology,
        fixed_layout,
        base_mode,
        joint_mix,
        axis_profile,
        inertial_profile,
    }
}

fn parse_seed(value: &str) -> u64 {
    if let Some(hex) = value.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).expect("DYNIBO_TEST_SEED must be a valid hexadecimal u64")
    } else {
        value
            .parse()
            .expect("DYNIBO_TEST_SEED must be an unsigned integer")
    }
}

fn parse_case_id(value: &str) -> u64 {
    let numeric = value
        .strip_prefix('v')
        .and_then(|rest| rest.split_once('-').map(|(_, rest)| rest))
        .and_then(|rest| rest.split('-').next())
        .unwrap_or(value);
    numeric
        .parse()
        .expect("DYNIBO_TEST_CASE_ID must be a corpus index or a generated case id")
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
