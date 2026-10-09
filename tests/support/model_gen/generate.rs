//! Seeded construction of model specifications; RNG ordering is stable.
use super::{GENERATOR_VERSION, spec::*};
use std::collections::BTreeSet;

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
pub(super) fn domain_rng(seed: u64, domain: u64, index: u64) -> StableRng {
    StableRng::new(mix64(
        seed ^ domain.rotate_left(17) ^ index.wrapping_mul(0x9e37_79b9_7f4a_7c15),
    ))
}

fn mix64(mut value: u64) -> u64 {
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
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
