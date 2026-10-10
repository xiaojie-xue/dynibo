//! Reproducible coverage plans and environment-based corpus selection.
use super::super::context::TestRootType as RootType;
use super::{
    generate::{StableRng, domain_rng},
    spec::*,
};

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
            RootType::Fixed,
            JointMix::RevoluteOnly,
            AxisProfile::Cardinal,
            InertialProfile::Identity,
        ),
        options(
            2,
            TopologyKind::Serial,
            FixedJointLayout::ToolFrames,
            RootType::Floating,
            JointMix::AllSupported,
            AxisProfile::NearCardinal,
            InertialProfile::OffsetRotated,
        ),
        options(
            3,
            TopologyKind::SingleBranch,
            FixedJointLayout::Interleaved,
            RootType::Fixed,
            JointMix::AllSupported,
            AxisProfile::General,
            InertialProfile::Rotated,
        ),
        options(
            4,
            TopologyKind::SingleBranch,
            FixedJointLayout::Consecutive,
            RootType::Floating,
            JointMix::Rotational,
            AxisProfile::Cardinal,
            InertialProfile::Offset,
        ),
        options(
            5,
            TopologyKind::Balanced,
            FixedJointLayout::None,
            RootType::Fixed,
            JointMix::PrismaticOnly,
            AxisProfile::NearCardinal,
            InertialProfile::Rotated,
        ),
        options(
            6,
            TopologyKind::Balanced,
            FixedJointLayout::ToolFrames,
            RootType::Floating,
            JointMix::AllSupported,
            AxisProfile::General,
            InertialProfile::OffsetRotated,
        ),
        options(
            7,
            TopologyKind::Wide,
            FixedJointLayout::Interleaved,
            RootType::Fixed,
            JointMix::AllSupported,
            AxisProfile::Cardinal,
            InertialProfile::Offset,
        ),
        options(
            8,
            TopologyKind::Wide,
            FixedJointLayout::Consecutive,
            RootType::Floating,
            JointMix::Rotational,
            AxisProfile::NearCardinal,
            InertialProfile::Identity,
        ),
        options(
            9,
            TopologyKind::Unbalanced,
            FixedJointLayout::None,
            RootType::Fixed,
            JointMix::AllSupported,
            AxisProfile::General,
            InertialProfile::OffsetRotated,
        ),
        options(
            10,
            TopologyKind::Unbalanced,
            FixedJointLayout::ToolFrames,
            RootType::Floating,
            JointMix::PrismaticOnly,
            AxisProfile::Cardinal,
            InertialProfile::Rotated,
        ),
        options(
            11,
            TopologyKind::Serial,
            FixedJointLayout::Interleaved,
            RootType::Fixed,
            JointMix::Rotational,
            AxisProfile::NearCardinal,
            InertialProfile::Offset,
        ),
        options(
            12,
            TopologyKind::Serial,
            FixedJointLayout::Consecutive,
            RootType::Floating,
            JointMix::AllSupported,
            AxisProfile::General,
            InertialProfile::Identity,
        ),
        options(
            4,
            TopologyKind::Balanced,
            FixedJointLayout::ToolFrames,
            RootType::Fixed,
            JointMix::RevoluteOnly,
            AxisProfile::Cardinal,
            InertialProfile::Rotated,
        ),
        options(
            8,
            TopologyKind::Wide,
            FixedJointLayout::None,
            RootType::Floating,
            JointMix::AllSupported,
            AxisProfile::NearCardinal,
            InertialProfile::Offset,
        ),
        options(
            12,
            TopologyKind::Unbalanced,
            FixedJointLayout::Interleaved,
            RootType::Fixed,
            JointMix::PrismaticOnly,
            AxisProfile::General,
            InertialProfile::Identity,
        ),
        options(
            2,
            TopologyKind::SingleBranch,
            FixedJointLayout::Consecutive,
            RootType::Floating,
            JointMix::Rotational,
            AxisProfile::Cardinal,
            InertialProfile::OffsetRotated,
        ),
        options(
            5,
            TopologyKind::Serial,
            FixedJointLayout::ToolFrames,
            RootType::Fixed,
            JointMix::AllSupported,
            AxisProfile::NearCardinal,
            InertialProfile::Rotated,
        ),
        options(
            6,
            TopologyKind::Balanced,
            FixedJointLayout::Interleaved,
            RootType::Floating,
            JointMix::RevoluteOnly,
            AxisProfile::General,
            InertialProfile::Offset,
        ),
        options(
            7,
            TopologyKind::Wide,
            FixedJointLayout::Consecutive,
            RootType::Fixed,
            JointMix::AllSupported,
            AxisProfile::Cardinal,
            InertialProfile::Identity,
        ),
        options(
            9,
            TopologyKind::Unbalanced,
            FixedJointLayout::None,
            RootType::Floating,
            JointMix::PrismaticOnly,
            AxisProfile::NearCardinal,
            InertialProfile::OffsetRotated,
        ),
        options(
            10,
            TopologyKind::SingleBranch,
            FixedJointLayout::ToolFrames,
            RootType::Fixed,
            JointMix::Rotational,
            AxisProfile::General,
            InertialProfile::Rotated,
        ),
        options(
            11,
            TopologyKind::Serial,
            FixedJointLayout::Interleaved,
            RootType::Floating,
            JointMix::AllSupported,
            AxisProfile::Cardinal,
            InertialProfile::Offset,
        ),
        options(
            12,
            TopologyKind::Balanced,
            FixedJointLayout::Consecutive,
            RootType::Fixed,
            JointMix::RevoluteOnly,
            AxisProfile::NearCardinal,
            InertialProfile::Identity,
        ),
        options(
            8,
            TopologyKind::Wide,
            FixedJointLayout::None,
            RootType::Floating,
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
    base_mode: RootType,
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
