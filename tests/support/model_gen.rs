use std::{
    fmt::Write as _,
    fs,
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};

use dynibo::{BaseMode, Robot};

static NEXT_FILE_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Clone, Copy, Debug)]
pub struct ModelGenOptions {
    pub active_joints: usize,
    pub branched: bool,
    pub include_fixed_joints: bool,
    pub base_mode: BaseMode,
}

impl ModelGenOptions {
    pub fn from_seed(seed: u64) -> Self {
        Self {
            active_joints: 1 + seed as usize % 12,
            branched: seed & 1 != 0,
            include_fixed_joints: !seed.is_multiple_of(3),
            base_mode: if seed % 4 == 3 {
                BaseMode::Floating
            } else {
                BaseMode::Fixed
            },
        }
    }
}

#[derive(Debug)]
pub struct ModelMetadata {
    pub joint_count: usize,
    pub link_names: Vec<String>,
    pub branch_targets: Vec<String>,
    pub base_mode: BaseMode,
}

#[derive(Debug)]
pub struct GeneratedModel {
    pub seed: u64,
    pub urdf: String,
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
                "generated model must load: seed={} path={} error={error}\n{}",
                self.seed,
                self.path.display(),
                self.urdf
            )
        })
    }
}

impl Drop for GeneratedModel {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.path);
    }
}

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
        let mut value = self.state;
        value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        value ^ (value >> 31)
    }

    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1_u64 << 53) as f64)
    }

    pub fn signed(&mut self, magnitude: f64) -> f64 {
        magnitude * (2.0 * self.unit() - 1.0)
    }
}

pub fn generate_model(seed: u64, options: ModelGenOptions) -> GeneratedModel {
    assert!((1..=12).contains(&options.active_joints));
    let mut rng = StableRng::new(seed);
    let mut links = String::new();
    let mut joints = String::new();
    let mut link_names = vec!["base".to_owned()];
    let mut branch_targets = Vec::new();

    links.push_str(&inertial_link("base", 2.0 + rng.unit(), &mut rng));
    let mut previous = "base".to_owned();

    for active in 0..options.active_joints {
        let mut parent = if options.branched && active >= 3 && active % 3 == 0 {
            format!("link_{}", active / 2)
        } else {
            previous.clone()
        };

        if options.include_fixed_joints && active % 3 == 1 {
            let mount = format!("mount_{active}");
            writeln!(links, "  <link name=\"{mount}\"/>").unwrap();
            writeln!(
                joints,
                "  <joint name=\"fixed_{active}\" type=\"fixed\">\n    \
                 <parent link=\"{parent}\"/><child link=\"{mount}\"/>\n    \
                 <origin xyz=\"{:.6} {:.6} {:.6}\" rpy=\"0 0 0\"/>\n  </joint>",
                rng.signed(0.12),
                rng.signed(0.12),
                0.08 + rng.unit() * 0.12,
            )
            .unwrap();
            link_names.push(mount.clone());
            parent = mount;
        }

        let child = format!("link_{active}");
        links.push_str(&inertial_link(&child, 0.4 + 4.0 * rng.unit(), &mut rng));
        let joint_type = match rng.next_u64() % 3 {
            0 => "revolute",
            1 => "continuous",
            _ => "prismatic",
        };
        let mut axis = [rng.signed(1.0), rng.signed(1.0), rng.signed(1.0)];
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if norm < 0.2 {
            axis = [0.0, 0.0, 1.0];
        } else {
            for value in &mut axis {
                *value /= norm;
            }
        }
        let limit = if joint_type == "continuous" {
            "    <limit effort=\"100\" velocity=\"10\"/>\n".to_owned()
        } else if joint_type == "prismatic" {
            "    <limit effort=\"100\" lower=\"-0.5\" upper=\"0.5\" velocity=\"10\"/>\n".to_owned()
        } else {
            "    <limit effort=\"100\" lower=\"-2.8\" upper=\"2.8\" velocity=\"10\"/>\n".to_owned()
        };
        writeln!(
            joints,
            "  <joint name=\"joint_{active}\" type=\"{joint_type}\">\n    \
             <parent link=\"{parent}\"/><child link=\"{child}\"/>\n    \
             <origin xyz=\"{:.6} {:.6} {:.6}\" rpy=\"{:.6} {:.6} {:.6}\"/>\n    \
             <axis xyz=\"{:.12} {:.12} {:.12}\"/>\n{limit}  </joint>",
            0.12 + 0.18 * rng.unit(),
            rng.signed(0.15),
            rng.signed(0.15),
            rng.signed(0.35),
            rng.signed(0.35),
            rng.signed(0.35),
            axis[0],
            axis[1],
            axis[2],
        )
        .unwrap();
        link_names.push(child.clone());
        if options.branched && active % 3 == 0 {
            branch_targets.push(child.clone());
        }
        previous = child;
    }
    if !branch_targets.contains(&previous) {
        branch_targets.push(previous);
    }

    let urdf = format!(
        "<?xml version=\"1.0\"?>\n<robot name=\"generated_{seed}\">\n{links}{joints}</robot>\n"
    );
    let file_id = NEXT_FILE_ID.fetch_add(1, Ordering::Relaxed);
    let path = std::env::temp_dir().join(format!(
        "dynibo-generated-{}-{seed}-{file_id}.urdf",
        std::process::id()
    ));
    fs::write(&path, &urdf).unwrap_or_else(|error| {
        panic!(
            "failed to write generated fixture {}: {error}",
            path.display()
        )
    });

    GeneratedModel {
        seed,
        urdf,
        metadata: ModelMetadata {
            joint_count: options.active_joints,
            link_names,
            branch_targets,
            base_mode: options.base_mode,
        },
        path,
    }
}

pub fn selected_model_seeds(default_cases: u64) -> Vec<u64> {
    if let Some(seed) = std::env::var_os("DYNIBO_TEST_SEED") {
        let seed = parse_seed(&seed.to_string_lossy());
        return vec![seed];
    }
    let cases = std::env::var_os("DYNIBO_TEST_CASES").map_or(default_cases, |cases| {
        cases
            .to_string_lossy()
            .parse()
            .expect("DYNIBO_TEST_CASES must be an unsigned integer")
    });
    corpus_model_seeds(cases)
}

/// Produces a reproducible, stratified sequence of pseudo-random `u64` seeds.
///
/// The high bits come from SplitMix64. The residue modulo 12 is assigned by
/// corpus position so the first twelve cases still cover every supported model
/// size and the associated fixed/floating and serial/branched option patterns.
pub fn corpus_model_seeds(cases: u64) -> Vec<u64> {
    let mut rng = StableRng::new(0xd1b5_4a32_d192_ed03);
    (0..cases)
        .map(|index| {
            let random = rng.next_u64();
            let residue = index % 12;
            let base = random - random % 12;
            base.checked_add(residue)
                .unwrap_or_else(|| base - 12 + residue)
        })
        .collect()
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

fn inertial_link(name: &str, mass: f64, rng: &mut StableRng) -> String {
    let x = 0.08 + 0.25 * rng.unit();
    let y = 0.08 + 0.25 * rng.unit();
    let z = 0.08 + 0.25 * rng.unit();
    let ixx = mass * (y * y + z * z) / 12.0;
    let iyy = mass * (x * x + z * z) / 12.0;
    let izz = mass * (x * x + y * y) / 12.0;
    format!(
        "  <link name=\"{name}\">\n    <inertial>\n      \
         <origin xyz=\"{:.6} {:.6} {:.6}\" rpy=\"{:.6} {:.6} {:.6}\"/>\n      \
         <mass value=\"{mass:.12}\"/>\n      \
         <inertia ixx=\"{ixx:.12}\" ixy=\"0\" ixz=\"0\" iyy=\"{iyy:.12}\" iyz=\"0\" izz=\"{izz:.12}\"/>\n    \
         </inertial>\n  </link>\n",
        rng.signed(0.04),
        rng.signed(0.04),
        rng.signed(0.04),
        rng.signed(0.4),
        rng.signed(0.4),
        rng.signed(0.4),
    )
}
