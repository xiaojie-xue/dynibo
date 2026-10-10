//! Temporary URDF ownership and failure reproduction artifacts.
use super::super::context::TestRootType as RootType;
use super::{GENERATOR_VERSION, generate::generate_spec, spec::*, urdf::serialize_urdf};
use dynibo::Robot;
use std::{
    fs,
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};
static NEXT_FILE_ID: AtomicU64 = AtomicU64::new(1);

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
        assert_eq!(
            self.metadata.base_mode,
            RootType::Fixed,
            "use FloatingRobot for floating generated cases"
        );
        Robot::from_urdf(&self.path).unwrap_or_else(|error| {
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
