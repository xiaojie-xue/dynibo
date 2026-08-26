use std::path::{Path, PathBuf};

use super::context::TestBaseMode as BaseMode;
use dynibo::{IndexedLoad, Robot, Wrench};

#[derive(Clone, Copy, Debug)]
pub struct Fixture {
    pub name: &'static str,
    pub urdf: &'static str,
    pub targets: &'static [&'static str],
}

impl Fixture {
    pub fn path(self) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/data")
            .join(self.urdf)
    }

    pub fn robot(self, base_mode: BaseMode) -> Robot {
        assert_eq!(
            base_mode,
            BaseMode::Fixed,
            "use FloatingRobot for floating fixtures"
        );
        Robot::from_urdf(self.path())
            .unwrap_or_else(|error| panic!("fixture {} must load: {error}", self.name))
    }
}

pub const SERIAL_ARM: Fixture = Fixture {
    name: "serial-arm",
    urdf: "test_arm.urdf",
    targets: &["test_link_4"],
};

pub const MIXED_ARM: Fixture = Fixture {
    name: "mixed-arm",
    urdf: "oracle_mixed.urdf",
    targets: &["tool"],
};

pub const TREE_ARM: Fixture = Fixture {
    name: "tree-arm",
    urdf: "test_tree_7.urdf",
    targets: &["left_tool", "right_tool"],
};

pub const FLOATING_ARM: Fixture = Fixture {
    name: "floating-arm",
    urdf: "floating_arm.urdf",
    targets: &["tool"],
};

#[derive(Clone, Debug)]
pub struct LoadSpec {
    pub link_name: String,
    pub wrench: Wrench,
}

impl LoadSpec {
    pub fn new(link_name: impl Into<String>, wrench: Wrench) -> Self {
        Self {
            link_name: link_name.into(),
            wrench,
        }
    }

    pub fn resolve(&self, robot: &Robot) -> IndexedLoad {
        IndexedLoad {
            link: robot.link_id(&self.link_name).unwrap_or_else(|error| {
                panic!("load link {} must resolve: {error}", self.link_name)
            }),
            wrench: self.wrench,
        }
    }
}

pub fn fixture_path(name: impl AsRef<Path>) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}
