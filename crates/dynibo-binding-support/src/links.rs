use std::sync::Arc;

use dynibo::{FloatingRobot, LinkId, Robot, RobotModel};

/// Bidirectional index table mapping integer binding link IDs to model-scoped
/// [`LinkId`] handles.
#[derive(Clone, Debug)]
pub struct LinkTable {
    links: Arc<[LinkId]>,
}

impl LinkTable {
    pub fn from_model(model: &RobotModel) -> Self {
        let links: Vec<LinkId> = (0..model.link_count())
            .map(|index| model.link_id_at(index).expect("enumerated link is valid"))
            .collect();
        Self {
            links: links.into(),
        }
    }

    pub fn from_robot(robot: &Robot) -> Self {
        let links: Vec<LinkId> = (0..robot.link_count())
            .map(|index| robot.link_id_at(index).expect("enumerated link is valid"))
            .collect();
        Self {
            links: links.into(),
        }
    }

    pub fn from_floating_robot(robot: &FloatingRobot) -> Self {
        let links: Vec<LinkId> = (0..robot.link_count())
            .map(|index| robot.link_id_at(index).expect("enumerated link is valid"))
            .collect();
        Self {
            links: links.into(),
        }
    }

    pub fn resolve(&self, index: usize) -> Option<LinkId> {
        self.links.get(index).copied()
    }

    pub fn index_of(&self, link: LinkId) -> Option<usize> {
        self.links.iter().position(|candidate| *candidate == link)
    }

    pub fn root(&self) -> Option<LinkId> {
        self.links.first().copied()
    }

    pub fn same_model(&self, other: &Self) -> bool {
        self.root() == other.root()
    }

    pub fn len(&self) -> usize {
        self.links.len()
    }

    pub fn is_empty(&self) -> bool {
        self.links.is_empty()
    }
}

#[cfg(test)]
#[cfg_attr(coverage_nightly, coverage(off))]
mod tests {
    use std::path::PathBuf;

    use super::*;

    #[test]
    fn link_table_tracks_length_and_identity() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/data/test_arm.urdf");
        let model = RobotModel::from_urdf(&path).unwrap();
        let table = LinkTable::from_model(&model);
        assert_eq!(table.len(), model.link_count());
        assert!(!table.is_empty());
        assert_eq!(table.resolve(0), table.root());
        assert_eq!(table.index_of(table.root().unwrap()), Some(0));
    }
}
