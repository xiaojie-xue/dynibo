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
