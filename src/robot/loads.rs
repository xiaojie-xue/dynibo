use crate::{Error, Result, Wrench};

use super::{FloatingRobot, LinkId, Model, Robot};

/// A resisting wrench associated with a model-scoped link identifier.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IndexedLoad {
    /// Link at whose origin the wrench is applied.
    pub link: LinkId,
    /// Resisting wrench expressed in the selected link's coordinate frame.
    pub wrench: Wrench,
}

/// Model-scoped, reusable storage for at most one resisting wrench per link.
///
/// Creation allocates capacity for every link. Updating, adding, removing, and
/// clearing loads do not allocate on success. Pass [`Self::as_slice`] to dynamics.
#[derive(Debug)]
pub struct LoadBuffer {
    owner: LinkId,
    positions: Vec<usize>,
    loads: Vec<IndexedLoad>,
}

impl LoadBuffer {
    pub(super) fn new(model: &Model) -> Self {
        Self {
            owner: LinkId::new(model.model_id, 0),
            positions: vec![usize::MAX; model.link_count()],
            loads: Vec::with_capacity(model.link_count()),
        }
    }

    fn validate_link(&self, link: LinkId) -> Result<()> {
        if link.model_id == self.owner.model_id && link.index < self.positions.len() {
            Ok(())
        } else {
            Err(Error::InvalidLinkId)
        }
    }

    /// Replaces a link's load, inserting it if absent. Errors leave the buffer unchanged.
    pub fn set(&mut self, link: LinkId, wrench: Wrench) -> Result<()> {
        self.validate_link(link)?;
        if !wrench.is_finite() {
            return Err(Error::NonFiniteInput { input: "load" });
        }
        let position = &mut self.positions[link.index];
        if *position == usize::MAX {
            *position = self.loads.len();
            self.loads.push(IndexedLoad { link, wrench });
        } else {
            self.loads[*position].wrench = wrench;
        }
        Ok(())
    }

    /// Adds to a link's resisting wrench. Overflow leaves the buffer unchanged.
    pub fn add(&mut self, link: LinkId, wrench: Wrench) -> Result<()> {
        self.validate_link(link)?;
        if !wrench.is_finite() {
            return Err(Error::NonFiniteInput { input: "load" });
        }
        let position = self.positions[link.index];
        let value = if position == usize::MAX {
            wrench
        } else {
            let previous = self.loads[position].wrench;
            Wrench::new(
                previous.torque + wrench.torque,
                previous.force + wrench.force,
            )
        };
        if !value.is_finite() {
            return Err(Error::NumericalFailure {
                operation: "load aggregation",
            });
        }
        self.set(link, value)
    }

    /// Removes a link's load if present, retaining all allocated capacity.
    pub fn remove(&mut self, link: LinkId) -> Result<()> {
        self.validate_link(link)?;
        let position = self.positions[link.index];
        if position != usize::MAX {
            self.loads.swap_remove(position);
            self.positions[link.index] = usize::MAX;
            if let Some(moved) = self.loads.get(position) {
                self.positions[moved.link.index] = position;
            }
        }
        Ok(())
    }

    /// Removes every load while retaining allocated capacity.
    pub fn clear(&mut self) {
        for load in &self.loads {
            self.positions[load.link.index] = usize::MAX;
        }
        self.loads.clear();
    }

    /// Borrows the active loads. Their order can change after removal.
    pub fn as_slice(&self) -> &[IndexedLoad] {
        &self.loads
    }

    /// Returns the number of links with an explicitly stored load.
    pub fn len(&self) -> usize {
        self.loads.len()
    }

    /// Returns whether the buffer contains no loads.
    pub fn is_empty(&self) -> bool {
        self.loads.is_empty()
    }
}

impl Robot {
    /// Creates reusable external-load storage scoped to this robot's model.
    pub fn load_buffer(&self) -> LoadBuffer {
        LoadBuffer::new(&self.model)
    }
}

impl FloatingRobot {
    /// Creates reusable external-load storage scoped to this robot's model.
    pub fn load_buffer(&self) -> LoadBuffer {
        LoadBuffer::new(&self.model)
    }
}
