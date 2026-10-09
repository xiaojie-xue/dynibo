//! Index conversions for the validated, root-first tree layout.
//!
//! Link 0 is the root. Model joint `j` connects its parent link to child link
//! `j + 1`; workspace entries use model joint indices and omit the root. Fixed
//! joints participate in this layout but have no entry in joint input vectors.
//! Use `Model::joint_dof_indices` to map model joints to those input entries.

/// An opaque, model-scoped identifier for a link.
///
/// A `LinkId` is valid for the robot model from which it was obtained, including
/// instances created with [`crate::Robot::fork`]. It is a process-local handle
/// and is not intended for persistence or serialization.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LinkId {
    pub(super) model_id: u64,
    pub(super) index: usize,
}

impl LinkId {
    pub(super) const fn new(model_id: u64, index: usize) -> Self {
        Self { model_id, index }
    }
}

/// Returns the child link reached through a model joint, including fixed joints.
#[inline]
pub(super) const fn child_link_index(joint_index: usize) -> usize {
    joint_index + 1
}

/// Returns the incoming model joint and workspace slot of a non-root link.
/// Callers must handle root link 0 separately before using this conversion.
#[inline]
pub(super) const fn incoming_joint_index(non_root_link_index: usize) -> usize {
    non_root_link_index - 1
}
