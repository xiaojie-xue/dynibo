//! Index conversions for the validated, root-first tree layout.
//!
//! Link 0 is the root. Model joint `j` connects its parent link to child link
//! `j + 1`; workspace entries use model joint indices and omit the root. Fixed
//! joints participate in this layout but have no entry in joint input vectors.
//! Use `Model::joint_dof_indices` to map model joints to those input entries.

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
