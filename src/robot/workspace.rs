use nalgebra::{Matrix3, SMatrix, Vector3};

use crate::{Frame, Twist, Wrench};

use super::Model;

/// Instance-local reusable storage for runtime-sized calculations.
///
/// Per-body arrays use model joint indices: slot `j` belongs to child link
/// `j + 1`, includes fixed joints, and excludes the separately handled root.
/// Joint vectors and Jacobian columns use DOF indices instead.
///
/// Contents are scratch, not cached results. Each algorithm initializes every
/// entry it reads, including after a failed call. Path-based kernels only write
/// their ancestor path; other entries must not be consumed. Algorithm-specific
/// borrows name the coordinate convention without allocating another buffer.
#[derive(Debug)]
pub(super) struct Workspace {
    /// Shared by parent-from-child transforms (dynamics), root-from-link poses
    /// (FK, IK, J-dot), and world-oriented poses relative to the root origin
    /// (direct Jacobian). Each consumer rebuilds the entries it needs.
    pub(super) frames: Vec<Frame>,
    pub(super) angular_velocities: Vec<Vector3<f64>>,
    /// Link-local angular acceleration in inverse dynamics; reused as
    /// link-local gravity by `GravityScratch::gravity_at_link`.
    pub(super) angular_accelerations: Vec<Vector3<f64>>,
    pub(super) origin_accelerations: Vec<Vector3<f64>>,
    pub(super) link_accelerations: Vec<Vector3<f64>>,
    pub(super) link_loads: Vec<Wrench>,
    pub(super) composite_masses: Vec<f64>,
    pub(super) composite_moments: Vec<Vector3<f64>>,
    pub(super) composite_inertias: Vec<Matrix3<f64>>,
    pub(super) spatial_velocities: Vec<Twist>,
    pub(super) bias_accelerations: Vec<Twist>,
    pub(super) spatial_accelerations: Vec<Twist>,
    pub(super) articulated_inertias: Vec<SMatrix<f64, 6, 6>>,
    pub(super) articulated_bias_forces: Vec<Wrench>,
    pub(super) articulated_u: Vec<Wrench>,
    pub(super) articulated_d: Vec<f64>,
    pub(super) articulated_joint_bias: Vec<f64>,
    pub(super) origin_velocities: Vec<Vector3<f64>>,
    pub(super) jacobian: Vec<f64>,
    pub(super) jacobian_derivative: Vec<f64>,
    pub(super) q_work: Vec<f64>,
    /// IK joint step, or freshly zeroed joint accelerations for velocity products.
    pub(super) step: Vec<f64>,
    pub(super) ancestor_path: Vec<usize>,
}

impl Workspace {
    pub(super) fn new(model: &Model) -> Self {
        let joint_count = model.joint_count();
        let model_joint_count = model.model_joint_count();
        Self {
            frames: vec![Frame::identity(); model_joint_count],
            angular_velocities: vec![Vector3::zeros(); model_joint_count],
            angular_accelerations: vec![Vector3::zeros(); model_joint_count],
            origin_accelerations: vec![Vector3::zeros(); model_joint_count],
            link_accelerations: vec![Vector3::zeros(); model_joint_count],
            link_loads: vec![Wrench::zeros(); model_joint_count],
            composite_masses: vec![0.0; model_joint_count],
            composite_moments: vec![Vector3::zeros(); model_joint_count],
            composite_inertias: vec![Matrix3::zeros(); model_joint_count],
            spatial_velocities: vec![Twist::zeros(); model_joint_count],
            bias_accelerations: vec![Twist::zeros(); model_joint_count],
            spatial_accelerations: vec![Twist::zeros(); model_joint_count],
            articulated_inertias: vec![SMatrix::zeros(); model_joint_count],
            articulated_bias_forces: vec![Wrench::zeros(); model_joint_count],
            articulated_u: vec![Wrench::zeros(); model_joint_count],
            articulated_d: vec![0.0; model_joint_count],
            articulated_joint_bias: vec![0.0; model_joint_count],
            origin_velocities: vec![Vector3::zeros(); model_joint_count],
            jacobian: vec![0.0; 6 * joint_count],
            jacobian_derivative: vec![0.0; 6 * joint_count],
            q_work: vec![0.0; joint_count],
            step: vec![0.0; joint_count],
            ancestor_path: vec![0; model_joint_count],
        }
    }
}
