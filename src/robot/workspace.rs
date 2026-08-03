use nalgebra::Vector3;

use crate::{Frame, Wrench};

/// An opaque, model-scoped identifier for a link.
///
/// A `LinkId` is valid for the robot model from which it was obtained, including
/// clones of that [`crate::Robot`]. It is a process-local handle and is not
/// intended for persistence or serialization.
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

/// A wrench associated with a model-scoped link identifier.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IndexedLoad {
    /// Link at whose origin the wrench is applied.
    pub link: LinkId,
    /// Wrench expressed in the selected link's coordinate frame.
    pub wrench: Wrench,
}

/// Reusable storage for runtime-sized robot calculations.
///
/// A workspace is created by [`crate::Robot::workspace`] and is bound to that
/// robot model, including clones of that [`crate::Robot`]. Creating it allocates
/// all required buffers; calculations that reuse it do not resize those
/// buffers. Use a distinct workspace for each concurrent calculation.
#[derive(Clone, Debug)]
pub struct Workspace {
    pub(super) model_id: u64,
    pub(super) joint_count: usize,
    pub(super) frames: Vec<Frame>,
    pub(super) angular_velocities: Vec<Vector3<f64>>,
    pub(super) angular_accelerations: Vec<Vector3<f64>>,
    pub(super) origin_accelerations: Vec<Vector3<f64>>,
    pub(super) link_accelerations: Vec<Vector3<f64>>,
    pub(super) link_loads: Vec<Wrench>,
    pub(super) jacobian: Vec<f64>,
    pub(super) q_work: Vec<f64>,
    pub(super) step: Vec<f64>,
}

impl Workspace {
    pub(super) fn new(model_id: u64, joint_count: usize) -> Self {
        Self {
            model_id,
            joint_count,
            frames: vec![Frame::identity(); joint_count],
            angular_velocities: vec![Vector3::zeros(); joint_count],
            angular_accelerations: vec![Vector3::zeros(); joint_count],
            origin_accelerations: vec![Vector3::zeros(); joint_count],
            link_accelerations: vec![Vector3::zeros(); joint_count],
            link_loads: vec![Wrench::zeros(); joint_count],
            jacobian: vec![0.0; 6 * joint_count],
            q_work: vec![0.0; joint_count],
            step: vec![0.0; joint_count],
        }
    }

    /// Returns the joint count for which this workspace was allocated.
    pub const fn joint_count(&self) -> usize {
        self.joint_count
    }
}
