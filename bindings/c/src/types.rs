//! C-compatible value types and their core-library conversions.
use crate::boundary::{CResult, core_error, invalid};
use dynibo::{BaseState, Frame, InverseKinematicsOptions, Twist};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DyniboPose {
    pub translation: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}
impl Default for DyniboPose {
    fn default() -> Self {
        Self {
            translation: [0.0; 3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }
}
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DyniboTwist {
    pub angular: [f64; 3],
    pub linear: [f64; 3],
}
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DyniboBaseState {
    pub frame: DyniboPose,
    pub velocity: DyniboTwist,
    pub acceleration: DyniboTwist,
}
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DyniboLoad {
    pub link_id: usize,
    pub torque: [f64; 3],
    pub force: [f64; 3],
}
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DyniboIkOptions {
    pub max_iterations: usize,
    pub translation_tolerance: f64,
    pub rotation_tolerance: f64,
    pub damping: f64,
    pub max_step_norm: f64,
}
impl Default for DyniboIkOptions {
    fn default() -> Self {
        let x = InverseKinematicsOptions::default();
        Self {
            max_iterations: x.max_iterations,
            translation_tolerance: x.translation_tolerance,
            rotation_tolerance: x.rotation_tolerance,
            damping: x.damping,
            max_step_norm: x.max_step_norm,
        }
    }
}

pub(super) fn frame_from_pose(p: &DyniboPose) -> CResult<Frame> {
    let [x, y, z, w] = p.rotation_xyzw;
    let norm = x * x + y * y + z * z + w * w;
    if !p.translation.iter().all(|x| x.is_finite()) || !norm.is_finite() || norm <= 1e-24 {
        return Err(invalid(
            "pose contains non-finite values or a zero quaternion",
        ));
    }
    Ok(Frame::from_parts(
        Translation3::from(Vector3::from(p.translation)),
        UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
    ))
}
pub(super) fn pose_from_frame(f: &Frame) -> DyniboPose {
    let q = f.rotation.quaternion();
    DyniboPose {
        translation: f.translation.vector.into(),
        rotation_xyzw: [q.i, q.j, q.k, q.w],
    }
}
pub(super) fn twist_from_c(t: DyniboTwist) -> Twist {
    Twist::new(Vector3::from(t.angular), Vector3::from(t.linear))
}
pub(super) fn twist_to_c(t: Twist) -> DyniboTwist {
    DyniboTwist {
        angular: t.angular.into(),
        linear: t.linear.into(),
    }
}
pub(super) fn base_from_c(b: &DyniboBaseState) -> CResult<BaseState> {
    BaseState::new(
        frame_from_pose(&b.frame)?,
        twist_from_c(b.velocity),
        twist_from_c(b.acceleration),
    )
    .map_err(core_error)
}
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_ik_options_default() -> DyniboIkOptions {
    DyniboIkOptions::default()
}
