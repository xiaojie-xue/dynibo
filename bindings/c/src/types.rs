//! C-compatible value types and their core-library conversions.
use crate::boundary::{CResult, core_error, invalid};
use dynibo::{BaseState, Frame, InverseKinematicsOptions, Twist};
use dynibo_binding_support::{frame_from_xyzw, xyzw_from_frame};
use nalgebra::Vector3;

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
    frame_from_xyzw(p.translation, p.rotation_xyzw).map_err(invalid)
}
pub(super) fn pose_from_frame(f: &Frame) -> DyniboPose {
    let (translation, rotation_xyzw) = xyzw_from_frame(f);
    DyniboPose {
        translation,
        rotation_xyzw,
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
