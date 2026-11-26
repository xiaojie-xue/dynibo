use crate::{BaseState, Frame, Twist};

pub(super) const FLOATING_BASE_DOF: usize = 6;

/// Compile-time specialization separating fixed-base (`DOF = 0`) and
/// floating-base (`DOF = 6`) kernels without duplicating algorithms.
pub(super) trait BaseKind {
    const DOF: usize;
    const IS_FLOATING: bool;
}

#[derive(Debug)]
pub(super) struct Fixed;
#[derive(Debug)]
pub(super) struct Floating;

impl BaseKind for Fixed {
    const DOF: usize = 0;
    const IS_FLOATING: bool = false;
}

impl BaseKind for Floating {
    const DOF: usize = FLOATING_BASE_DOF;
    const IS_FLOATING: bool = true;
}

#[derive(Clone, Copy, Debug)]
pub(super) struct BaseMotion<'a> {
    pub(super) frame: &'a Frame,
    pub(super) velocity: Twist,
    pub(super) acceleration: Twist,
}

impl<'a> BaseMotion<'a> {
    #[inline]
    pub(super) fn stationary(frame: &'a Frame) -> Self {
        Self {
            frame,
            velocity: Twist::zeros(),
            acceleration: Twist::zeros(),
        }
    }

    #[inline]
    pub(super) fn from_state(state: &'a BaseState) -> Self {
        Self {
            frame: state.frame(),
            velocity: state.velocity(),
            acceleration: state.acceleration(),
        }
    }
}
