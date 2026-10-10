//! Tree-structured robot kinematics and dynamics with allocation-free calculation APIs.
#![cfg_attr(coverage_nightly, feature(coverage_attribute))]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod base;
mod error;
mod model;
mod robot;
mod spatial;

pub use base::BaseState;
pub use error::{Error, ErrorCategory, Result};
pub use model::JointType;
pub use robot::{
    FloatingRobot, IndexedLoad, InverseKinematicsOptions, LinkId, LoadBuffer, Robot, RobotModel,
};
pub use spatial::{Frame, Twist, Wrench};
