//! Tree-structured robot kinematics and dynamics with allocation-free calculation APIs.

#![warn(missing_docs)]

mod base;
mod error;
mod model;
mod robot;
mod spatial;

pub use base::{BaseMode, BaseState};
pub use error::{Error, ErrorCategory, Result};
pub use model::JointType;
pub use robot::{IndexedLoad, InverseKinematicsOptions, LinkId, Robot};
pub use spatial::{Frame, Twist, Wrench};
