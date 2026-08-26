//! Tree-structured robot kinematics and dynamics with allocation-free calculation APIs.

#![warn(missing_docs)]

mod base;
mod error;
mod model;
mod robot;
mod spatial;

pub use base::BaseState;
pub use error::{Error, ErrorCategory, Result};
pub use model::JointType;
pub use robot::{FloatingRobot, IndexedLoad, InverseKinematicsOptions, LinkId, Robot};
pub use spatial::{Frame, Twist, Wrench};
