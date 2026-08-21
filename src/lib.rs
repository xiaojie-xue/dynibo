//! Tree-structured robot kinematics and dynamics with runtime-size workspace APIs.

#![warn(missing_docs)]

mod base;
mod error;
mod model;
mod robot;
mod spatial;

pub use base::{BaseMode, BaseState};
pub use error::{Error, ErrorCategory, Result};
pub use model::{Joint, JointType, Link};
pub use robot::{IndexedLoad, InverseKinematicsOptions, LinkId, Robot, Workspace};
pub use spatial::{Frame, Twist, Wrench};
