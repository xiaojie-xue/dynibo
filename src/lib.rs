//! Tree-structured robot kinematics and dynamics with runtime-size workspace APIs.

#![warn(missing_docs)]

mod error;
mod model;
mod robot;
mod spatial;
mod urdf;

pub use error::{Error, Result};
pub use model::{Joint, JointType, Link};
pub use robot::{IndexedLoad, InverseKinematicsOptions, LinkId, Robot, Workspace};
pub use spatial::{Frame, Twist, Wrench};
