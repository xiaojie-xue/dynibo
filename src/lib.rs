//! Tree-structured robot kinematics and dynamics with fixed-size joint-space calculation types.

#![warn(missing_docs)]

mod error;
mod model;
mod robot;
mod spatial;
mod urdf;

pub use error::{Error, Result};
pub use model::{Joint, JointType, Link};
pub use robot::{InverseKinematicsOptions, Load, Robot};
pub use spatial::{Frame, Jacobian, JointVector, Twist, Wrench};
