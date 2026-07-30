//! Serial robot-arm kinematics and inverse dynamics with fixed-size calculation types.

mod error;
mod model;
mod robot_arm;
mod spatial;
mod urdf;

pub use error::{Error, Result};
pub use model::{JointKind, JointLimit, RobotJoint, RobotLink};
pub use robot_arm::{ExternalWrench, LinkId, RobotArm};
pub use spatial::{Frame, Jacobian, JointVector, Motion, Wrench};
