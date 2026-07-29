//! Serial robot-arm kinematics and inverse dynamics with fixed-size calculation types.

mod error;
mod model;
mod passive;
mod robot_arm;
mod spatial;
mod urdf;

pub use error::{Error, Result};
pub use model::{JointKind, JointLimit, RobotLink};
pub use passive::{PassiveJointMap, RobotWithPassiveJoints};
pub use robot_arm::RobotArm;
pub use spatial::{Frame, Jacobian, JointVector, Motion, Wrench};
