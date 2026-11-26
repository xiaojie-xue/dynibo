//! Tree-structured robot kinematics and dynamics with allocation-free calculation APIs.
//!
//! # Main Entry Points
//!
//! - [`Robot`] — fixed-base robot with owned, reusable calculation storage (`G == J`).
//! - [`FloatingRobot`] — floating-base robot with owned storage (`G == J + 6`);
//!   takes an explicit [`BaseState`] on every calculation.
//! - [`RobotModel`] — immutable, `Clone`able model handle without calculation
//!   storage; construct multiple independent [`Robot`] or [`FloatingRobot`]
//!   instances for parallel threads, or call [`Robot::fork`] / [`FloatingRobot::fork`].
//! - [`RobotState`] / [`FloatingRobotState`] — zero-allocation precomputed kinematic
//!   snapshots created by [`Robot::state`] / [`FloatingRobot::state`] for reusing
//!   transforms, poses, and link velocities across multiple queries at the same `(q, qd)`.
//! - [`LoadBuffer`] — model-scoped reusable external-load storage.
//!
//! # Shared Conventions
//!
//! - **Units**: SI (metres, radians, seconds, kilograms, newtons, newton-metres).
//! - **Joint order**: `q`, `qd`, and `qdd` contain `joint_count()` non-fixed joints
//!   in breadth-first topological order from the root link (preserving URDF joint
//!   order among children of the same parent). Inspect with `joint_name(i)`.
//! - **Generalized order**: floating-base vectors prepend world-frame angular (3)
//!   then linear (3) base components at the root-link origin before the `J` joints.
//! - **Twists**: angular-first `[angular_xyz, linear_xyz]` representing classical
//!   velocity or classical acceleration in the world frame.
//! - **External loads**: link-local resisting wrenches `[torque_xyz, force_xyz]` at
//!   the link origin (added in inverse dynamics, subtracted in forward dynamics;
//!   opposite sign to a physical force applied to the robot).
//! - **Matrices**: flat column-major slices (`6 * G` for Jacobians, `G * G` for
//!   mass matrices).
//!
//! # Quick Start
//!
//! ```no_run
//! use dynibo::Robot;
//!
//! fn main() -> dynibo::Result<()> {
//!     let mut robot = Robot::from_urdf("robot.urdf")?;
//!     let tool = robot.link_id("tool")?;
//!     let q = vec![0.0; robot.joint_count()];
//!     let pose = robot.forward_kinematics(&q, tool)?;
//!     println!("translation: {}", pose.translation.vector.transpose());
//!     Ok(())
//! }
//! ```
#![cfg_attr(coverage_nightly, feature(coverage_attribute))]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod base;
mod error;
mod model;
mod robot;
mod spatial;

pub use nalgebra;

pub use base::BaseState;
pub use error::{Error, ErrorCategory, Result, UrdfError};
pub use model::JointType;
pub use robot::{
    FloatingRobot, FloatingRobotState, IndexedLoad, InverseKinematicsOptions, LinkId, LoadBuffer,
    Robot, RobotModel, RobotState,
};
pub use spatial::{Frame, Twist, Wrench};
