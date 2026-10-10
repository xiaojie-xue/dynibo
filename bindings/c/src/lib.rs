//! Stable, typed C ABI for dynibo.
#![cfg_attr(coverage_nightly, feature(coverage_attribute))]
#![allow(clippy::missing_safety_doc, reason = "C contracts are in dynibo.h")]

mod boundary;
mod dynamics;
mod handles;
mod kinematics;
mod types;

pub use boundary::{DyniboStatus, dynibo_last_error_message};
pub use dynamics::{
    dynibo_floating_forward_dynamics, dynibo_floating_gravity, dynibo_floating_inverse_dynamics,
    dynibo_floating_mass_matrix, dynibo_floating_velocity_product_forces, dynibo_forward_dynamics,
    dynibo_gravity, dynibo_inverse_dynamics, dynibo_mass_matrix, dynibo_velocity_product_forces,
};
pub use handles::{
    DyniboFloatingRobot, DyniboFloatingWorkspace, DyniboRobot, DyniboWorkspace,
    dynibo_floating_robot_destroy, dynibo_floating_robot_from_urdf,
    dynibo_floating_robot_generalized_count, dynibo_floating_robot_joint_count,
    dynibo_floating_robot_link_count, dynibo_floating_robot_link_id, dynibo_floating_robot_name,
    dynibo_floating_workspace_create, dynibo_floating_workspace_destroy, dynibo_robot_destroy,
    dynibo_robot_from_urdf, dynibo_robot_generalized_count, dynibo_robot_joint_count,
    dynibo_robot_link_count, dynibo_robot_link_id, dynibo_robot_name, dynibo_robot_set_base_frame,
    dynibo_workspace_create, dynibo_workspace_destroy,
};
pub use kinematics::{
    dynibo_floating_forward_acceleration_kinematics, dynibo_floating_forward_kinematics,
    dynibo_floating_forward_kinematics_all, dynibo_floating_forward_velocity_kinematics,
    dynibo_floating_jacobian, dynibo_floating_jacobian_derivative,
    dynibo_forward_acceleration_kinematics, dynibo_forward_kinematics,
    dynibo_forward_kinematics_all, dynibo_forward_velocity_kinematics, dynibo_inverse_kinematics,
    dynibo_jacobian, dynibo_jacobian_derivative,
};
pub use types::{
    DyniboBaseState, DyniboIkOptions, DyniboLoad, DyniboPose, DyniboTwist,
    dynibo_ik_options_default,
};

use std::ffi::c_char;

#[unsafe(no_mangle)]
pub extern "C" fn dynibo_version() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0").as_ptr().cast()
}

#[cfg(test)]
#[cfg_attr(coverage_nightly, coverage(off))]
mod tests;
