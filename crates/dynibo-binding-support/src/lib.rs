//! Shared helpers for C/C++ and Python language bindings.
#![cfg_attr(all(coverage_nightly, test), feature(coverage_attribute))]

pub mod links;
pub mod loads;
pub mod pose;

pub use links::LinkTable;
pub use loads::{LoadInputError, aggregate_loads};
pub use pose::{frame_from_xyzw, xyzw_from_frame};
