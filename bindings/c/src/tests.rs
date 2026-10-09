use super::*;
use crate::boundary::{CResult, call};
use std::{
    ffi::{CStr, CString},
    ptr,
};

fn fixture_path() -> CString {
    CString::new(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../tests/data/test_arm.urdf")
            .to_string_lossy()
            .as_bytes(),
    )
    .unwrap()
}

unsafe fn fixed_handles() -> (*mut DyniboRobot, *mut DyniboWorkspace, usize) {
    let mut robot = ptr::null_mut();
    assert_eq!(
        unsafe { dynibo_robot_from_urdf(fixture_path().as_ptr(), &mut robot) },
        DyniboStatus::Ok
    );
    let mut workspace = ptr::null_mut();
    assert_eq!(
        unsafe { dynibo_workspace_create(robot, &mut workspace) },
        DyniboStatus::Ok
    );
    let mut target = 0;
    assert_eq!(
        unsafe { dynibo_robot_link_id(robot, c"test_link_4".as_ptr(), &mut target) },
        DyniboStatus::Ok
    );
    (robot, workspace, target)
}

unsafe fn floating_handles() -> (
    *mut DyniboFloatingRobot,
    *mut DyniboFloatingWorkspace,
    usize,
) {
    let mut robot = ptr::null_mut();
    assert_eq!(
        unsafe { dynibo_floating_robot_from_urdf(fixture_path().as_ptr(), &mut robot) },
        DyniboStatus::Ok
    );
    let mut workspace = ptr::null_mut();
    assert_eq!(
        unsafe { dynibo_floating_workspace_create(robot, &mut workspace) },
        DyniboStatus::Ok
    );
    let mut target = 0;
    assert_eq!(
        unsafe { dynibo_floating_robot_link_id(robot, c"test_link_4".as_ptr(), &mut target) },
        DyniboStatus::Ok
    );
    (robot, workspace, target)
}

mod calculations;
mod overlap;
mod ownership;
