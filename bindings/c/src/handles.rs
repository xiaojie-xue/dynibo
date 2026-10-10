//! Model/workspace ownership, metadata, and reusable load conversion.
use crate::boundary::{
    CResult, DyniboStatus, call, core_error, invalid, required_mut, required_ref,
};
use crate::types::{DyniboLoad, DyniboPose, frame_from_pose};
use dynibo::{FloatingRobot, Frame, IndexedLoad, LinkId, Robot, RobotModel, Wrench};
use nalgebra::Vector3;
use std::{
    ffi::{CStr, CString, c_char},
    ptr,
};

/// Fixed-base model handle and metadata, plus its persistent base frame.
pub struct DyniboRobot {
    pub(super) inner: RobotModel,
    pub(super) base_frame: Frame,
    pub(super) link_ids: Vec<LinkId>,
    pub(super) name: CString,
}
/// Fixed-base calculation storage.
pub struct DyniboWorkspace {
    pub(super) inner: Robot,
    pub(super) poses: Box<[Frame]>,
    pub(super) converted_loads: Box<[IndexedLoad]>,
    pub(super) load_positions: Box<[usize]>,
}
/// Floating-base model and metadata. It intentionally contains no base state.
pub struct DyniboFloatingRobot {
    pub(super) inner: RobotModel,
    pub(super) link_ids: Vec<LinkId>,
    pub(super) name: CString,
}
/// Floating-base calculation storage.
pub struct DyniboFloatingWorkspace {
    pub(super) inner: FloatingRobot,
    pub(super) poses: Box<[Frame]>,
    pub(super) converted_loads: Box<[IndexedLoad]>,
    pub(super) load_positions: Box<[usize]>,
}

/// Converts and aggregates C loads using preallocated workspace buffers.
///
/// # Safety
/// For nonzero `n`, a non-null `p` must describe `n` initialized, aligned loads
/// in one allocation of at most `isize::MAX` bytes. It must remain readable and
/// unmodified during this call, without aliasing `output` or `positions`.
pub(super) unsafe fn loads<'a>(
    ids: &[LinkId],
    output: &'a mut [IndexedLoad],
    positions: &mut [usize],
    p: *const DyniboLoad,
    n: usize,
) -> CResult<&'a [IndexedLoad]> {
    if n == 0 {
        return Ok(&output[..0]);
    }
    if p.is_null() {
        return Err(invalid(
            "loads must not be null when load_count is non-zero",
        ));
    }
    let values = unsafe { std::slice::from_raw_parts(p, n) };
    for load in values {
        if load.link_id >= ids.len() {
            return Err(invalid(format!("invalid link id {}", load.link_id)));
        }
        if !load
            .torque
            .iter()
            .chain(load.force.iter())
            .all(|v| v.is_finite())
        {
            return Err(core_error(dynibo::Error::NonFiniteInput { input: "load" }));
        }
    }
    let mut used = 0;
    for load in values {
        let position = &mut positions[load.link_id];
        if *position == usize::MAX {
            *position = used;
            output[used] = IndexedLoad {
                link: ids[load.link_id],
                wrench: Wrench::zeros(),
            };
            used += 1;
        }
        let converted = &mut output[*position].wrench;
        *converted = Wrench::new(
            converted.torque + Vector3::from(load.torque),
            converted.force + Vector3::from(load.force),
        );
    }
    for load in values {
        positions[load.link_id] = usize::MAX;
    }
    // Restore the index map before reporting overflow, so a failed conversion
    // cannot leave stale entries for the next call.
    if output[..used].iter().any(|load| !load.wrench.is_finite()) {
        return Err(core_error(dynibo::Error::NumericalFailure {
            operation: "load aggregation",
        }));
    }
    Ok(&output[..used])
}
/// Borrows fixed-base handles and synchronizes the instance's base frame.
///
/// # Safety
/// Non-null handles must come from this ABI and remain live for `'a`. The robot
/// must not be mutated, and the workspace must be exclusively accessible, for
/// that entire lifetime. Null handles and model mismatches return errors.
pub(super) unsafe fn fixed_parts<'a>(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
) -> CResult<(&'a DyniboRobot, &'a mut DyniboWorkspace)> {
    let robot = unsafe { required_ref(robot, "robot") }?;
    let workspace = unsafe { required_mut(workspace, "workspace") }?;
    if robot.inner.root_link_id() != workspace.inner.root_link_id() {
        return Err(invalid("workspace does not belong to this robot model"));
    }
    workspace
        .inner
        .set_base_frame(robot.base_frame)
        .map_err(core_error)?;
    Ok((robot, workspace))
}
/// Borrows floating-base handles after checking their shared model identity.
///
/// # Safety
/// Non-null handles must come from this ABI and remain live for `'a`. The robot
/// must not be mutated, and the workspace must be exclusively accessible, for
/// that entire lifetime. Null handles and model mismatches return errors.
pub(super) unsafe fn floating_parts<'a>(
    robot: *const DyniboFloatingRobot,
    workspace: *mut DyniboFloatingWorkspace,
) -> CResult<(&'a DyniboFloatingRobot, &'a mut DyniboFloatingWorkspace)> {
    let robot = unsafe { required_ref(robot, "robot") }?;
    let workspace = unsafe { required_mut(workspace, "workspace") }?;
    if robot.inner.root_link_id() != workspace.inner.root_link_id() {
        return Err(invalid("workspace does not belong to this robot model"));
    }
    Ok((robot, workspace))
}
pub(super) fn link(ids: &[LinkId], target: usize) -> CResult<LinkId> {
    ids.get(target)
        .copied()
        .ok_or_else(|| invalid(format!("invalid link id {target}")))
}
fn make_loads(ids: &[LinkId]) -> Box<[IndexedLoad]> {
    ids.iter()
        .copied()
        .map(|link| IndexedLoad {
            link,
            wrench: Wrench::zeros(),
        })
        .collect()
}
fn make_load_positions(ids: &[LinkId]) -> Box<[usize]> {
    vec![usize::MAX; ids.len()].into_boxed_slice()
}
fn info<R>(
    robot: &R,
    links: impl FnOnce(&R) -> usize,
    at: impl Fn(&R, usize) -> dynibo::Result<LinkId>,
    name: impl FnOnce(&R) -> &str,
) -> CResult<(Vec<LinkId>, CString)> {
    let ids = (0..links(robot))
        .map(|i| at(robot, i).expect("enumerated link is valid"))
        .collect();
    let name = CString::new(name(robot))
        .map_err(|_| (DyniboStatus::ModelError, "robot name contains NUL".into()))?;
    Ok((ids, name))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_from_urdf(
    path: *const c_char,
    output: *mut *mut DyniboRobot,
) -> DyniboStatus {
    call(|| {
        let out = unsafe { required_mut(output, "output") }?;
        *out = ptr::null_mut();
        if path.is_null() {
            return Err(invalid("path must not be null"));
        }
        let path = unsafe { CStr::from_ptr(path) }
            .to_str()
            .map_err(|_| invalid("path must be valid UTF-8"))?;
        let inner = RobotModel::from_urdf(path).map_err(core_error)?;
        let (link_ids, name) = info(
            &inner,
            RobotModel::link_count,
            RobotModel::link_id_at,
            RobotModel::name,
        )?;
        *out = Box::into_raw(Box::new(DyniboRobot {
            inner,
            base_frame: Frame::identity(),
            link_ids,
            name,
        }));
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_robot_from_urdf(
    path: *const c_char,
    output: *mut *mut DyniboFloatingRobot,
) -> DyniboStatus {
    call(|| {
        let out = unsafe { required_mut(output, "output") }?;
        *out = ptr::null_mut();
        if path.is_null() {
            return Err(invalid("path must not be null"));
        }
        let path = unsafe { CStr::from_ptr(path) }
            .to_str()
            .map_err(|_| invalid("path must be valid UTF-8"))?;
        let inner = RobotModel::from_urdf(path).map_err(core_error)?;
        inner.validate_floating_base().map_err(core_error)?;
        let (link_ids, name) = info(
            &inner,
            RobotModel::link_count,
            RobotModel::link_id_at,
            RobotModel::name,
        )?;
        *out = Box::into_raw(Box::new(DyniboFloatingRobot {
            inner,
            link_ids,
            name,
        }));
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_destroy(p: *mut DyniboRobot) {
    if !p.is_null() {
        drop(unsafe { Box::from_raw(p) });
    }
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_robot_destroy(p: *mut DyniboFloatingRobot) {
    if !p.is_null() {
        drop(unsafe { Box::from_raw(p) });
    }
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_name(p: *const DyniboRobot) -> *const c_char {
    unsafe { p.as_ref() }.map_or(ptr::null(), |x| x.name.as_ptr())
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_robot_name(
    p: *const DyniboFloatingRobot,
) -> *const c_char {
    unsafe { p.as_ref() }.map_or(ptr::null(), |x| x.name.as_ptr())
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_joint_count(p: *const DyniboRobot) -> usize {
    unsafe { p.as_ref() }.map_or(0, |x| x.inner.joint_count())
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_generalized_count(p: *const DyniboRobot) -> usize {
    unsafe { p.as_ref() }.map_or(0, |x| x.inner.joint_count())
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_link_count(p: *const DyniboRobot) -> usize {
    unsafe { p.as_ref() }.map_or(0, |x| x.inner.link_count())
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_robot_joint_count(p: *const DyniboFloatingRobot) -> usize {
    unsafe { p.as_ref() }.map_or(0, |x| x.inner.joint_count())
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_robot_generalized_count(
    p: *const DyniboFloatingRobot,
) -> usize {
    unsafe { p.as_ref() }.map_or(0, |x| x.inner.joint_count() + 6)
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_robot_link_count(p: *const DyniboFloatingRobot) -> usize {
    unsafe { p.as_ref() }.map_or(0, |x| x.inner.link_count())
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_set_base_frame(
    p: *mut DyniboRobot,
    frame: *const DyniboPose,
) -> DyniboStatus {
    call(|| {
        let robot = unsafe { required_mut(p, "robot") }?;
        robot.base_frame = frame_from_pose(unsafe { required_ref(frame, "frame") }?)?;
        Ok(())
    })
}
fn find_link(ids: &[LinkId], got: LinkId) -> usize {
    ids.iter()
        .position(|x| *x == got)
        .expect("link belongs to model")
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_link_id(
    p: *const DyniboRobot,
    name: *const c_char,
    out: *mut usize,
) -> DyniboStatus {
    call(|| {
        let robot = unsafe { required_ref(p, "robot") }?;
        let out = unsafe { required_mut(out, "output") }?;
        if name.is_null() {
            return Err(invalid("name must not be null"));
        }
        let name = unsafe { CStr::from_ptr(name) }
            .to_str()
            .map_err(|_| invalid("name must be valid UTF-8"))?;
        *out = find_link(
            &robot.link_ids,
            robot.inner.link_id(name).map_err(core_error)?,
        );
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_robot_link_id(
    p: *const DyniboFloatingRobot,
    name: *const c_char,
    out: *mut usize,
) -> DyniboStatus {
    call(|| {
        let robot = unsafe { required_ref(p, "robot") }?;
        let out = unsafe { required_mut(out, "output") }?;
        if name.is_null() {
            return Err(invalid("name must not be null"));
        }
        let name = unsafe { CStr::from_ptr(name) }
            .to_str()
            .map_err(|_| invalid("name must be valid UTF-8"))?;
        *out = find_link(
            &robot.link_ids,
            robot.inner.link_id(name).map_err(core_error)?,
        );
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_workspace_create(
    p: *const DyniboRobot,
    out: *mut *mut DyniboWorkspace,
) -> DyniboStatus {
    call(|| {
        let robot = unsafe { required_ref(p, "robot") }?;
        let out = unsafe { required_mut(out, "output") }?;
        *out = Box::into_raw(Box::new(DyniboWorkspace {
            inner: robot.inner.robot(),
            poses: vec![Frame::identity(); robot.inner.link_count()].into_boxed_slice(),
            converted_loads: make_loads(&robot.link_ids),
            load_positions: make_load_positions(&robot.link_ids),
        }));
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_workspace_create(
    p: *const DyniboFloatingRobot,
    out: *mut *mut DyniboFloatingWorkspace,
) -> DyniboStatus {
    call(|| {
        let robot = unsafe { required_ref(p, "robot") }?;
        let out = unsafe { required_mut(out, "output") }?;
        *out = Box::into_raw(Box::new(DyniboFloatingWorkspace {
            inner: robot.inner.floating_robot().map_err(core_error)?,
            poses: vec![Frame::identity(); robot.inner.link_count()].into_boxed_slice(),
            converted_loads: make_loads(&robot.link_ids),
            load_positions: make_load_positions(&robot.link_ids),
        }));
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_workspace_destroy(p: *mut DyniboWorkspace) {
    if !p.is_null() {
        drop(unsafe { Box::from_raw(p) });
    }
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_workspace_destroy(p: *mut DyniboFloatingWorkspace) {
    if !p.is_null() {
        drop(unsafe { Box::from_raw(p) });
    }
}
