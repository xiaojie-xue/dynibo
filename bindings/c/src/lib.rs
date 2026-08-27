//! Stable, typed C ABI for dynibo.
#![allow(clippy::missing_safety_doc, reason = "C contracts are in dynibo.h")]

use dynibo::{
    BaseState, ErrorCategory, FloatingRobot, Frame, IndexedLoad, InverseKinematicsOptions, LinkId,
    Robot, Twist, Wrench,
};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};
use std::{
    cell::RefCell,
    ffi::{CStr, CString, c_char},
    panic::{AssertUnwindSafe, catch_unwind},
    ptr,
};

thread_local! { static LAST_ERROR: RefCell<Vec<u8>> = RefCell::new(vec![0]); }

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DyniboStatus {
    Ok = 0,
    InvalidArgument = 1,
    ModelError = 2,
    Panic = 3,
    SolverError = 4,
}
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DyniboPose {
    pub translation: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}
impl Default for DyniboPose {
    fn default() -> Self {
        Self {
            translation: [0.0; 3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }
}
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DyniboTwist {
    pub angular: [f64; 3],
    pub linear: [f64; 3],
}
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DyniboBaseState {
    pub frame: DyniboPose,
    pub velocity: DyniboTwist,
    pub acceleration: DyniboTwist,
}
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DyniboLoad {
    pub link_id: usize,
    pub torque: [f64; 3],
    pub force: [f64; 3],
}
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DyniboIkOptions {
    pub max_iterations: usize,
    pub translation_tolerance: f64,
    pub rotation_tolerance: f64,
    pub damping: f64,
    pub max_step_norm: f64,
}
impl Default for DyniboIkOptions {
    fn default() -> Self {
        let x = InverseKinematicsOptions::default();
        Self {
            max_iterations: x.max_iterations,
            translation_tolerance: x.translation_tolerance,
            rotation_tolerance: x.rotation_tolerance,
            damping: x.damping,
            max_step_norm: x.max_step_norm,
        }
    }
}

/// Fixed-base model and metadata. The frame is persistent model state.
pub struct DyniboRobot {
    inner: Robot,
    base_frame: Frame,
    link_ids: Vec<LinkId>,
    name: CString,
}
/// Fixed-base calculation storage.
pub struct DyniboWorkspace {
    inner: Robot,
    indexed_loads: Box<[IndexedLoad]>,
}
/// Floating-base model and metadata. It intentionally contains no base state.
pub struct DyniboFloatingRobot {
    inner: FloatingRobot,
    link_ids: Vec<LinkId>,
    name: CString,
}
/// Floating-base calculation storage.
pub struct DyniboFloatingWorkspace {
    inner: FloatingRobot,
    indexed_loads: Box<[IndexedLoad]>,
}

type CResult<T> = Result<T, (DyniboStatus, String)>;
fn invalid(s: impl Into<String>) -> (DyniboStatus, String) {
    (DyniboStatus::InvalidArgument, s.into())
}
fn core_error(e: dynibo::Error) -> (DyniboStatus, String) {
    let status = match e.category() {
        ErrorCategory::InvalidInput => DyniboStatus::InvalidArgument,
        ErrorCategory::Model => DyniboStatus::ModelError,
        ErrorCategory::Solver => DyniboStatus::SolverError,
    };
    (status, e.to_string())
}
fn set_error(s: impl Into<String>) {
    LAST_ERROR.with(|slot| {
        let mut out = slot.borrow_mut();
        out.clear();
        out.extend(s.into().bytes().map(|b| if b == 0 { b' ' } else { b }));
        out.push(0);
    });
}
fn call(f: impl FnOnce() -> CResult<()>) -> DyniboStatus {
    LAST_ERROR.with(|slot| {
        let mut out = slot.borrow_mut();
        out.clear();
        out.push(0);
    });
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(())) => DyniboStatus::Ok,
        Ok(Err((status, message))) => {
            set_error(message);
            status
        }
        Err(_) => {
            set_error("panic caught at dynibo C ABI boundary");
            DyniboStatus::Panic
        }
    }
}
unsafe fn required_ref<'a, T>(p: *const T, n: &str) -> CResult<&'a T> {
    unsafe { p.as_ref() }.ok_or_else(|| invalid(format!("{n} must not be null")))
}
unsafe fn required_mut<'a, T>(p: *mut T, n: &str) -> CResult<&'a mut T> {
    unsafe { p.as_mut() }.ok_or_else(|| invalid(format!("{n} must not be null")))
}
unsafe fn input_slice<'a>(p: *const f64, n: usize, name: &str) -> CResult<&'a [f64]> {
    if n == 0 {
        Ok(&[])
    } else if p.is_null() {
        Err(invalid(format!("{name} must not be null")))
    } else {
        Ok(unsafe { std::slice::from_raw_parts(p, n) })
    }
}
unsafe fn output_slice<'a>(p: *mut f64, n: usize, name: &str) -> CResult<&'a mut [f64]> {
    if n == 0 {
        Ok(&mut [])
    } else if p.is_null() {
        Err(invalid(format!("{name} must not be null")))
    } else {
        Ok(unsafe { std::slice::from_raw_parts_mut(p, n) })
    }
}
/// Reject a C buffer layout that would create aliased Rust references.
///
/// This intentionally runs before `input_slice` and `output_slice`: creating
/// `&[f64]` and `&mut [f64]` from overlapping C buffers is already undefined
/// behavior, even if the calculation would subsequently reject its lengths.
fn reject_byte_overlap(
    input: *const u8,
    input_bytes: usize,
    input_name: &str,
    output: *mut u8,
    output_bytes: usize,
) -> CResult<()> {
    if input.is_null() || output.is_null() || input_bytes == 0 || output_bytes == 0 {
        return Ok(());
    }
    let input_start = input.addr();
    let output_start = output.addr();
    let input_end = input_start
        .checked_add(input_bytes)
        .ok_or_else(|| invalid(format!("{input_name} address range overflows")))?;
    let output_end = output_start
        .checked_add(output_bytes)
        .ok_or_else(|| invalid("output address range overflows"))?;
    if input_start < output_end && output_start < input_end {
        Err(invalid(format!("{input_name} and output must not overlap")))
    } else {
        Ok(())
    }
}

fn reject_f64_overlap(
    input: *const f64,
    input_len: usize,
    input_name: &str,
    output: *mut f64,
    output_len: usize,
) -> CResult<()> {
    let input_bytes = input_len
        .checked_mul(std::mem::size_of::<f64>())
        .ok_or_else(|| invalid(format!("{input_name} length is too large")))?;
    let output_bytes = output_len
        .checked_mul(std::mem::size_of::<f64>())
        .ok_or_else(|| invalid("output length is too large"))?;
    reject_byte_overlap(
        input.cast(),
        input_bytes,
        input_name,
        output.cast(),
        output_bytes,
    )
}

fn reject_struct_output_overlap<T>(
    input: *const f64,
    input_len: usize,
    input_name: &str,
    output: *mut T,
) -> CResult<()> {
    let input_bytes = input_len
        .checked_mul(std::mem::size_of::<f64>())
        .ok_or_else(|| invalid(format!("{input_name} length is too large")))?;
    reject_byte_overlap(
        input.cast(),
        input_bytes,
        input_name,
        output.cast(),
        std::mem::size_of::<T>(),
    )
}

macro_rules! reject_output_overlap {
    ($output:expr, $output_len:expr; $($input:expr, $input_len:expr, $input_name:expr);+ $(;)?) => {
        $(reject_f64_overlap($input, $input_len, $input_name, $output, $output_len)?;)+
    };
}

macro_rules! reject_struct_output_overlap {
    ($output:expr; $($input:expr, $input_len:expr, $input_name:expr);+ $(;)?) => {
        $(reject_struct_output_overlap($input, $input_len, $input_name, $output)?;)+
    };
}
fn frame_from_pose(p: &DyniboPose) -> CResult<Frame> {
    let [x, y, z, w] = p.rotation_xyzw;
    let norm = x * x + y * y + z * z + w * w;
    if !p.translation.iter().all(|x| x.is_finite()) || !norm.is_finite() || norm <= 1e-24 {
        return Err(invalid(
            "pose contains non-finite values or a zero quaternion",
        ));
    }
    Ok(Frame::from_parts(
        Translation3::from(Vector3::from(p.translation)),
        UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
    ))
}
fn pose_from_frame(f: &Frame) -> DyniboPose {
    let q = f.rotation.quaternion();
    DyniboPose {
        translation: f.translation.vector.into(),
        rotation_xyzw: [q.i, q.j, q.k, q.w],
    }
}
fn twist_from_c(t: DyniboTwist) -> Twist {
    Twist::new(Vector3::from(t.angular), Vector3::from(t.linear))
}
fn twist_to_c(t: Twist) -> DyniboTwist {
    DyniboTwist {
        angular: t.angular.into(),
        linear: t.linear.into(),
    }
}
fn base_from_c(b: &DyniboBaseState) -> CResult<BaseState> {
    BaseState::new(
        frame_from_pose(&b.frame)?,
        twist_from_c(b.velocity),
        twist_from_c(b.acceleration),
    )
    .map_err(core_error)
}
fn loads<'a>(
    ids: &[LinkId],
    output: &'a mut [IndexedLoad],
    p: *const DyniboLoad,
    n: usize,
) -> CResult<&'a [IndexedLoad]> {
    for load in output.iter_mut() {
        load.wrench = Wrench::zeros();
    }
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
        let old = output[load.link_id].wrench;
        output[load.link_id].wrench = Wrench::new(
            old.torque + Vector3::from(load.torque),
            old.force + Vector3::from(load.force),
        );
    }
    Ok(output)
}
fn fixed_parts<'a>(
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
fn floating_parts<'a>(
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
fn link(ids: &[LinkId], target: usize) -> CResult<LinkId> {
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
pub extern "C" fn dynibo_last_error_message() -> *const c_char {
    LAST_ERROR.with(|x| x.borrow().as_ptr().cast())
}
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_version() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0").as_ptr().cast()
}
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_ik_options_default() -> DyniboIkOptions {
    DyniboIkOptions::default()
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
        let inner = Robot::from_urdf(path).map_err(core_error)?;
        let (link_ids, name) = info(&inner, Robot::link_count, Robot::link_id_at, Robot::name)?;
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
        let inner = FloatingRobot::from_urdf(path).map_err(core_error)?;
        let (link_ids, name) = info(
            &inner,
            FloatingRobot::link_count,
            FloatingRobot::link_id_at,
            FloatingRobot::name,
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
    unsafe { p.as_ref() }.map_or(0, |x| x.inner.generalized_count())
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
    unsafe { p.as_ref() }.map_or(0, |x| x.inner.generalized_count())
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
            inner: robot.inner.fork(),
            indexed_loads: make_loads(&robot.link_ids),
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
            inner: robot.inner.fork(),
            indexed_loads: make_loads(&robot.link_ids),
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

#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_kinematics(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    n: usize,
    target: usize,
    out: *mut DyniboPose,
) -> DyniboStatus {
    call(|| {
        let (r, w) = fixed_parts(r, w)?;
        reject_struct_output_overlap!(out; q, n, "q");
        let q = unsafe { input_slice(q, n, "q") }?;
        let out = unsafe { required_mut(out, "output") }?;
        *out = pose_from_frame(
            &w.inner
                .forward_kinematics(q, link(&r.link_ids, target)?)
                .map_err(core_error)?,
        );
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_forward_kinematics(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    n: usize,
    target: usize,
    out: *mut DyniboPose,
) -> DyniboStatus {
    call(|| {
        let (r, w) = floating_parts(r, w)?;
        reject_struct_output_overlap!(out; q, n, "q");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let out = unsafe { required_mut(out, "output") }?;
        *out = pose_from_frame(
            &w.inner
                .forward_kinematics(&b, q, link(&r.link_ids, target)?)
                .map_err(core_error)?,
        );
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_jacobian(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    n: usize,
    target: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = fixed_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q");
        let q = unsafe { input_slice(q, n, "q") }?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .jacobian(q, link(&r.link_ids, target)?, out)
            .map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_jacobian(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    n: usize,
    target: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = floating_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .jacobian(&b, q, link(&r.link_ids, target)?, out)
            .map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_jacobian_derivative(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    n: usize,
    target: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = fixed_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd");
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .jacobian_derivative(q, qd, link(&r.link_ids, target)?, out)
            .map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_jacobian_derivative(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    qd: *const f64,
    n: usize,
    target: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = floating_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .jacobian_derivative(&b, q, qd, link(&r.link_ids, target)?, out)
            .map_err(core_error)
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_mass_matrix(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    n: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (_, w) = fixed_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q");
        let q = unsafe { input_slice(q, n, "q") }?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner.mass_matrix(q, out).map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_mass_matrix(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    n: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (_, w) = floating_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner.mass_matrix(&b, q, out).map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_velocity_product_forces(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    n: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (_, w) = fixed_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd");
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .velocity_product_forces(q, qd, out)
            .map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_velocity_product_forces(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    qd: *const f64,
    n: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (_, w) = floating_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .velocity_product_forces(&b, q, qd, out)
            .map_err(core_error)
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_velocity_kinematics(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    n: usize,
    target: usize,
    tool: *const DyniboPose,
    out: *mut DyniboTwist,
) -> DyniboStatus {
    call(|| {
        let (r, w) = fixed_parts(r, w)?;
        reject_struct_output_overlap!(out; q, n, "q"; qd, n, "qd");
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let tool = frame_from_pose(unsafe { required_ref(tool, "tool") }?)?;
        let out = unsafe { required_mut(out, "output") }?;
        *out = twist_to_c(
            w.inner
                .forward_velocity_kinematics(q, qd, link(&r.link_ids, target)?, &tool)
                .map_err(core_error)?,
        );
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_forward_velocity_kinematics(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    qd: *const f64,
    n: usize,
    target: usize,
    tool: *const DyniboPose,
    out: *mut DyniboTwist,
) -> DyniboStatus {
    call(|| {
        let (r, w) = floating_parts(r, w)?;
        reject_struct_output_overlap!(out; q, n, "q"; qd, n, "qd");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let tool = frame_from_pose(unsafe { required_ref(tool, "tool") }?)?;
        let out = unsafe { required_mut(out, "output") }?;
        *out = twist_to_c(
            w.inner
                .forward_velocity_kinematics(&b, q, qd, link(&r.link_ids, target)?, &tool)
                .map_err(core_error)?,
        );
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_acceleration_kinematics(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    n: usize,
    target: usize,
    out: *mut DyniboTwist,
) -> DyniboStatus {
    call(|| {
        let (r, w) = fixed_parts(r, w)?;
        reject_struct_output_overlap!(out; q, n, "q"; qd, n, "qd"; qdd, n, "qdd");
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let qdd = unsafe { input_slice(qdd, n, "qdd") }?;
        let out = unsafe { required_mut(out, "output") }?;
        *out = twist_to_c(
            w.inner
                .forward_acceleration_kinematics(q, qd, qdd, link(&r.link_ids, target)?)
                .map_err(core_error)?,
        );
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_forward_acceleration_kinematics(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    n: usize,
    target: usize,
    out: *mut DyniboTwist,
) -> DyniboStatus {
    call(|| {
        let (r, w) = floating_parts(r, w)?;
        reject_struct_output_overlap!(out; q, n, "q"; qd, n, "qd"; qdd, n, "qdd");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let qdd = unsafe { input_slice(qdd, n, "qdd") }?;
        let out = unsafe { required_mut(out, "output") }?;
        *out = twist_to_c(
            w.inner
                .forward_acceleration_kinematics(&b, q, qd, qdd, link(&r.link_ids, target)?)
                .map_err(core_error)?,
        );
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_gravity(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    n: usize,
    lp: *const DyniboLoad,
    ln: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = fixed_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q");
        let q = unsafe { input_slice(q, n, "q") }?;
        let ls = loads(&r.link_ids, &mut w.indexed_loads, lp, ln)?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner.gravity(q, ls, out).map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_gravity(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    n: usize,
    lp: *const DyniboLoad,
    ln: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = floating_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let ls = loads(&r.link_ids, &mut w.indexed_loads, lp, ln)?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner.gravity(&b, q, ls, out).map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_inverse_dynamics(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    n: usize,
    lp: *const DyniboLoad,
    ln: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = fixed_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd"; qdd, n, "qdd");
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let qdd = unsafe { input_slice(qdd, n, "qdd") }?;
        let ls = loads(&r.link_ids, &mut w.indexed_loads, lp, ln)?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .inverse_dynamics(q, qd, qdd, ls, out)
            .map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_inverse_dynamics(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    n: usize,
    lp: *const DyniboLoad,
    ln: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = floating_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd"; qdd, n, "qdd");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let qdd = unsafe { input_slice(qdd, n, "qdd") }?;
        let ls = loads(&r.link_ids, &mut w.indexed_loads, lp, ln)?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .inverse_dynamics(&b, q, qd, qdd, ls, out)
            .map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_dynamics(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    n: usize,
    f: *const f64,
    fn_: usize,
    lp: *const DyniboLoad,
    ln: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = fixed_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd"; f, fn_, "generalized_forces");
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let f = unsafe { input_slice(f, fn_, "generalized_forces") }?;
        let ls = loads(&r.link_ids, &mut w.indexed_loads, lp, ln)?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .forward_dynamics(q, qd, f, ls, out)
            .map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_forward_dynamics(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    qd: *const f64,
    n: usize,
    f: *const f64,
    fn_: usize,
    lp: *const DyniboLoad,
    ln: usize,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = floating_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd"; f, fn_, "generalized_forces");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let f = unsafe { input_slice(f, fn_, "generalized_forces") }?;
        let ls = loads(&r.link_ids, &mut w.indexed_loads, lp, ln)?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .forward_dynamics(&b, q, qd, f, ls, out)
            .map_err(core_error)
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_inverse_kinematics(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    n: usize,
    target: usize,
    desired: *const DyniboPose,
    options: DyniboIkOptions,
    out: *mut f64,
    on: usize,
) -> DyniboStatus {
    call(|| {
        let (r, w) = fixed_parts(r, w)?;
        reject_output_overlap!(out, on; q, n, "initial_q");
        let q = unsafe { input_slice(q, n, "initial_q") }?;
        let desired = frame_from_pose(unsafe { required_ref(desired, "desired") }?)?;
        let out = unsafe { output_slice(out, on, "output") }?;
        let options = InverseKinematicsOptions {
            max_iterations: options.max_iterations,
            translation_tolerance: options.translation_tolerance,
            rotation_tolerance: options.rotation_tolerance,
            damping: options.damping,
            max_step_norm: options.max_step_norm,
        };
        w.inner
            .inverse_kinematics(q, link(&r.link_ids, target)?, &desired, options, out)
            .map_err(core_error)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CString;

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

    #[test]
    fn fixed_and_floating_abi_success_paths_return_finite_results() {
        // SAFETY: Every opaque handle is created by this ABI, all slices remain
        // live for the duration of each call, and output buffers have the sizes
        // required by the fixed- and floating-base contracts.
        unsafe {
            let (robot, workspace, target) = fixed_handles();
            let (floating, floating_workspace, floating_target) = floating_handles();
            let q = [0.2, -0.3, 0.4, -0.1];
            let qd = [-0.1, 0.2, -0.3, 0.4];
            let qdd = [0.3, -0.2, 0.1, -0.4];
            let base = DyniboBaseState {
                frame: DyniboPose {
                    translation: [0.1, -0.2, 0.3],
                    ..DyniboPose::default()
                },
                velocity: DyniboTwist {
                    angular: [0.1, -0.2, 0.3],
                    linear: [-0.3, 0.2, 0.1],
                },
                acceleration: DyniboTwist {
                    angular: [-0.2, 0.1, 0.3],
                    linear: [0.4, -0.1, 0.2],
                },
            };
            let tool = DyniboPose {
                translation: [0.02, -0.01, 0.04],
                ..DyniboPose::default()
            };

            assert_eq!(
                CStr::from_ptr(dynibo_version()).to_bytes(),
                env!("CARGO_PKG_VERSION").as_bytes()
            );
            assert!(
                !CStr::from_ptr(dynibo_robot_name(robot))
                    .to_bytes()
                    .is_empty()
            );
            assert!(
                !CStr::from_ptr(dynibo_floating_robot_name(floating))
                    .to_bytes()
                    .is_empty()
            );
            assert_eq!(dynibo_robot_joint_count(robot), q.len());
            assert_eq!(dynibo_robot_generalized_count(robot), q.len());
            assert_eq!(dynibo_floating_robot_joint_count(floating), q.len());
            assert_eq!(
                dynibo_floating_robot_generalized_count(floating),
                q.len() + 6
            );
            assert_eq!(
                dynibo_robot_link_count(robot),
                dynibo_floating_robot_link_count(floating)
            );
            assert_eq!(
                dynibo_robot_set_base_frame(robot, &DyniboPose::default()),
                DyniboStatus::Ok
            );

            let mut fixed_pose = DyniboPose::default();
            assert_eq!(
                dynibo_forward_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.len(),
                    target,
                    &mut fixed_pose,
                ),
                DyniboStatus::Ok
            );
            let mut floating_pose = DyniboPose::default();
            assert_eq!(
                dynibo_floating_forward_kinematics(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    q.len(),
                    floating_target,
                    &mut floating_pose,
                ),
                DyniboStatus::Ok
            );

            let mut fixed_jacobian = [0.0; 24];
            assert_eq!(
                dynibo_jacobian(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.len(),
                    target,
                    fixed_jacobian.as_mut_ptr(),
                    fixed_jacobian.len(),
                ),
                DyniboStatus::Ok
            );
            let mut floating_jacobian = [0.0; 60];
            assert_eq!(
                dynibo_floating_jacobian(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    q.len(),
                    floating_target,
                    floating_jacobian.as_mut_ptr(),
                    floating_jacobian.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    target,
                    fixed_jacobian.as_mut_ptr(),
                    fixed_jacobian.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_floating_jacobian_derivative(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    floating_target,
                    floating_jacobian.as_mut_ptr(),
                    floating_jacobian.len(),
                ),
                DyniboStatus::Ok
            );

            let mut fixed_matrix = [0.0; 16];
            assert_eq!(
                dynibo_mass_matrix(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.len(),
                    fixed_matrix.as_mut_ptr(),
                    fixed_matrix.len(),
                ),
                DyniboStatus::Ok
            );
            let mut floating_matrix = [0.0; 100];
            assert_eq!(
                dynibo_floating_mass_matrix(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    q.len(),
                    floating_matrix.as_mut_ptr(),
                    floating_matrix.len(),
                ),
                DyniboStatus::Ok
            );

            let mut fixed_output = [0.0; 4];
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    fixed_output.as_mut_ptr(),
                    fixed_output.len(),
                ),
                DyniboStatus::Ok
            );
            let mut floating_output = [0.0; 10];
            assert_eq!(
                dynibo_floating_velocity_product_forces(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    floating_output.as_mut_ptr(),
                    floating_output.len(),
                ),
                DyniboStatus::Ok
            );

            let mut fixed_twist = DyniboTwist::default();
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    target,
                    &tool,
                    &mut fixed_twist,
                ),
                DyniboStatus::Ok
            );
            let mut floating_twist = DyniboTwist::default();
            assert_eq!(
                dynibo_floating_forward_velocity_kinematics(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    floating_target,
                    &tool,
                    &mut floating_twist,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_forward_acceleration_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    qdd.as_ptr(),
                    q.len(),
                    target,
                    &mut fixed_twist,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_floating_forward_acceleration_kinematics(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    qd.as_ptr(),
                    qdd.as_ptr(),
                    q.len(),
                    floating_target,
                    &mut floating_twist,
                ),
                DyniboStatus::Ok
            );

            assert_eq!(
                dynibo_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.len(),
                    ptr::null(),
                    0,
                    fixed_output.as_mut_ptr(),
                    fixed_output.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_floating_gravity(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    q.len(),
                    ptr::null(),
                    0,
                    floating_output.as_mut_ptr(),
                    floating_output.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    qdd.as_ptr(),
                    q.len(),
                    ptr::null(),
                    0,
                    fixed_output.as_mut_ptr(),
                    fixed_output.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_floating_inverse_dynamics(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    qd.as_ptr(),
                    qdd.as_ptr(),
                    q.len(),
                    ptr::null(),
                    0,
                    floating_output.as_mut_ptr(),
                    floating_output.len(),
                ),
                DyniboStatus::Ok
            );
            let fixed_forces = fixed_output;
            assert_eq!(
                dynibo_forward_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    fixed_forces.as_ptr(),
                    fixed_forces.len(),
                    ptr::null(),
                    0,
                    fixed_output.as_mut_ptr(),
                    fixed_output.len(),
                ),
                DyniboStatus::Ok
            );
            let floating_forces = floating_output;
            assert_eq!(
                dynibo_floating_forward_dynamics(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    qd.as_ptr(),
                    q.len(),
                    floating_forces.as_ptr(),
                    floating_forces.len(),
                    ptr::null(),
                    0,
                    floating_output.as_mut_ptr(),
                    floating_output.len(),
                ),
                DyniboStatus::Ok
            );

            let mut ik_output = [0.0; 4];
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.len(),
                    target,
                    &fixed_pose,
                    dynibo_ik_options_default(),
                    ik_output.as_mut_ptr(),
                    ik_output.len(),
                ),
                DyniboStatus::Ok
            );

            assert!(fixed_jacobian.iter().all(|x| x.is_finite()));
            assert!(floating_jacobian.iter().all(|x| x.is_finite()));
            assert!(fixed_matrix.iter().all(|x| x.is_finite()));
            assert!(floating_matrix.iter().all(|x| x.is_finite()));
            assert!(fixed_output.iter().all(|x| x.is_finite()));
            assert!(floating_output.iter().all(|x| x.is_finite()));
            assert!(ik_output.iter().all(|x| x.is_finite()));

            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(robot);
            dynibo_floating_workspace_destroy(floating_workspace);
            dynibo_floating_robot_destroy(floating);
            dynibo_workspace_destroy(ptr::null_mut());
            dynibo_floating_workspace_destroy(ptr::null_mut());
            dynibo_robot_destroy(ptr::null_mut());
            dynibo_floating_robot_destroy(ptr::null_mut());
        }
    }

    #[test]
    fn fixed_vector_calculations_reject_overlapping_output_before_slicing() {
        // SAFETY: Every pointer comes from a live allocation and the calls are
        // deliberately rejected before the declared output range is accessed.
        unsafe {
            let (robot, workspace, target) = fixed_handles();
            let mut q = [0.0; 4];
            let qd = [0.0; 4];
            let qdd = [0.0; 4];
            let forces = [0.0; 4];
            let overlap = q.as_mut_ptr();
            assert_eq!(
                dynibo_jacobian(robot, workspace, overlap, 4, target, overlap, 24),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    overlap,
                    qd.as_ptr(),
                    4,
                    target,
                    overlap,
                    24
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_mass_matrix(robot, workspace, overlap, 4, overlap, 16),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    overlap,
                    qd.as_ptr(),
                    4,
                    overlap,
                    4
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_gravity(robot, workspace, overlap, 4, ptr::null(), 0, overlap, 4),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    overlap,
                    qd.as_ptr(),
                    qdd.as_ptr(),
                    4,
                    ptr::null(),
                    0,
                    overlap,
                    4
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_dynamics(
                    robot,
                    workspace,
                    overlap,
                    qd.as_ptr(),
                    4,
                    forces.as_ptr(),
                    4,
                    ptr::null(),
                    0,
                    overlap,
                    4
                ),
                DyniboStatus::InvalidArgument
            );
            let desired = DyniboPose::default();
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    overlap,
                    4,
                    target,
                    &desired,
                    DyniboIkOptions::default(),
                    overlap,
                    4
                ),
                DyniboStatus::InvalidArgument
            );
            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(robot);
        }
    }

    #[test]
    fn floating_vector_calculations_reject_overlapping_output_and_invalid_base() {
        // SAFETY: Handles are valid; overlap calls return before constructing slices.
        unsafe {
            let (robot, workspace, target) = floating_handles();
            let mut q = [0.0; 4];
            let qd = [0.0; 4];
            let qdd = [0.0; 4];
            let forces = [0.0; 10];
            let base = DyniboBaseState::default();
            let overlap = q.as_mut_ptr();
            assert_eq!(
                dynibo_floating_jacobian(robot, workspace, &base, overlap, 4, target, overlap, 60),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_jacobian_derivative(
                    robot,
                    workspace,
                    &base,
                    overlap,
                    qd.as_ptr(),
                    4,
                    target,
                    overlap,
                    60
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_mass_matrix(robot, workspace, &base, overlap, 4, overlap, 100),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_velocity_product_forces(
                    robot,
                    workspace,
                    &base,
                    overlap,
                    qd.as_ptr(),
                    4,
                    overlap,
                    10
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_gravity(
                    robot,
                    workspace,
                    &base,
                    overlap,
                    4,
                    ptr::null(),
                    0,
                    overlap,
                    10
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_inverse_dynamics(
                    robot,
                    workspace,
                    &base,
                    overlap,
                    qd.as_ptr(),
                    qdd.as_ptr(),
                    4,
                    ptr::null(),
                    0,
                    overlap,
                    10
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_forward_dynamics(
                    robot,
                    workspace,
                    &base,
                    overlap,
                    qd.as_ptr(),
                    4,
                    forces.as_ptr(),
                    10,
                    ptr::null(),
                    0,
                    overlap,
                    10
                ),
                DyniboStatus::InvalidArgument
            );
            let invalid_base = DyniboBaseState {
                velocity: DyniboTwist {
                    angular: [f64::NAN, 0.0, 0.0],
                    ..DyniboTwist::default()
                },
                ..DyniboBaseState::default()
            };
            let mut pose = DyniboPose::default();
            assert_eq!(
                dynibo_floating_forward_kinematics(
                    robot,
                    workspace,
                    &invalid_base,
                    q.as_ptr(),
                    4,
                    target,
                    &mut pose
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_forward_kinematics(
                    robot,
                    workspace,
                    ptr::null(),
                    q.as_ptr(),
                    4,
                    target,
                    &mut pose
                ),
                DyniboStatus::InvalidArgument
            );
            dynibo_floating_workspace_destroy(workspace);
            dynibo_floating_robot_destroy(robot);
        }
    }

    #[test]
    fn pose_and_twist_outputs_reject_overlapping_joint_buffers() {
        // SAFETY: The invalid calls return from raw range validation before a
        // typed output reference or the declared oversized output is accessed.
        unsafe {
            let (robot, workspace, target) = fixed_handles();
            let (floating, floating_workspace, floating_target) = floating_handles();
            let mut q = [0.0; 4];
            let qd = [0.0; 4];
            let qdd = [0.0; 4];
            let pose = q.as_mut_ptr().cast::<DyniboPose>();
            let twist = q.as_mut_ptr().cast::<DyniboTwist>();
            let tool = DyniboPose::default();
            let base = DyniboBaseState::default();
            assert_eq!(
                dynibo_forward_kinematics(robot, workspace, q.as_ptr(), 4, target, pose),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    4,
                    target,
                    &tool,
                    twist
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_acceleration_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    qdd.as_ptr(),
                    4,
                    target,
                    twist
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_forward_kinematics(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    4,
                    floating_target,
                    pose
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_forward_velocity_kinematics(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    qd.as_ptr(),
                    4,
                    floating_target,
                    &tool,
                    twist
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_floating_forward_acceleration_kinematics(
                    floating,
                    floating_workspace,
                    &base,
                    q.as_ptr(),
                    qd.as_ptr(),
                    qdd.as_ptr(),
                    4,
                    floating_target,
                    twist
                ),
                DyniboStatus::InvalidArgument
            );
            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(robot);
            dynibo_floating_workspace_destroy(floating_workspace);
            dynibo_floating_robot_destroy(floating);
        }
    }

    #[test]
    fn typed_workspaces_reject_foreign_models_and_errors_are_thread_local() {
        // SAFETY: All opaque handles below are allocated by this ABI.
        unsafe {
            let (robot, workspace, target) = fixed_handles();
            let (foreign, foreign_workspace, _) = fixed_handles();
            let q = [0.0; 4];
            let mut pose = DyniboPose::default();
            assert_eq!(
                dynibo_forward_kinematics(
                    robot,
                    foreign_workspace,
                    q.as_ptr(),
                    q.len(),
                    target,
                    &mut pose
                ),
                DyniboStatus::InvalidArgument
            );
            assert!(
                !CStr::from_ptr(dynibo_last_error_message())
                    .to_bytes()
                    .is_empty()
            );
            assert_eq!(
                dynibo_forward_kinematics(robot, workspace, q.as_ptr(), q.len(), target, &mut pose),
                DyniboStatus::Ok
            );
            assert!(
                CStr::from_ptr(dynibo_last_error_message())
                    .to_bytes()
                    .is_empty()
            );
            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(robot);
            dynibo_workspace_destroy(foreign_workspace);
            dynibo_robot_destroy(foreign);
        }

        let barrier = std::sync::Arc::new(std::sync::Barrier::new(2));
        let first_barrier = std::sync::Arc::clone(&barrier);
        let first = std::thread::spawn(move || {
            let mut output = ptr::null_mut();
            assert_eq!(
                unsafe { dynibo_robot_from_urdf(ptr::null(), &mut output) },
                DyniboStatus::InvalidArgument
            );
            first_barrier.wait();
            unsafe { CStr::from_ptr(dynibo_last_error_message()) }
                .to_string_lossy()
                .into_owned()
        });
        let second_barrier = std::sync::Arc::clone(&barrier);
        let second = std::thread::spawn(move || {
            assert_eq!(
                unsafe { dynibo_robot_set_base_frame(ptr::null_mut(), ptr::null()) },
                DyniboStatus::InvalidArgument
            );
            second_barrier.wait();
            unsafe { CStr::from_ptr(dynibo_last_error_message()) }
                .to_string_lossy()
                .into_owned()
        });
        assert!(first.join().unwrap().contains("path must not be null"));
        assert!(second.join().unwrap().contains("robot must not be null"));

        assert_eq!(
            call(|| -> CResult<()> { panic!("test panic") }),
            DyniboStatus::Panic
        );
        assert!(
            unsafe { CStr::from_ptr(dynibo_last_error_message()) }
                .to_string_lossy()
                .contains("panic caught")
        );
    }
}
