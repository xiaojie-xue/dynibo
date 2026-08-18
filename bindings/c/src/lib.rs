//! Stable C ABI for `dynibo`.

#![allow(
    clippy::missing_safety_doc,
    reason = "pointer ownership and validity contracts are documented in dynibo.h"
)]

use std::{
    cell::RefCell,
    ffi::{CStr, CString, c_char},
    panic::{AssertUnwindSafe, catch_unwind},
    ptr,
};

use dynibo::{
    BaseMode, ErrorCategory, Frame, IndexedLoad, InverseKinematicsOptions, LinkId, Robot, Twist,
    Workspace, Wrench,
};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};

thread_local! {
    static LAST_ERROR: RefCell<CString> = RefCell::new(CString::default());
}

/// Status returned by every fallible C function.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DyniboStatus {
    /// The operation succeeded.
    Ok = 0,
    /// A pointer, length, UTF-8 string, or quaternion was invalid.
    InvalidArgument = 1,
    /// A robot description could not be loaded or represented.
    ModelError = 2,
    /// A Rust panic was caught at the ABI boundary.
    Panic = 3,
    /// An iterative numerical calculation failed to produce a valid result.
    SolverError = 4,
}

/// Root-link connection mode.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DyniboBaseMode {
    /// The root link is fixed to the world.
    #[default]
    Fixed = 0,
    /// The root link has six generalized velocity coordinates.
    Floating = 1,
}

impl From<DyniboBaseMode> for BaseMode {
    fn from(value: DyniboBaseMode) -> Self {
        match value {
            DyniboBaseMode::Fixed => Self::Fixed,
            DyniboBaseMode::Floating => Self::Floating,
        }
    }
}

/// Translation plus an `(x, y, z, w)` unit quaternion.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DyniboPose {
    /// Translation in metres.
    pub translation: [f64; 3],
    /// Quaternion ordered `(x, y, z, w)`.
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

/// Angular-first spatial vector.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DyniboTwist {
    /// Angular component.
    pub angular: [f64; 3],
    /// Linear component.
    pub linear: [f64; 3],
}

/// External wrench applied at a link origin and expressed in that link frame.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DyniboLoad {
    /// Model-scoped link handle returned by `dynibo_robot_link_id`.
    pub link_id: usize,
    /// Torque component.
    pub torque: [f64; 3],
    /// Force component.
    pub force: [f64; 3],
}

/// Damped-least-squares inverse-kinematics configuration.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DyniboIkOptions {
    /// Maximum number of updates.
    pub max_iterations: usize,
    /// Translation tolerance in metres.
    pub translation_tolerance: f64,
    /// Rotation-vector tolerance in radians.
    pub rotation_tolerance: f64,
    /// Damping factor.
    pub damping: f64,
    /// Maximum norm of one update.
    pub max_step_norm: f64,
}

impl Default for DyniboIkOptions {
    fn default() -> Self {
        let value = InverseKinematicsOptions::default();
        Self {
            max_iterations: value.max_iterations,
            translation_tolerance: value.translation_tolerance,
            rotation_tolerance: value.rotation_tolerance,
            damping: value.damping,
            max_step_norm: value.max_step_norm,
        }
    }
}

/// Opaque robot model.
pub struct DyniboRobot {
    inner: Robot,
    link_ids: Vec<LinkId>,
    name: CString,
}

/// Opaque reusable calculation workspace.
pub struct DyniboWorkspace {
    inner: Workspace,
}

fn set_error(message: impl Into<String>) {
    let message = message.into().replace('\0', "\\0");
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = CString::new(message).expect("NUL bytes were replaced");
    });
}

fn call(function: impl FnOnce() -> Result<(), (DyniboStatus, String)>) -> DyniboStatus {
    LAST_ERROR.with(|slot| *slot.borrow_mut() = CString::default());
    match catch_unwind(AssertUnwindSafe(function)) {
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

fn invalid(message: impl Into<String>) -> (DyniboStatus, String) {
    (DyniboStatus::InvalidArgument, message.into())
}

fn core_error(error: dynibo::Error) -> (DyniboStatus, String) {
    let status = match error.category() {
        ErrorCategory::InvalidInput => DyniboStatus::InvalidArgument,
        ErrorCategory::Model => DyniboStatus::ModelError,
        ErrorCategory::Solver => DyniboStatus::SolverError,
    };
    (status, error.to_string())
}

fn model_error(message: impl Into<String>) -> (DyniboStatus, String) {
    (DyniboStatus::ModelError, message.into())
}

unsafe fn required_ref<'a, T>(
    pointer: *const T,
    name: &str,
) -> Result<&'a T, (DyniboStatus, String)> {
    // SAFETY: The caller of the C ABI promises that non-null pointers are valid.
    unsafe { pointer.as_ref() }.ok_or_else(|| invalid(format!("{name} must not be null")))
}

unsafe fn required_mut<'a, T>(
    pointer: *mut T,
    name: &str,
) -> Result<&'a mut T, (DyniboStatus, String)> {
    // SAFETY: The caller of the C ABI promises that non-null pointers are valid and unique.
    unsafe { pointer.as_mut() }.ok_or_else(|| invalid(format!("{name} must not be null")))
}

unsafe fn input_slice<'a>(
    pointer: *const f64,
    length: usize,
    name: &str,
) -> Result<&'a [f64], (DyniboStatus, String)> {
    if length == 0 {
        return Ok(&[]);
    }
    if pointer.is_null() {
        return Err(invalid(format!("{name} must not be null")));
    }
    // SAFETY: Validity for `length` readable elements is part of the C contract.
    Ok(unsafe { std::slice::from_raw_parts(pointer, length) })
}

unsafe fn output_slice<'a>(
    pointer: *mut f64,
    length: usize,
    name: &str,
) -> Result<&'a mut [f64], (DyniboStatus, String)> {
    if length == 0 {
        return Ok(&mut []);
    }
    if pointer.is_null() {
        return Err(invalid(format!("{name} must not be null")));
    }
    // SAFETY: Validity and uniqueness for `length` elements is part of the C contract.
    Ok(unsafe { std::slice::from_raw_parts_mut(pointer, length) })
}

fn reject_f64_overlap(
    input: *const f64,
    input_length: usize,
    input_name: &str,
    output: *mut f64,
    output_length: usize,
) -> Result<(), (DyniboStatus, String)> {
    if input.is_null() || output.is_null() || input_length == 0 || output_length == 0 {
        return Ok(());
    }
    let element_size = std::mem::size_of::<f64>();
    let input_bytes = input_length
        .checked_mul(element_size)
        .ok_or_else(|| invalid(format!("{input_name} length is too large")))?;
    let output_bytes = output_length
        .checked_mul(element_size)
        .ok_or_else(|| invalid("output length is too large"))?;
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

fn frame_from_pose(pose: &DyniboPose) -> Result<Frame, (DyniboStatus, String)> {
    let [x, y, z, w] = pose.rotation_xyzw;
    let norm_squared = x * x + y * y + z * z + w * w;
    if !pose.translation.iter().all(|value| value.is_finite())
        || !norm_squared.is_finite()
        || norm_squared <= 1.0e-24
    {
        return Err(invalid(
            "pose contains non-finite values or a zero quaternion",
        ));
    }
    Ok(Frame::from_parts(
        Translation3::from(Vector3::from(pose.translation)),
        UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
    ))
}

fn pose_from_frame(frame: &Frame) -> DyniboPose {
    let quaternion = frame.rotation.quaternion();
    DyniboPose {
        translation: frame.translation.vector.into(),
        rotation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    }
}

fn twist_from_c(value: DyniboTwist) -> Twist {
    Twist::new(Vector3::from(value.angular), Vector3::from(value.linear))
}

fn twist_to_c(value: Twist) -> DyniboTwist {
    DyniboTwist {
        angular: value.angular.into(),
        linear: value.linear.into(),
    }
}

unsafe fn load_slice(
    robot: &DyniboRobot,
    pointer: *const DyniboLoad,
    length: usize,
) -> Result<Vec<IndexedLoad>, (DyniboStatus, String)> {
    let loads = if length == 0 {
        &[]
    } else {
        if pointer.is_null() {
            return Err(invalid(
                "loads must not be null when load_count is non-zero",
            ));
        }
        // SAFETY: Validity for `length` readable elements is part of the C contract.
        unsafe { std::slice::from_raw_parts(pointer, length) }
    };
    loads
        .iter()
        .map(|load| {
            let link = robot
                .link_ids
                .get(load.link_id)
                .copied()
                .ok_or_else(|| invalid(format!("invalid link id {}", load.link_id)))?;
            Ok(IndexedLoad {
                link,
                wrench: Wrench::new(Vector3::from(load.torque), Vector3::from(load.force)),
            })
        })
        .collect()
}

/// Returns the last error message for the calling thread.
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_last_error_message() -> *const c_char {
    LAST_ERROR.with(|slot| slot.borrow().as_ptr())
}

/// Returns the linked ABI version string.
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_version() -> *const c_char {
    c"0.2.0".as_ptr()
}

/// Returns default inverse-kinematics options.
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_ik_options_default() -> DyniboIkOptions {
    DyniboIkOptions::default()
}

/// Loads a URDF and allocates a robot handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_load_urdf(
    path: *const c_char,
    output: *mut *mut DyniboRobot,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helper.
        let output = unsafe { required_mut(output, "output") }?;
        *output = ptr::null_mut();
        if path.is_null() {
            return Err(invalid("path must not be null"));
        }
        // SAFETY: A NUL-terminated string is part of the C contract.
        let path = unsafe { CStr::from_ptr(path) }
            .to_str()
            .map_err(|_| invalid("path must be valid UTF-8"))?;
        let inner = Robot::from_urdf(path).map_err(core_error)?;
        let link_ids = inner
            .links()
            .iter()
            .map(|link| inner.link_id(link.name()).expect("link came from robot"))
            .collect();
        let name =
            CString::new(inner.name()).map_err(|_| model_error("robot name contains NUL"))?;
        *output = Box::into_raw(Box::new(DyniboRobot {
            inner,
            link_ids,
            name,
        }));
        Ok(())
    })
}

/// Loads a URDF with an explicit root-link mode and allocates a robot handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_load_urdf_with_base(
    path: *const c_char,
    base_mode: DyniboBaseMode,
    output: *mut *mut DyniboRobot,
) -> DyniboStatus {
    call(|| {
        let output = unsafe { required_mut(output, "output") }?;
        *output = ptr::null_mut();
        if path.is_null() {
            return Err(invalid("path must not be null"));
        }
        let path = unsafe { CStr::from_ptr(path) }
            .to_str()
            .map_err(|_| invalid("path must be valid UTF-8"))?;
        let inner = Robot::from_urdf_with_base(path, base_mode.into()).map_err(core_error)?;
        let link_ids = inner
            .links()
            .iter()
            .map(|link| inner.link_id(link.name()).expect("link came from robot"))
            .collect();
        let name =
            CString::new(inner.name()).map_err(|_| model_error("robot name contains NUL"))?;
        *output = Box::into_raw(Box::new(DyniboRobot {
            inner,
            link_ids,
            name,
        }));
        Ok(())
    })
}

/// Destroys a robot handle. Passing null is allowed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_destroy(robot: *mut DyniboRobot) {
    if !robot.is_null() {
        // SAFETY: The pointer was returned by `Box::into_raw` and is owned by the caller.
        drop(unsafe { Box::from_raw(robot) });
    }
}

/// Returns the URDF robot name, valid until the robot is destroyed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_name(robot: *const DyniboRobot) -> *const c_char {
    // SAFETY: Reading a valid opaque handle is part of the C contract.
    unsafe { robot.as_ref() }.map_or(ptr::null(), |robot| robot.name.as_ptr())
}

/// Returns the number of joints, or zero for a null handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_joint_count(robot: *const DyniboRobot) -> usize {
    // SAFETY: Reading a valid opaque handle is part of the C contract.
    unsafe { robot.as_ref() }.map_or(0, |robot| robot.inner.joint_count())
}

/// Returns the generalized-vector size, or zero for a null handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_generalized_count(robot: *const DyniboRobot) -> usize {
    unsafe { robot.as_ref() }.map_or(0, |robot| robot.inner.generalized_count())
}

/// Replaces the complete floating-base state.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_set_base_state(
    robot: *mut DyniboRobot,
    frame: *const DyniboPose,
    velocity: DyniboTwist,
    acceleration: DyniboTwist,
) -> DyniboStatus {
    call(|| {
        let robot = unsafe { required_mut(robot, "robot") }?;
        let frame = frame_from_pose(unsafe { required_ref(frame, "frame") }?)?;
        robot
            .inner
            .set_floating_base_state(frame, twist_from_c(velocity), twist_from_c(acceleration))
            .map_err(core_error)
    })
}

/// Returns the number of links, or zero for a null handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_link_count(robot: *const DyniboRobot) -> usize {
    // SAFETY: Reading a valid opaque handle is part of the C contract.
    unsafe { robot.as_ref() }.map_or(0, |robot| robot.inner.link_count())
}

/// Resolves a link name to a model-scoped handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_link_id(
    robot: *const DyniboRobot,
    name: *const c_char,
    output: *mut usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helper.
        let output = unsafe { required_mut(output, "output") }?;
        if name.is_null() {
            return Err(invalid("name must not be null"));
        }
        // SAFETY: A NUL-terminated string is part of the C contract.
        let name = unsafe { CStr::from_ptr(name) }
            .to_str()
            .map_err(|_| invalid("name must be valid UTF-8"))?;
        let link_id = robot.inner.link_id(name).map_err(core_error)?;
        *output = robot
            .link_ids
            .iter()
            .position(|candidate| *candidate == link_id)
            .expect("resolved link is cached");
        Ok(())
    })
}

/// Allocates a reusable workspace for a robot.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_workspace_create(
    robot: *const DyniboRobot,
    output: *mut *mut DyniboWorkspace,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helper.
        let output = unsafe { required_mut(output, "output") }?;
        *output = Box::into_raw(Box::new(DyniboWorkspace {
            inner: robot.inner.workspace(),
        }));
        Ok(())
    })
}

/// Destroys a workspace handle. Passing null is allowed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_workspace_destroy(workspace: *mut DyniboWorkspace) {
    if !workspace.is_null() {
        // SAFETY: The pointer was returned by `Box::into_raw` and is owned by the caller.
        drop(unsafe { Box::from_raw(workspace) });
    }
}

/// Computes forward kinematics for one link.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_kinematics(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    q_len: usize,
    target: usize,
    output: *mut DyniboPose,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, q_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { required_mut(output, "output") }?;
        let link = robot
            .link_ids
            .get(target)
            .copied()
            .ok_or_else(|| invalid(format!("invalid link id {target}")))?;
        let frame = robot
            .inner
            .forward_kinematics(q, link, &mut workspace.inner)
            .map_err(core_error)?;
        *output = pose_from_frame(&frame);
        Ok(())
    })
}

/// Writes the column-major `6 x joint_count` geometric Jacobian.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_jacobian(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    q_len: usize,
    target: usize,
    output: *mut f64,
    output_len: usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, q_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        let link = robot
            .link_ids
            .get(target)
            .copied()
            .ok_or_else(|| invalid(format!("invalid link id {target}")))?;
        robot
            .inner
            .jacobian(q, link, &mut workspace.inner, output)
            .map_err(core_error)
    })
}

/// Writes the column-major `6 x joint_count` Jacobian time derivative.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_jacobian_derivative(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    state_len: usize,
    target: usize,
    output: *mut f64,
    output_len: usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, state_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qd = unsafe { input_slice(qd, state_len, "qd") }?;
        reject_f64_overlap(q.as_ptr(), q.len(), "q", output, output_len)?;
        reject_f64_overlap(qd.as_ptr(), qd.len(), "qd", output, output_len)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        let link = robot
            .link_ids
            .get(target)
            .copied()
            .ok_or_else(|| invalid(format!("invalid link id {target}")))?;
        robot
            .inner
            .jacobian_derivative(q, qd, link, &mut workspace.inner, output)
            .map_err(core_error)
    })
}

/// Writes the column-major `joint_count x joint_count` mass matrix.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_mass_matrix(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    q_len: usize,
    output: *mut f64,
    output_len: usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, q_len, "q") }?;
        reject_f64_overlap(q.as_ptr(), q.len(), "q", output, output_len)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        robot
            .inner
            .mass_matrix(q, &mut workspace.inner, output)
            .map_err(core_error)
    })
}

/// Writes the column-major `joint_count x joint_count` Coriolis matrix.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_coriolis_matrix(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    state_len: usize,
    output: *mut f64,
    output_len: usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, state_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qd = unsafe { input_slice(qd, state_len, "qd") }?;
        reject_f64_overlap(q.as_ptr(), q.len(), "q", output, output_len)?;
        reject_f64_overlap(qd.as_ptr(), qd.len(), "qd", output, output_len)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        robot
            .inner
            .coriolis_matrix(q, qd, &mut workspace.inner, output)
            .map_err(core_error)
    })
}

/// Solves inverse kinematics for one link.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_inverse_kinematics(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    initial_q: *const f64,
    q_len: usize,
    target: usize,
    desired: *const DyniboPose,
    options: DyniboIkOptions,
    output: *mut f64,
    output_len: usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let initial_q = unsafe { input_slice(initial_q, q_len, "initial_q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let desired = frame_from_pose(unsafe { required_ref(desired, "desired") }?)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        let link = robot
            .link_ids
            .get(target)
            .copied()
            .ok_or_else(|| invalid(format!("invalid link id {target}")))?;
        robot
            .inner
            .inverse_kinematics(
                initial_q,
                link,
                &desired,
                InverseKinematicsOptions {
                    max_iterations: options.max_iterations,
                    translation_tolerance: options.translation_tolerance,
                    rotation_tolerance: options.rotation_tolerance,
                    damping: options.damping,
                    max_step_norm: options.max_step_norm,
                },
                &mut workspace.inner,
                output,
            )
            .map_err(core_error)
    })
}

/// Computes target-link/tool spatial velocity.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_velocity(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    state_len: usize,
    target: usize,
    base: *const DyniboPose,
    tool: *const DyniboPose,
    output: *mut DyniboTwist,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, state_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qd = unsafe { input_slice(qd, state_len, "qd") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let base = frame_from_pose(unsafe { required_ref(base, "base") }?)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let tool = frame_from_pose(unsafe { required_ref(tool, "tool") }?)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { required_mut(output, "output") }?;
        let link = robot
            .link_ids
            .get(target)
            .copied()
            .ok_or_else(|| invalid(format!("invalid link id {target}")))?;
        let value = if robot.inner.base_mode() == BaseMode::Floating {
            robot
                .inner
                .forward_velocity_kinematics(q, qd, link, &tool, &mut workspace.inner)
        } else {
            let mut calculation_robot = robot.inner.clone();
            calculation_robot.set_base_frame(base).map_err(core_error)?;
            calculation_robot.forward_velocity_kinematics(q, qd, link, &tool, &mut workspace.inner)
        }
        .map_err(core_error)?;
        *output = twist_to_c(value);
        Ok(())
    })
}

/// Computes target-link-origin spatial acceleration.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_acceleration(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    state_len: usize,
    target: usize,
    output: *mut DyniboTwist,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, state_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qd = unsafe { input_slice(qd, state_len, "qd") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qdd = unsafe { input_slice(qdd, state_len, "qdd") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { required_mut(output, "output") }?;
        let link = robot
            .link_ids
            .get(target)
            .copied()
            .ok_or_else(|| invalid(format!("invalid link id {target}")))?;
        let value = robot
            .inner
            .forward_acceleration_kinematics(q, qd, qdd, link, &mut workspace.inner)
            .map_err(core_error)?;
        *output = twist_to_c(value);
        Ok(())
    })
}

/// Writes gravity and external-load joint forces.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_gravity(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    q_len: usize,
    base: *const DyniboPose,
    loads: *const DyniboLoad,
    load_count: usize,
    output: *mut f64,
    output_len: usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, q_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let base = frame_from_pose(unsafe { required_ref(base, "base") }?)?;
        // SAFETY: Pointer validation is performed by the helper.
        let loads = unsafe { load_slice(robot, loads, load_count) }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        if robot.inner.base_mode() == BaseMode::Floating {
            robot
                .inner
                .gravity(q, &loads, &mut workspace.inner, output)
                .map_err(core_error)
        } else {
            let mut calculation_robot = robot.inner.clone();
            calculation_robot.set_base_frame(base).map_err(core_error)?;
            calculation_robot
                .gravity(q, &loads, &mut workspace.inner, output)
                .map_err(core_error)
        }
    })
}

/// Writes Newton-Euler inverse-dynamics joint forces.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn dynibo_inverse_dynamics(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    state_len: usize,
    base: *const DyniboPose,
    _base_velocity: DyniboTwist,
    _base_acceleration: DyniboTwist,
    loads: *const DyniboLoad,
    load_count: usize,
    output: *mut f64,
    output_len: usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, state_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qd = unsafe { input_slice(qd, state_len, "qd") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qdd = unsafe { input_slice(qdd, state_len, "qdd") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let base = frame_from_pose(unsafe { required_ref(base, "base") }?)?;
        // SAFETY: Pointer validation is performed by the helper.
        let loads = unsafe { load_slice(robot, loads, load_count) }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        if robot.inner.base_mode() == BaseMode::Floating {
            robot
                .inner
                .inverse_dynamics(q, qd, qdd, &loads, &mut workspace.inner, output)
                .map_err(core_error)
        } else {
            let mut calculation_robot = robot.inner.clone();
            calculation_robot.set_base_frame(base).map_err(core_error)?;
            calculation_robot
                .inverse_dynamics(q, qd, qdd, &loads, &mut workspace.inner, output)
                .map_err(core_error)
        }
    })
}

#[cfg(test)]
mod tests {
    use std::{ffi::CString, ptr, thread};

    use super::*;

    fn fixture_path() -> CString {
        CString::new(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../tests/data/test_arm.urdf")
                .to_string_lossy()
                .as_bytes(),
        )
        .unwrap()
    }

    fn last_error() -> String {
        // SAFETY: the function always returns a valid thread-local C string.
        unsafe {
            CStr::from_ptr(dynibo_last_error_message())
                .to_string_lossy()
                .into_owned()
        }
    }

    unsafe fn create_handles() -> (*mut DyniboRobot, *mut DyniboWorkspace, usize) {
        let mut robot = ptr::null_mut();
        // SAFETY: all pointers refer to live local storage or a valid C string.
        assert_eq!(
            unsafe { dynibo_robot_load_urdf(fixture_path().as_ptr(), &mut robot) },
            DyniboStatus::Ok
        );
        let mut workspace = ptr::null_mut();
        // SAFETY: `robot` was created above and output is writable.
        assert_eq!(
            unsafe { dynibo_workspace_create(robot, &mut workspace) },
            DyniboStatus::Ok
        );
        let mut target = usize::MAX;
        // SAFETY: handles, string, and output are valid.
        assert_eq!(
            unsafe { dynibo_robot_link_id(robot, c"test_link_4".as_ptr(), &mut target) },
            DyniboStatus::Ok
        );
        (robot, workspace, target)
    }

    #[test]
    fn metadata_and_construction_reject_invalid_arguments() {
        // SAFETY: null is explicitly supported by the metadata/destructor functions.
        unsafe {
            assert_eq!(CStr::from_ptr(dynibo_version()), c"0.2.0");
            assert!(dynibo_robot_name(ptr::null()).is_null());
            assert_eq!(dynibo_robot_joint_count(ptr::null()), 0);
            assert_eq!(dynibo_robot_link_count(ptr::null()), 0);
            dynibo_robot_destroy(ptr::null_mut());
            dynibo_workspace_destroy(ptr::null_mut());

            let path = fixture_path();
            assert_eq!(
                dynibo_robot_load_urdf(path.as_ptr(), ptr::null_mut()),
                DyniboStatus::InvalidArgument
            );
            let mut robot = ptr::dangling_mut::<DyniboRobot>();
            assert_eq!(
                dynibo_robot_load_urdf(ptr::null(), &mut robot),
                DyniboStatus::InvalidArgument
            );
            assert!(robot.is_null());
            let invalid_utf8 = [0xff_u8, 0];
            assert_eq!(
                dynibo_robot_load_urdf(invalid_utf8.as_ptr().cast(), &mut robot),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_workspace_create(ptr::null(), ptr::null_mut()),
                DyniboStatus::InvalidArgument
            );
        }
    }

    #[test]
    fn link_and_workspace_functions_validate_every_pointer_class() {
        // SAFETY: valid handles are created and destroyed exactly once; deliberately null
        // pointers are passed only to functions that validate them before dereferencing.
        unsafe {
            let (robot, workspace, _) = create_handles();
            let mut target = 0;
            assert_eq!(
                dynibo_robot_link_id(ptr::null(), c"test_link_4".as_ptr(), &mut target),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_robot_link_id(robot, ptr::null(), &mut target),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_robot_link_id(robot, c"test_link_4".as_ptr(), ptr::null_mut()),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_robot_link_id(robot, c"missing".as_ptr(), &mut target),
                DyniboStatus::InvalidArgument
            );
            let invalid_utf8 = [0xff_u8, 0];
            assert_eq!(
                dynibo_robot_link_id(robot, invalid_utf8.as_ptr().cast(), &mut target),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_workspace_create(robot, ptr::null_mut()),
                DyniboStatus::InvalidArgument
            );
            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(robot);
        }
    }

    #[test]
    fn calculation_abi_covers_success_and_defensive_paths() {
        // SAFETY: valid buffers satisfy the documented ABI contracts; null pointers are used
        // only for validation cases where the callee checks them before constructing slices.
        unsafe {
            let (robot, workspace, target) = create_handles();
            let q = [0.0; 4];
            let mut pose = DyniboPose::default();
            let mut twist = DyniboTwist::default();
            let mut jacobian = [0.0; 24];
            let mut jacobian_derivative = [0.0; 24];
            let mut square = [0.0; 16];
            let mut output = [0.0; 4];

            assert_eq!(
                dynibo_forward_kinematics(robot, workspace, q.as_ptr(), 4, target, &mut pose),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_jacobian(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    jacobian.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    jacobian_derivative.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_mass_matrix(robot, workspace, q.as_ptr(), 4, square.as_mut_ptr(), 16,),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    16,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &pose,
                    dynibo_ik_options_default(),
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &DyniboPose::default(),
                    &DyniboPose::default(),
                    &mut twist,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_forward_acceleration(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &mut twist,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    &DyniboPose::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::Ok
            );
            let gravity = output;
            let load = DyniboLoad {
                link_id: target,
                force: [0.0, 1.0, 0.0],
                torque: [0.0, 0.0, 0.5],
            };
            assert_eq!(
                dynibo_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    &DyniboPose::default(),
                    &load,
                    1,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::Ok
            );
            assert_ne!(output, gravity);
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    &DyniboPose::default(),
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::Ok
            );

            assert_eq!(
                dynibo_forward_kinematics(ptr::null(), workspace, q.as_ptr(), 4, target, &mut pose,),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_kinematics(robot, ptr::null_mut(), q.as_ptr(), 4, target, &mut pose),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_kinematics(robot, workspace, ptr::null(), 4, target, &mut pose),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_kinematics(robot, workspace, q.as_ptr(), 4, usize::MAX, &mut pose,),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_kinematics(robot, workspace, q.as_ptr(), 4, target, ptr::null_mut(),),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian(robot, workspace, q.as_ptr(), 4, target, ptr::null_mut(), 24,),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    jacobian.as_mut_ptr(),
                    23,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian(
                    robot,
                    workspace,
                    ptr::null(),
                    0,
                    target,
                    jacobian.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian(robot, workspace, q.as_ptr(), 4, target, ptr::null_mut(), 0,),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    q.as_ptr(),
                    ptr::null(),
                    4,
                    target,
                    jacobian_derivative.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    usize::MAX,
                    jacobian_derivative.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_mass_matrix(robot, workspace, q.as_ptr(), 4, square.as_mut_ptr(), 15,),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_mass_matrix(robot, workspace, ptr::null(), 4, square.as_mut_ptr(), 16,),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    3,
                    square.as_mut_ptr(),
                    16,
                ),
                DyniboStatus::InvalidArgument
            );

            let zero_quaternion = DyniboPose {
                rotation_xyzw: [0.0; 4],
                ..DyniboPose::default()
            };
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &zero_quaternion,
                    dynibo_ik_options_default(),
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            let non_finite_translation = DyniboPose {
                translation: [f64::NAN, 0.0, 0.0],
                ..DyniboPose::default()
            };
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &non_finite_translation,
                    dynibo_ik_options_default(),
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            let non_finite_quaternion = DyniboPose {
                rotation_xyzw: [f64::INFINITY, 0.0, 0.0, 1.0],
                ..DyniboPose::default()
            };
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &non_finite_quaternion,
                    dynibo_ik_options_default(),
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    ptr::null(),
                    &DyniboPose::default(),
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_acceleration(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    ptr::null(),
                    4,
                    target,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    &DyniboPose::default(),
                    ptr::null(),
                    1,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            let invalid_load = DyniboLoad {
                link_id: usize::MAX,
                ..DyniboLoad::default()
            };
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    &DyniboPose::default(),
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    &invalid_load,
                    1,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );

            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(robot);
        }
    }

    #[test]
    fn matrix_and_derivative_abi_validate_every_pointer_and_length() {
        // SAFETY: valid buffers satisfy the documented ABI contracts; null pointers and
        // zero lengths are passed only to exercise validation paths that check them
        // before constructing slices.
        unsafe {
            let (robot, workspace, target) = create_handles();
            let q = [0.0; 4];
            let mut derivative = [0.0; 24];
            let mut square = [0.0; 16];

            // dynibo_jacobian_derivative: every validation arm.
            assert_eq!(
                dynibo_jacobian_derivative(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    derivative.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    derivative.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    ptr::null(),
                    q.as_ptr(),
                    4,
                    target,
                    derivative.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    ptr::null_mut(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    derivative.as_mut_ptr(),
                    23,
                ),
                DyniboStatus::InvalidArgument
            );
            // Zero state length takes the empty-slice arm, then the core rejects it.
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    ptr::null(),
                    ptr::null(),
                    0,
                    target,
                    derivative.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    ptr::null_mut(),
                    0,
                ),
                DyniboStatus::InvalidArgument
            );
            let mut overlapping_derivative = [0.0; 24];
            let overlapping_q = overlapping_derivative.as_ptr();
            let overlapping_output = overlapping_derivative.as_mut_ptr();
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    overlapping_q,
                    q.as_ptr(),
                    4,
                    target,
                    overlapping_output,
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert!(last_error().contains("q and output must not overlap"));
            let overlapping_qd = overlapping_derivative.as_ptr();
            let overlapping_output = overlapping_derivative.as_mut_ptr();
            assert_eq!(
                dynibo_jacobian_derivative(
                    robot,
                    workspace,
                    q.as_ptr(),
                    overlapping_qd,
                    4,
                    target,
                    overlapping_output,
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert!(last_error().contains("qd and output must not overlap"));

            // dynibo_mass_matrix: every validation arm.
            assert_eq!(
                dynibo_mass_matrix(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_mass_matrix(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_mass_matrix(robot, workspace, q.as_ptr(), 4, ptr::null_mut(), 16,),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_mass_matrix(robot, workspace, ptr::null(), 0, square.as_mut_ptr(), 16,),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_mass_matrix(robot, workspace, q.as_ptr(), 4, ptr::null_mut(), 0,),
                DyniboStatus::InvalidArgument
            );
            let mut overlapping_mass = [0.0; 16];
            let overlapping_q = overlapping_mass.as_ptr();
            let overlapping_output = overlapping_mass.as_mut_ptr();
            assert_eq!(
                dynibo_mass_matrix(robot, workspace, overlapping_q, 4, overlapping_output, 16,),
                DyniboStatus::InvalidArgument
            );
            assert!(last_error().contains("q and output must not overlap"));

            // dynibo_coriolis_matrix: every validation arm.
            assert_eq!(
                dynibo_coriolis_matrix(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    ptr::null(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    q.as_ptr(),
                    ptr::null(),
                    4,
                    square.as_mut_ptr(),
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    ptr::null_mut(),
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    15,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    ptr::null(),
                    ptr::null(),
                    0,
                    square.as_mut_ptr(),
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    ptr::null_mut(),
                    0,
                ),
                DyniboStatus::InvalidArgument
            );
            let mut overlapping_coriolis = [0.0; 16];
            let overlapping_q = overlapping_coriolis.as_ptr();
            let overlapping_output = overlapping_coriolis.as_mut_ptr();
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    overlapping_q,
                    q.as_ptr(),
                    4,
                    overlapping_output,
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert!(last_error().contains("q and output must not overlap"));
            let overlapping_qd = overlapping_coriolis.as_ptr();
            let overlapping_output = overlapping_coriolis.as_mut_ptr();
            assert_eq!(
                dynibo_coriolis_matrix(
                    robot,
                    workspace,
                    q.as_ptr(),
                    overlapping_qd,
                    4,
                    overlapping_output,
                    16,
                ),
                DyniboStatus::InvalidArgument
            );
            assert!(last_error().contains("qd and output must not overlap"));

            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(robot);
        }
    }

    #[test]
    fn abi_catches_panics_clears_errors_and_keeps_them_thread_local() {
        let status = call(|| panic!("intentional test panic"));
        assert_eq!(status, DyniboStatus::Panic);
        assert!(last_error().contains("panic caught"));

        // A successful call clears the previous error.
        assert_eq!(dynibo_ik_options_default().max_iterations, 100);
        assert_eq!(call(|| Ok(())), DyniboStatus::Ok);
        assert!(last_error().is_empty());

        set_error("main thread");
        let child = thread::spawn(|| {
            assert!(last_error().is_empty());
            set_error("child thread");
            last_error()
        });
        assert_eq!(child.join().unwrap(), "child thread");
        assert_eq!(last_error(), "main thread");
    }

    #[test]
    fn legacy_calculation_abi_validates_every_remaining_arm() {
        // SAFETY: valid buffers satisfy the documented ABI contracts; null pointers,
        // truncated lengths, and non-finite poses are passed only to exercise
        // validation paths that check them before constructing slices.
        unsafe {
            let (robot, workspace, target) = create_handles();
            let q = [0.0; 4];
            let short = [0.0; 3];
            let mut pose = DyniboPose::default();
            let mut twist = DyniboTwist::default();
            let mut jacobian = [0.0; 24];
            let mut output = [0.0; 4];
            let identity = DyniboPose::default();
            let non_finite = DyniboPose {
                translation: [f64::NAN, 0.0, 0.0],
                ..DyniboPose::default()
            };

            // A malformed URDF reaches the core-error arm of dynibo_robot_load_urdf.
            let invalid_path = CString::new(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../tests/data/invalid.urdf")
                    .to_string_lossy()
                    .as_bytes(),
            )
            .unwrap();
            let mut broken = ptr::null_mut();
            assert_eq!(
                dynibo_robot_load_urdf(invalid_path.as_ptr(), &mut broken),
                DyniboStatus::ModelError
            );
            assert!(broken.is_null());

            // dynibo_forward_kinematics: core slice-length rejection.
            assert_eq!(
                dynibo_forward_kinematics(robot, workspace, short.as_ptr(), 3, target, &mut pose,),
                DyniboStatus::InvalidArgument
            );

            // dynibo_jacobian: null handles, null q, and an invalid target.
            assert_eq!(
                dynibo_jacobian(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    jacobian.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    4,
                    target,
                    jacobian.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian(
                    robot,
                    workspace,
                    ptr::null(),
                    4,
                    target,
                    jacobian.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_jacobian(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    usize::MAX,
                    jacobian.as_mut_ptr(),
                    24,
                ),
                DyniboStatus::InvalidArgument
            );

            // dynibo_inverse_kinematics: null handles, inputs, desired, output, and target.
            let options = dynibo_ik_options_default();
            assert_eq!(
                dynibo_inverse_kinematics(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    options,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    options,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    ptr::null(),
                    4,
                    target,
                    &identity,
                    options,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    ptr::null(),
                    options,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    options,
                    ptr::null_mut(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    usize::MAX,
                    &identity,
                    options,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );

            // dynibo_forward_velocity: null handles, state, tool, output, non-finite
            // base, invalid target, and a core slice-length rejection.
            assert_eq!(
                dynibo_forward_velocity(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    ptr::null(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    ptr::null(),
                    4,
                    target,
                    &identity,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    ptr::null(),
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    &identity,
                    ptr::null_mut(),
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &non_finite,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    &non_finite,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    usize::MAX,
                    &identity,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    short.as_ptr(),
                    short.as_ptr(),
                    3,
                    target,
                    &identity,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );

            // dynibo_forward_acceleration: null handles, state slices, output,
            // invalid target, and a core slice-length rejection.
            assert_eq!(
                dynibo_forward_acceleration(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_acceleration(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_acceleration(
                    robot,
                    workspace,
                    ptr::null(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_acceleration(
                    robot,
                    workspace,
                    q.as_ptr(),
                    ptr::null(),
                    q.as_ptr(),
                    4,
                    target,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_acceleration(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    ptr::null_mut(),
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_acceleration(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    usize::MAX,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_acceleration(
                    robot,
                    workspace,
                    short.as_ptr(),
                    short.as_ptr(),
                    short.as_ptr(),
                    3,
                    target,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );

            // dynibo_gravity: null handles, q, base, output, and a non-finite base.
            assert_eq!(
                dynibo_gravity(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    4,
                    &identity,
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_gravity(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    4,
                    &identity,
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_gravity(
                    robot,
                    workspace,
                    ptr::null(),
                    4,
                    &identity,
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    ptr::null(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    &non_finite,
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    &identity,
                    ptr::null(),
                    0,
                    ptr::null_mut(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );

            // dynibo_inverse_dynamics: null handles, state slices, base, output, and
            // a non-finite base.
            assert_eq!(
                dynibo_inverse_dynamics(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    &identity,
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    &identity,
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    ptr::null(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    &identity,
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    ptr::null(),
                    q.as_ptr(),
                    4,
                    &identity,
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    ptr::null(),
                    4,
                    &identity,
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    ptr::null(),
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    &non_finite,
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    &identity,
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                    ptr::null(),
                    0,
                    ptr::null_mut(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );

            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(robot);
        }
    }

    #[test]
    fn core_errors_map_to_stable_abi_categories() {
        assert_eq!(
            core_error(dynibo::Error::InvalidModel("broken tree".to_owned())).0,
            DyniboStatus::ModelError
        );
        assert_eq!(
            core_error(dynibo::Error::UnknownLink {
                name: "missing".to_owned(),
            })
            .0,
            DyniboStatus::InvalidArgument
        );
        assert_eq!(
            core_error(dynibo::Error::IkNotConverged {
                iterations: 1,
                translation_error: 1.0,
                rotation_error: 0.0,
            })
            .0,
            DyniboStatus::SolverError
        );
    }
}
