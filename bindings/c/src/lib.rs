//! Stable C ABI for `dynibo`.

#![allow(
    clippy::missing_safety_doc,
    reason = "pointer ownership and validity contracts are documented in dynibo.h"
)]

use std::{
    cell::RefCell,
    ffi::{CStr, CString, c_char, c_int},
    panic::{AssertUnwindSafe, catch_unwind},
    ptr,
};

use dynibo::{
    BaseMode, BaseState, ErrorCategory, Frame, IndexedLoad, InverseKinematicsOptions, LinkId,
    Robot, Twist, Wrench,
};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};

thread_local! {
    static LAST_ERROR: RefCell<Vec<u8>> = RefCell::new(vec![0]);
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

/// Validated integer selecting the root-link connection mode.
pub type DyniboBaseMode = c_int;
/// The root link is fixed to the world.
pub const DYNIBO_BASE_FIXED: DyniboBaseMode = 0;
/// The root link has six generalized velocity coordinates.
pub const DYNIBO_BASE_FLOATING: DyniboBaseMode = 1;

fn base_mode_from_c(value: c_int) -> Result<BaseMode, (DyniboStatus, String)> {
    match value {
        DYNIBO_BASE_FIXED => Ok(BaseMode::Fixed),
        DYNIBO_BASE_FLOATING => Ok(BaseMode::Floating),
        _ => Err(invalid(format!("invalid base mode {value}"))),
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

/// Angular-first spatial vector `[angular_x, angular_y, angular_z, linear_x,
/// linear_y, linear_z]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DyniboTwist {
    /// Angular component.
    pub angular: [f64; 3],
    /// Linear component.
    pub linear: [f64; 3],
}

/// Resisting wrench at a link origin and expressed in that link frame.
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
    base: BaseState,
    link_ids: Vec<LinkId>,
    name: CString,
}

/// Opaque reusable calculation workspace.
pub struct DyniboWorkspace {
    inner: Robot,
    indexed_loads: Box<[IndexedLoad]>,
}

fn set_error(message: impl Into<String>) {
    let message = message.into();
    LAST_ERROR.with(|slot| {
        let mut output = slot.borrow_mut();
        output.clear();
        for byte in message.bytes() {
            if byte == 0 {
                output.extend_from_slice(b"\\0");
            } else {
                output.push(byte);
            }
        }
        output.push(0);
    });
}

fn call(function: impl FnOnce() -> Result<(), (DyniboStatus, String)>) -> DyniboStatus {
    LAST_ERROR.with(|slot| {
        let mut output = slot.borrow_mut();
        output.clear();
        output.push(0);
    });
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

fn validate_workspace_model(
    robot: &DyniboRobot,
    workspace: &DyniboWorkspace,
) -> Result<(), (DyniboStatus, String)> {
    if robot.inner.root_link_id() == workspace.inner.root_link_id() {
        Ok(())
    } else {
        Err(invalid("workspace does not belong to this robot model"))
    }
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

unsafe fn load_slice<'a>(
    robot: &DyniboRobot,
    output: &'a mut [IndexedLoad],
    pointer: *const DyniboLoad,
    length: usize,
) -> Result<&'a [IndexedLoad], (DyniboStatus, String)> {
    for load in output.iter_mut() {
        load.wrench = Wrench::zeros();
    }
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
    for load in loads {
        if load.link_id >= robot.link_ids.len() {
            return Err(invalid(format!("invalid link id {}", load.link_id)));
        }
        let current = output[load.link_id].wrench;
        output[load.link_id].wrench = Wrench::new(
            current.torque + Vector3::from(load.torque),
            current.force + Vector3::from(load.force),
        );
    }
    Ok(if loads.is_empty() {
        &output[..0]
    } else {
        output
    })
}

/// Returns the last error message for the calling thread.
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_last_error_message() -> *const c_char {
    LAST_ERROR.with(|slot| slot.borrow().as_ptr().cast())
}

/// Returns the linked ABI version string.
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_version() -> *const c_char {
    c"0.3.0".as_ptr()
}

/// Returns default inverse-kinematics options.
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_ik_options_default() -> DyniboIkOptions {
    DyniboIkOptions::default()
}

/// Loads a URDF and allocates a robot handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_from_urdf(
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
        let link_ids = (0..inner.link_count())
            .map(|index| inner.link_id_at(index).expect("link index came from robot"))
            .collect();
        let name =
            CString::new(inner.name()).map_err(|_| model_error("robot name contains NUL"))?;
        *output = Box::into_raw(Box::new(DyniboRobot {
            inner,
            base: BaseState::fixed(),
            link_ids,
            name,
        }));
        Ok(())
    })
}

/// Loads a URDF with an explicit root-link mode and allocates a robot handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_from_urdf_with_base(
    path: *const c_char,
    base_mode: c_int,
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
        let base_mode = base_mode_from_c(base_mode)?;
        let inner = Robot::from_urdf_with_base(path, base_mode).map_err(core_error)?;
        let base = match base_mode {
            BaseMode::Fixed => BaseState::fixed(),
            BaseMode::Floating => BaseState::new(Frame::identity(), Twist::zeros(), Twist::zeros())
                .expect("zero floating-base state is valid"),
        };
        let link_ids = (0..inner.link_count())
            .map(|index| inner.link_id_at(index).expect("link index came from robot"))
            .collect();
        let name =
            CString::new(inner.name()).map_err(|_| model_error("robot name contains NUL"))?;
        *output = Box::into_raw(Box::new(DyniboRobot {
            inner,
            base,
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

/// Returns the number of non-fixed joints, or zero for a null handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_joint_count(robot: *const DyniboRobot) -> usize {
    // SAFETY: Reading a valid opaque handle is part of the C contract.
    unsafe { robot.as_ref() }.map_or(0, |robot| robot.inner.joint_count())
}

/// Returns the generalized-vector size, or zero for a null handle.
///
/// Floating-base vectors begin with world-frame angular then linear components,
/// followed by non-fixed joints in URDF order.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_generalized_count(robot: *const DyniboRobot) -> usize {
    unsafe { robot.as_ref() }.map_or(0, |robot| robot.inner.generalized_count())
}

/// Replaces the complete floating-base state.
///
/// `velocity` and `acceleration` are angular-first and expressed in the world
/// frame at the root-link origin.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_set_floating_base_state(
    robot: *mut DyniboRobot,
    frame: *const DyniboPose,
    velocity: DyniboTwist,
    acceleration: DyniboTwist,
) -> DyniboStatus {
    call(|| {
        let robot = unsafe { required_mut(robot, "robot") }?;
        let frame = frame_from_pose(unsafe { required_ref(frame, "frame") }?)?;
        if robot.inner.base_mode() != BaseMode::Floating {
            return Err(core_error(dynibo::Error::InvalidBaseState {
                field: "mode",
                reason: "does not match robot base mode",
            }));
        }
        robot.base = BaseState::new(frame, twist_from_c(velocity), twist_from_c(acceleration))
            .map_err(core_error)?;
        Ok(())
    })
}

/// Replaces the root-link pose used by every calculation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_robot_set_base_frame(
    robot: *mut DyniboRobot,
    frame: *const DyniboPose,
) -> DyniboStatus {
    call(|| {
        let robot = unsafe { required_mut(robot, "robot") }?;
        let frame = frame_from_pose(unsafe { required_ref(frame, "frame") }?)?;
        robot.base = match robot.inner.base_mode() {
            BaseMode::Fixed => BaseState::fixed_at(frame).expect("C pose was already validated"),
            BaseMode::Floating => {
                BaseState::new(frame, robot.base.velocity(), robot.base.acceleration())
                    .expect("C pose and existing base motion were already validated")
            }
        };
        Ok(())
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
            inner: robot.inner.fork(),
            indexed_loads: robot
                .link_ids
                .iter()
                .copied()
                .map(|link| IndexedLoad {
                    link,
                    wrench: Wrench::zeros(),
                })
                .collect(),
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

/// Computes the target-link pose in the world frame.
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
        validate_workspace_model(robot, workspace)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, q_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { required_mut(output, "output") }?;
        let link = robot
            .link_ids
            .get(target)
            .copied()
            .ok_or_else(|| invalid(format!("invalid link id {target}")))?;
        let frame = workspace
            .inner
            .forward_kinematics(&robot.base, q, link)
            .map_err(core_error)?;
        *output = pose_from_frame(&frame);
        Ok(())
    })
}

/// Writes the world-frame, target-origin column-major `6 x generalized_count`
/// geometric Jacobian. Rows are angular then linear; floating-base columns are
/// world-frame angular then linear, followed by non-fixed URDF joint columns.
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
        validate_workspace_model(robot, workspace)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, q_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        let link = robot
            .link_ids
            .get(target)
            .copied()
            .ok_or_else(|| invalid(format!("invalid link id {target}")))?;
        workspace
            .inner
            .jacobian(&robot.base, q, link, output)
            .map_err(core_error)
    })
}

/// Writes the world-frame, target-origin column-major `6 x generalized_count`
/// Jacobian time derivative with the same row and column ordering as
/// [`dynibo_jacobian`].
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
        validate_workspace_model(robot, workspace)?;
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
        workspace
            .inner
            .jacobian_derivative(&robot.base, q, qd, link, output)
            .map_err(core_error)
    })
}

/// Writes the column-major `generalized_count x generalized_count` mass matrix.
/// Rows and columns use the generalized-vector ordering documented by
/// [`dynibo_robot_generalized_count`].
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
        validate_workspace_model(robot, workspace)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, q_len, "q") }?;
        reject_f64_overlap(q.as_ptr(), q.len(), "q", output, output_len)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        workspace
            .inner
            .mass_matrix(&robot.base, q, output)
            .map_err(core_error)
    })
}

/// Writes velocity-product generalized forces `C(q, qd) * qd`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_velocity_product_forces(
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
        validate_workspace_model(robot, workspace)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, state_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qd = unsafe { input_slice(qd, state_len, "qd") }?;
        reject_f64_overlap(q.as_ptr(), q.len(), "q", output, output_len)?;
        reject_f64_overlap(qd.as_ptr(), qd.len(), "qd", output, output_len)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        workspace
            .inner
            .velocity_product_forces(&robot.base, q, qd, output)
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
        validate_workspace_model(robot, workspace)?;
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
        let core_options = InverseKinematicsOptions {
            max_iterations: options.max_iterations,
            translation_tolerance: options.translation_tolerance,
            rotation_tolerance: options.rotation_tolerance,
            damping: options.damping,
            max_step_norm: options.max_step_norm,
        };
        workspace
            .inner
            .inverse_kinematics(&robot.base, initial_q, link, &desired, core_options, output)
            .map_err(core_error)
    })
}

/// Computes world-expressed target-link/tool spatial velocity.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_velocity_kinematics(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    state_len: usize,
    target: usize,
    tool: *const DyniboPose,
    output: *mut DyniboTwist,
) -> DyniboStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let workspace = unsafe { required_mut(workspace, "workspace") }?;
        validate_workspace_model(robot, workspace)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, state_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qd = unsafe { input_slice(qd, state_len, "qd") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let tool = frame_from_pose(unsafe { required_ref(tool, "tool") }?)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { required_mut(output, "output") }?;
        let link = robot
            .link_ids
            .get(target)
            .copied()
            .ok_or_else(|| invalid(format!("invalid link id {target}")))?;
        let value = workspace
            .inner
            .forward_velocity_kinematics(&robot.base, q, qd, link, &tool)
            .map_err(core_error)?;
        *output = twist_to_c(value);
        Ok(())
    })
}

/// Computes world-expressed target-link-origin spatial acceleration.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_acceleration_kinematics(
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
        validate_workspace_model(robot, workspace)?;
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
        let value = workspace
            .inner
            .forward_acceleration_kinematics(&robot.base, q, qd, qdd, link)
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
        validate_workspace_model(robot, workspace)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, q_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helper.
        let loads = unsafe { load_slice(robot, &mut workspace.indexed_loads, loads, load_count) }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        workspace
            .inner
            .gravity(&robot.base, q, loads, output)
            .map_err(core_error)
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
        validate_workspace_model(robot, workspace)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, state_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qd = unsafe { input_slice(qd, state_len, "qd") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qdd = unsafe { input_slice(qdd, state_len, "qdd") }?;
        // SAFETY: Pointer validation is performed by the helper.
        let loads = unsafe { load_slice(robot, &mut workspace.indexed_loads, loads, load_count) }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        workspace
            .inner
            .inverse_dynamics(&robot.base, q, qd, qdd, loads, output)
            .map_err(core_error)
    })
}

/// Writes articulated-body forward-dynamics generalized accelerations.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn dynibo_forward_dynamics(
    robot: *const DyniboRobot,
    workspace: *mut DyniboWorkspace,
    q: *const f64,
    qd: *const f64,
    state_len: usize,
    generalized_forces: *const f64,
    generalized_force_len: usize,
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
        validate_workspace_model(robot, workspace)?;
        // SAFETY: Pointer validation is performed by the helpers.
        let q = unsafe { input_slice(q, state_len, "q") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let qd = unsafe { input_slice(qd, state_len, "qd") }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let generalized_forces = unsafe {
            input_slice(
                generalized_forces,
                generalized_force_len,
                "generalized_forces",
            )
        }?;
        reject_f64_overlap(q.as_ptr(), q.len(), "q", output, output_len)?;
        reject_f64_overlap(qd.as_ptr(), qd.len(), "qd", output, output_len)?;
        reject_f64_overlap(
            generalized_forces.as_ptr(),
            generalized_forces.len(),
            "generalized_forces",
            output,
            output_len,
        )?;
        // SAFETY: Pointer validation is performed by the helper.
        let loads = unsafe { load_slice(robot, &mut workspace.indexed_loads, loads, load_count) }?;
        // SAFETY: Pointer validation is performed by the helpers.
        let output = unsafe { output_slice(output, output_len, "output") }?;
        workspace
            .inner
            .forward_dynamics(&robot.base, q, qd, generalized_forces, loads, output)
            .map_err(core_error)
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
            unsafe { dynibo_robot_from_urdf(fixture_path().as_ptr(), &mut robot) },
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
            assert_eq!(CStr::from_ptr(dynibo_version()), c"0.3.0");
            assert!(dynibo_robot_name(ptr::null()).is_null());
            assert_eq!(dynibo_robot_joint_count(ptr::null()), 0);
            assert_eq!(dynibo_robot_link_count(ptr::null()), 0);
            dynibo_robot_destroy(ptr::null_mut());
            dynibo_workspace_destroy(ptr::null_mut());

            let path = fixture_path();
            assert_eq!(
                dynibo_robot_from_urdf(path.as_ptr(), ptr::null_mut()),
                DyniboStatus::InvalidArgument
            );
            let mut robot = ptr::dangling_mut::<DyniboRobot>();
            assert_eq!(
                dynibo_robot_from_urdf(ptr::null(), &mut robot),
                DyniboStatus::InvalidArgument
            );
            assert!(robot.is_null());
            let invalid_utf8 = [0xff_u8, 0];
            assert_eq!(
                dynibo_robot_from_urdf(invalid_utf8.as_ptr().cast(), &mut robot),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_workspace_create(ptr::null(), ptr::null_mut()),
                DyniboStatus::InvalidArgument
            );
        }
    }

    #[test]
    fn floating_base_abi_covers_construction_state_and_calculations() {
        // SAFETY: all non-null pointers below reference live local storage or handles created
        // by this test; each allocated handle is destroyed exactly once.
        unsafe {
            assert_eq!(dynibo_robot_generalized_count(ptr::null()), 0);
            let path = fixture_path();

            let mut fixed = ptr::null_mut();
            assert_eq!(
                dynibo_robot_from_urdf_with_base(path.as_ptr(), DYNIBO_BASE_FIXED, &mut fixed,),
                DyniboStatus::Ok
            );
            assert_eq!(dynibo_robot_generalized_count(fixed), 4);
            let identity = DyniboPose::default();
            assert_eq!(
                dynibo_robot_set_base_frame(fixed, &identity),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_robot_set_floating_base_state(
                    fixed,
                    &identity,
                    DyniboTwist::default(),
                    DyniboTwist::default(),
                ),
                DyniboStatus::InvalidArgument
            );
            dynibo_robot_destroy(fixed);

            let mut floating = ptr::null_mut();
            assert_eq!(
                dynibo_robot_from_urdf_with_base(
                    path.as_ptr(),
                    DYNIBO_BASE_FLOATING,
                    &mut floating,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(dynibo_robot_generalized_count(floating), 10);
            let velocity = DyniboTwist {
                angular: [0.2, -0.1, 0.3],
                linear: [-0.4, 0.2, 0.1],
            };
            let acceleration = DyniboTwist {
                angular: [-0.1, 0.3, 0.2],
                linear: [0.5, -0.2, 0.4],
            };
            assert_eq!(
                dynibo_robot_set_floating_base_state(floating, &identity, velocity, acceleration),
                DyniboStatus::Ok
            );
            let invalid_velocity = DyniboTwist {
                angular: [f64::NAN, 0.0, 0.0],
                linear: [0.0; 3],
            };
            assert_eq!(
                dynibo_robot_set_floating_base_state(
                    floating,
                    &identity,
                    invalid_velocity,
                    acceleration,
                ),
                DyniboStatus::InvalidArgument
            );
            let shifted = DyniboPose {
                translation: [0.3, -0.2, 0.1],
                ..identity
            };
            assert_eq!(
                dynibo_robot_set_base_frame(floating, &shifted),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_robot_set_floating_base_state(
                    ptr::null_mut(),
                    &identity,
                    velocity,
                    acceleration,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_robot_set_floating_base_state(floating, ptr::null(), velocity, acceleration,),
                DyniboStatus::InvalidArgument
            );

            let mut workspace = ptr::null_mut();
            assert_eq!(
                dynibo_workspace_create(floating, &mut workspace),
                DyniboStatus::Ok
            );
            let mut target = usize::MAX;
            assert_eq!(
                dynibo_robot_link_id(floating, c"test_link_4".as_ptr(), &mut target,),
                DyniboStatus::Ok
            );
            let q = [0.1, 0.2, -0.3, 0.4];
            let mut pose = DyniboPose::default();
            let mut twist = DyniboTwist::default();
            let mut jacobian = [0.0; 60];
            let mut matrix = [0.0; 100];
            let mut generalized = [0.0; 10];
            assert_eq!(
                dynibo_forward_kinematics(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.len(),
                    target,
                    &mut pose,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_jacobian(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.len(),
                    target,
                    jacobian.as_mut_ptr(),
                    jacobian.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_jacobian_derivative(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.len(),
                    target,
                    jacobian.as_mut_ptr(),
                    jacobian.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.len(),
                    target,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_forward_acceleration_kinematics(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    q.len(),
                    target,
                    &mut twist,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_mass_matrix(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.len(),
                    matrix.as_mut_ptr(),
                    matrix.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_velocity_product_forces(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.len(),
                    generalized.as_mut_ptr(),
                    generalized.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_gravity(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.len(),
                    ptr::null(),
                    0,
                    generalized.as_mut_ptr(),
                    generalized.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    q.len(),
                    ptr::null(),
                    0,
                    generalized.as_mut_ptr(),
                    generalized.len(),
                ),
                DyniboStatus::Ok
            );
            let generalized_forces = generalized;
            assert_eq!(
                dynibo_forward_dynamics(
                    floating,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.len(),
                    generalized_forces.as_ptr(),
                    generalized_forces.len(),
                    ptr::null(),
                    0,
                    generalized.as_mut_ptr(),
                    generalized.len(),
                ),
                DyniboStatus::Ok
            );

            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(floating);

            let mut rejected = ptr::dangling_mut::<DyniboRobot>();
            assert_eq!(
                dynibo_robot_from_urdf_with_base(ptr::null(), DYNIBO_BASE_FLOATING, &mut rejected,),
                DyniboStatus::InvalidArgument
            );
            assert!(rejected.is_null());
            assert_eq!(
                dynibo_robot_from_urdf_with_base(
                    path.as_ptr(),
                    DYNIBO_BASE_FLOATING,
                    ptr::null_mut(),
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_robot_from_urdf_with_base(path.as_ptr(), 99, &mut rejected),
                DyniboStatus::InvalidArgument
            );
            assert!(rejected.is_null());
            let missing = CString::new("/definitely/missing/dynibo.urdf").unwrap();
            assert_eq!(
                dynibo_robot_from_urdf_with_base(
                    missing.as_ptr(),
                    DYNIBO_BASE_FIXED,
                    &mut rejected,
                ),
                DyniboStatus::ModelError
            );
            assert!(rejected.is_null());
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
            let (other_robot, other_workspace, _) = create_handles();
            let q = [0.0; 4];
            let mut pose = DyniboPose::default();
            assert_eq!(
                dynibo_forward_kinematics(
                    robot,
                    other_workspace,
                    q.as_ptr(),
                    q.len(),
                    target,
                    &mut pose,
                ),
                DyniboStatus::InvalidArgument
            );
            dynibo_workspace_destroy(other_workspace);
            dynibo_robot_destroy(other_robot);
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
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    4,
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
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &DyniboPose::default(),
                    &mut twist,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_forward_acceleration_kinematics(
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
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    3,
                    square.as_mut_ptr(),
                    4,
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
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    ptr::null(),
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_acceleration_kinematics(
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

            // dynibo_velocity_product_forces: every validation arm.
            assert_eq!(
                dynibo_velocity_product_forces(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    ptr::null(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    q.as_ptr(),
                    ptr::null(),
                    4,
                    square.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    ptr::null_mut(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    square.as_mut_ptr(),
                    3,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    ptr::null(),
                    ptr::null(),
                    0,
                    square.as_mut_ptr(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_velocity_product_forces(
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
            let mut overlapping_velocity_product = [0.0; 4];
            let overlapping_q = overlapping_velocity_product.as_ptr();
            let overlapping_output = overlapping_velocity_product.as_mut_ptr();
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    overlapping_q,
                    q.as_ptr(),
                    4,
                    overlapping_output,
                    4,
                ),
                DyniboStatus::InvalidArgument
            );
            assert!(last_error().contains("q and output must not overlap"));
            let overlapping_qd = overlapping_velocity_product.as_ptr();
            let overlapping_output = overlapping_velocity_product.as_mut_ptr();
            assert_eq!(
                dynibo_velocity_product_forces(
                    robot,
                    workspace,
                    q.as_ptr(),
                    overlapping_qd,
                    4,
                    overlapping_output,
                    4,
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
    fn error_buffer_and_overlap_checks_cover_nul_and_overflow_paths() {
        set_error("left\0right");
        assert_eq!(last_error(), "left\\0right");

        let input = ptr::dangling::<f64>();
        let output = ptr::dangling_mut::<f64>();
        let error = reject_f64_overlap(input, usize::MAX, "q", output, 1).unwrap_err();
        assert!(error.1.contains("q length is too large"));

        let error = reject_f64_overlap(input, 1, "q", output, usize::MAX).unwrap_err();
        assert!(error.1.contains("output length is too large"));

        let overflowing_input = ptr::without_provenance::<f64>(usize::MAX - 3);
        let error = reject_f64_overlap(overflowing_input, 1, "q", output, 1).unwrap_err();
        assert!(error.1.contains("q address range overflows"));

        let overflowing_output = ptr::without_provenance_mut::<f64>(usize::MAX - 3);
        let error = reject_f64_overlap(input, 1, "q", overflowing_output, 1).unwrap_err();
        assert!(error.1.contains("output address range overflows"));
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

            // A malformed URDF reaches the core-error arm of dynibo_robot_from_urdf.
            let invalid_path = CString::new(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../tests/data/invalid.urdf")
                    .to_string_lossy()
                    .as_bytes(),
            )
            .unwrap();
            let mut broken = ptr::null_mut();
            assert_eq!(
                dynibo_robot_from_urdf(invalid_path.as_ptr(), &mut broken),
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

            // dynibo_forward_velocity_kinematics: null handles, state, tool, output,
            // invalid target, and a core slice-length rejection.
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    ptr::null_mut(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    ptr::null(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    ptr::null(),
                    4,
                    target,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    ptr::null(),
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &identity,
                    ptr::null_mut(),
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &non_finite,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    usize::MAX,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    short.as_ptr(),
                    short.as_ptr(),
                    3,
                    target,
                    &identity,
                    &mut twist,
                ),
                DyniboStatus::InvalidArgument
            );

            // dynibo_forward_acceleration_kinematics: null handles, state slices, output,
            // invalid target, and a core slice-length rejection.
            assert_eq!(
                dynibo_forward_acceleration_kinematics(
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
                dynibo_forward_acceleration_kinematics(
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
                dynibo_forward_acceleration_kinematics(
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
                dynibo_forward_acceleration_kinematics(
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
                dynibo_forward_acceleration_kinematics(
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
                dynibo_forward_acceleration_kinematics(
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
                dynibo_forward_acceleration_kinematics(
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

            // Base-frame validation is centralized in the state setter.
            assert_eq!(
                dynibo_robot_set_base_frame(ptr::null_mut(), &identity),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_robot_set_base_frame(robot, ptr::null()),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                dynibo_robot_set_base_frame(robot, &non_finite),
                DyniboStatus::InvalidArgument
            );

            // dynibo_gravity: null handles, q, loads, and output.
            assert_eq!(
                dynibo_gravity(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    4,
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
                    0,
                    ptr::null_mut(),
                    4,
                ),
                DyniboStatus::InvalidArgument
            );

            // dynibo_inverse_dynamics: null handles, state slices, and output.
            assert_eq!(
                dynibo_inverse_dynamics(
                    ptr::null(),
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
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
