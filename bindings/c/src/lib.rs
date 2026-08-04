//! Stable C ABI for `dyno`.

#![allow(
    clippy::missing_safety_doc,
    reason = "pointer ownership and validity contracts are documented in dyno.h"
)]

use std::{
    cell::RefCell,
    ffi::{CStr, CString, c_char},
    panic::{AssertUnwindSafe, catch_unwind},
    ptr,
};

use dyno::{
    ErrorCategory, Frame, IndexedLoad, InverseKinematicsOptions, LinkId, Robot, Twist, Workspace,
    Wrench,
};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};

thread_local! {
    static LAST_ERROR: RefCell<CString> = RefCell::new(CString::default());
}

/// Status returned by every fallible C function.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynoStatus {
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

/// Translation plus an `(x, y, z, w)` unit quaternion.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DynoPose {
    /// Translation in metres.
    pub translation: [f64; 3],
    /// Quaternion ordered `(x, y, z, w)`.
    pub rotation_xyzw: [f64; 4],
}

impl Default for DynoPose {
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
pub struct DynoTwist {
    /// Angular component.
    pub angular: [f64; 3],
    /// Linear component.
    pub linear: [f64; 3],
}

/// External wrench applied at a link origin and expressed in that link frame.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DynoLoad {
    /// Model-scoped link handle returned by `dyno_robot_link_id`.
    pub link_id: usize,
    /// Torque component.
    pub torque: [f64; 3],
    /// Force component.
    pub force: [f64; 3],
}

/// Damped-least-squares inverse-kinematics configuration.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DynoIkOptions {
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

impl Default for DynoIkOptions {
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
pub struct DynoRobot {
    inner: Robot,
    link_ids: Vec<LinkId>,
    name: CString,
}

/// Opaque reusable calculation workspace.
pub struct DynoWorkspace {
    inner: Workspace,
}

fn set_error(message: impl Into<String>) {
    let message = message.into().replace('\0', "\\0");
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = CString::new(message).expect("NUL bytes were replaced");
    });
}

fn call(function: impl FnOnce() -> Result<(), (DynoStatus, String)>) -> DynoStatus {
    LAST_ERROR.with(|slot| *slot.borrow_mut() = CString::default());
    match catch_unwind(AssertUnwindSafe(function)) {
        Ok(Ok(())) => DynoStatus::Ok,
        Ok(Err((status, message))) => {
            set_error(message);
            status
        }
        Err(_) => {
            set_error("panic caught at dyno C ABI boundary");
            DynoStatus::Panic
        }
    }
}

fn invalid(message: impl Into<String>) -> (DynoStatus, String) {
    (DynoStatus::InvalidArgument, message.into())
}

fn core_error(error: dyno::Error) -> (DynoStatus, String) {
    let status = match error.category() {
        ErrorCategory::InvalidInput => DynoStatus::InvalidArgument,
        ErrorCategory::Model => DynoStatus::ModelError,
        ErrorCategory::Solver => DynoStatus::SolverError,
    };
    (status, error.to_string())
}

fn model_error(message: impl Into<String>) -> (DynoStatus, String) {
    (DynoStatus::ModelError, message.into())
}

unsafe fn required_ref<'a, T>(
    pointer: *const T,
    name: &str,
) -> Result<&'a T, (DynoStatus, String)> {
    // SAFETY: The caller of the C ABI promises that non-null pointers are valid.
    unsafe { pointer.as_ref() }.ok_or_else(|| invalid(format!("{name} must not be null")))
}

unsafe fn required_mut<'a, T>(
    pointer: *mut T,
    name: &str,
) -> Result<&'a mut T, (DynoStatus, String)> {
    // SAFETY: The caller of the C ABI promises that non-null pointers are valid and unique.
    unsafe { pointer.as_mut() }.ok_or_else(|| invalid(format!("{name} must not be null")))
}

unsafe fn input_slice<'a>(
    pointer: *const f64,
    length: usize,
    name: &str,
) -> Result<&'a [f64], (DynoStatus, String)> {
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
) -> Result<&'a mut [f64], (DynoStatus, String)> {
    if length == 0 {
        return Ok(&mut []);
    }
    if pointer.is_null() {
        return Err(invalid(format!("{name} must not be null")));
    }
    // SAFETY: Validity and uniqueness for `length` elements is part of the C contract.
    Ok(unsafe { std::slice::from_raw_parts_mut(pointer, length) })
}

fn frame_from_pose(pose: &DynoPose) -> Result<Frame, (DynoStatus, String)> {
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

fn pose_from_frame(frame: &Frame) -> DynoPose {
    let quaternion = frame.rotation.quaternion();
    DynoPose {
        translation: frame.translation.vector.into(),
        rotation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    }
}

fn twist_from_c(value: DynoTwist) -> Twist {
    Twist::new(Vector3::from(value.angular), Vector3::from(value.linear))
}

fn twist_to_c(value: Twist) -> DynoTwist {
    DynoTwist {
        angular: value.angular.into(),
        linear: value.linear.into(),
    }
}

unsafe fn load_slice(
    robot: &DynoRobot,
    pointer: *const DynoLoad,
    length: usize,
) -> Result<Vec<IndexedLoad>, (DynoStatus, String)> {
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
pub extern "C" fn dyno_last_error_message() -> *const c_char {
    LAST_ERROR.with(|slot| slot.borrow().as_ptr())
}

/// Returns the linked ABI version string.
#[unsafe(no_mangle)]
pub extern "C" fn dyno_version() -> *const c_char {
    c"0.1.0".as_ptr()
}

/// Returns default inverse-kinematics options.
#[unsafe(no_mangle)]
pub extern "C" fn dyno_ik_options_default() -> DynoIkOptions {
    DynoIkOptions::default()
}

/// Loads a URDF and allocates a robot handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_robot_load_urdf(
    path: *const c_char,
    output: *mut *mut DynoRobot,
) -> DynoStatus {
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
        *output = Box::into_raw(Box::new(DynoRobot {
            inner,
            link_ids,
            name,
        }));
        Ok(())
    })
}

/// Destroys a robot handle. Passing null is allowed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_robot_destroy(robot: *mut DynoRobot) {
    if !robot.is_null() {
        // SAFETY: The pointer was returned by `Box::into_raw` and is owned by the caller.
        drop(unsafe { Box::from_raw(robot) });
    }
}

/// Returns the URDF robot name, valid until the robot is destroyed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_robot_name(robot: *const DynoRobot) -> *const c_char {
    // SAFETY: Reading a valid opaque handle is part of the C contract.
    unsafe { robot.as_ref() }.map_or(ptr::null(), |robot| robot.name.as_ptr())
}

/// Returns the number of joints, or zero for a null handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_robot_joint_count(robot: *const DynoRobot) -> usize {
    // SAFETY: Reading a valid opaque handle is part of the C contract.
    unsafe { robot.as_ref() }.map_or(0, |robot| robot.inner.joint_count())
}

/// Returns the number of links, or zero for a null handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_robot_link_count(robot: *const DynoRobot) -> usize {
    // SAFETY: Reading a valid opaque handle is part of the C contract.
    unsafe { robot.as_ref() }.map_or(0, |robot| robot.inner.link_count())
}

/// Resolves a link name to a model-scoped handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_robot_link_id(
    robot: *const DynoRobot,
    name: *const c_char,
    output: *mut usize,
) -> DynoStatus {
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
pub unsafe extern "C" fn dyno_workspace_create(
    robot: *const DynoRobot,
    output: *mut *mut DynoWorkspace,
) -> DynoStatus {
    call(|| {
        // SAFETY: Pointer validation is performed by the helpers.
        let robot = unsafe { required_ref(robot, "robot") }?;
        // SAFETY: Pointer validation is performed by the helper.
        let output = unsafe { required_mut(output, "output") }?;
        *output = Box::into_raw(Box::new(DynoWorkspace {
            inner: robot.inner.workspace(),
        }));
        Ok(())
    })
}

/// Destroys a workspace handle. Passing null is allowed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_workspace_destroy(workspace: *mut DynoWorkspace) {
    if !workspace.is_null() {
        // SAFETY: The pointer was returned by `Box::into_raw` and is owned by the caller.
        drop(unsafe { Box::from_raw(workspace) });
    }
}

/// Computes forward kinematics for one link.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_forward_kinematics(
    robot: *const DynoRobot,
    workspace: *mut DynoWorkspace,
    q: *const f64,
    q_len: usize,
    target: usize,
    output: *mut DynoPose,
) -> DynoStatus {
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
pub unsafe extern "C" fn dyno_jacobian(
    robot: *const DynoRobot,
    workspace: *mut DynoWorkspace,
    q: *const f64,
    q_len: usize,
    target: usize,
    output: *mut f64,
    output_len: usize,
) -> DynoStatus {
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

/// Solves inverse kinematics for one link.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_inverse_kinematics(
    robot: *const DynoRobot,
    workspace: *mut DynoWorkspace,
    initial_q: *const f64,
    q_len: usize,
    target: usize,
    desired: *const DynoPose,
    options: DynoIkOptions,
    output: *mut f64,
    output_len: usize,
) -> DynoStatus {
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
pub unsafe extern "C" fn dyno_forward_velocity(
    robot: *const DynoRobot,
    workspace: *mut DynoWorkspace,
    q: *const f64,
    qd: *const f64,
    state_len: usize,
    target: usize,
    base: *const DynoPose,
    tool: *const DynoPose,
    output: *mut DynoTwist,
) -> DynoStatus {
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
        let value = robot
            .inner
            .forward_velocity_kinematics(q, qd, link, &base, &tool, &mut workspace.inner)
            .map_err(core_error)?;
        *output = twist_to_c(value);
        Ok(())
    })
}

/// Computes target-link-origin spatial acceleration.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dyno_forward_acceleration(
    robot: *const DynoRobot,
    workspace: *mut DynoWorkspace,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    state_len: usize,
    target: usize,
    output: *mut DynoTwist,
) -> DynoStatus {
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
pub unsafe extern "C" fn dyno_gravity(
    robot: *const DynoRobot,
    workspace: *mut DynoWorkspace,
    q: *const f64,
    q_len: usize,
    base: *const DynoPose,
    loads: *const DynoLoad,
    load_count: usize,
    output: *mut f64,
    output_len: usize,
) -> DynoStatus {
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
        robot
            .inner
            .gravity(q, &base, &loads, &mut workspace.inner, output)
            .map_err(core_error)
    })
}

/// Writes Newton-Euler inverse-dynamics joint forces.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn dyno_inverse_dynamics(
    robot: *const DynoRobot,
    workspace: *mut DynoWorkspace,
    q: *const f64,
    qd: *const f64,
    qdd: *const f64,
    state_len: usize,
    base: *const DynoPose,
    base_velocity: DynoTwist,
    base_acceleration: DynoTwist,
    loads: *const DynoLoad,
    load_count: usize,
    output: *mut f64,
    output_len: usize,
) -> DynoStatus {
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
        robot
            .inner
            .inverse_dynamics(
                q,
                qd,
                qdd,
                &base,
                twist_from_c(base_velocity),
                twist_from_c(base_acceleration),
                &loads,
                &mut workspace.inner,
                output,
            )
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
            CStr::from_ptr(dyno_last_error_message())
                .to_string_lossy()
                .into_owned()
        }
    }

    unsafe fn create_handles() -> (*mut DynoRobot, *mut DynoWorkspace, usize) {
        let mut robot = ptr::null_mut();
        // SAFETY: all pointers refer to live local storage or a valid C string.
        assert_eq!(
            unsafe { dyno_robot_load_urdf(fixture_path().as_ptr(), &mut robot) },
            DynoStatus::Ok
        );
        let mut workspace = ptr::null_mut();
        // SAFETY: `robot` was created above and output is writable.
        assert_eq!(
            unsafe { dyno_workspace_create(robot, &mut workspace) },
            DynoStatus::Ok
        );
        let mut target = usize::MAX;
        // SAFETY: handles, string, and output are valid.
        assert_eq!(
            unsafe { dyno_robot_link_id(robot, c"test_link_4".as_ptr(), &mut target) },
            DynoStatus::Ok
        );
        (robot, workspace, target)
    }

    #[test]
    fn metadata_and_construction_reject_invalid_arguments() {
        // SAFETY: null is explicitly supported by the metadata/destructor functions.
        unsafe {
            assert_eq!(CStr::from_ptr(dyno_version()), c"0.1.0");
            assert!(dyno_robot_name(ptr::null()).is_null());
            assert_eq!(dyno_robot_joint_count(ptr::null()), 0);
            assert_eq!(dyno_robot_link_count(ptr::null()), 0);
            dyno_robot_destroy(ptr::null_mut());
            dyno_workspace_destroy(ptr::null_mut());

            let path = fixture_path();
            assert_eq!(
                dyno_robot_load_urdf(path.as_ptr(), ptr::null_mut()),
                DynoStatus::InvalidArgument
            );
            let mut robot = ptr::dangling_mut::<DynoRobot>();
            assert_eq!(
                dyno_robot_load_urdf(ptr::null(), &mut robot),
                DynoStatus::InvalidArgument
            );
            assert!(robot.is_null());
            let invalid_utf8 = [0xff_u8, 0];
            assert_eq!(
                dyno_robot_load_urdf(invalid_utf8.as_ptr().cast(), &mut robot),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_workspace_create(ptr::null(), ptr::null_mut()),
                DynoStatus::InvalidArgument
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
                dyno_robot_link_id(ptr::null(), c"test_link_4".as_ptr(), &mut target),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_robot_link_id(robot, ptr::null(), &mut target),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_robot_link_id(robot, c"test_link_4".as_ptr(), ptr::null_mut()),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_robot_link_id(robot, c"missing".as_ptr(), &mut target),
                DynoStatus::InvalidArgument
            );
            let invalid_utf8 = [0xff_u8, 0];
            assert_eq!(
                dyno_robot_link_id(robot, invalid_utf8.as_ptr().cast(), &mut target),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_workspace_create(robot, ptr::null_mut()),
                DynoStatus::InvalidArgument
            );
            dyno_workspace_destroy(workspace);
            dyno_robot_destroy(robot);
        }
    }

    #[test]
    fn calculation_abi_covers_success_and_defensive_paths() {
        // SAFETY: valid buffers satisfy the documented ABI contracts; null pointers are used
        // only for validation cases where the callee checks them before constructing slices.
        unsafe {
            let (robot, workspace, target) = create_handles();
            let q = [0.0; 4];
            let mut pose = DynoPose::default();
            let mut twist = DynoTwist::default();
            let mut jacobian = [0.0; 24];
            let mut output = [0.0; 4];

            assert_eq!(
                dyno_forward_kinematics(robot, workspace, q.as_ptr(), 4, target, &mut pose),
                DynoStatus::Ok
            );
            assert_eq!(
                dyno_jacobian(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    jacobian.as_mut_ptr(),
                    24,
                ),
                DynoStatus::Ok
            );
            assert_eq!(
                dyno_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &pose,
                    dyno_ik_options_default(),
                    output.as_mut_ptr(),
                    4,
                ),
                DynoStatus::Ok
            );
            assert_eq!(
                dyno_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &DynoPose::default(),
                    &DynoPose::default(),
                    &mut twist,
                ),
                DynoStatus::Ok
            );
            assert_eq!(
                dyno_forward_acceleration(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    &mut twist,
                ),
                DynoStatus::Ok
            );
            assert_eq!(
                dyno_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    &DynoPose::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DynoStatus::Ok
            );
            let gravity = output;
            let load = DynoLoad {
                link_id: target,
                force: [0.0, 1.0, 0.0],
                torque: [0.0, 0.0, 0.5],
            };
            assert_eq!(
                dyno_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    &DynoPose::default(),
                    &load,
                    1,
                    output.as_mut_ptr(),
                    4,
                ),
                DynoStatus::Ok
            );
            assert_ne!(output, gravity);
            assert_eq!(
                dyno_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    &DynoPose::default(),
                    DynoTwist::default(),
                    DynoTwist::default(),
                    ptr::null(),
                    0,
                    output.as_mut_ptr(),
                    4,
                ),
                DynoStatus::Ok
            );

            assert_eq!(
                dyno_forward_kinematics(ptr::null(), workspace, q.as_ptr(), 4, target, &mut pose,),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_forward_kinematics(robot, ptr::null_mut(), q.as_ptr(), 4, target, &mut pose),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_forward_kinematics(robot, workspace, ptr::null(), 4, target, &mut pose),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_forward_kinematics(robot, workspace, q.as_ptr(), 4, usize::MAX, &mut pose,),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_forward_kinematics(robot, workspace, q.as_ptr(), 4, target, ptr::null_mut(),),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_jacobian(robot, workspace, q.as_ptr(), 4, target, ptr::null_mut(), 24,),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_jacobian(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    jacobian.as_mut_ptr(),
                    23,
                ),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_jacobian(
                    robot,
                    workspace,
                    ptr::null(),
                    0,
                    target,
                    jacobian.as_mut_ptr(),
                    24,
                ),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_jacobian(robot, workspace, q.as_ptr(), 4, target, ptr::null_mut(), 0,),
                DynoStatus::InvalidArgument
            );

            let zero_quaternion = DynoPose {
                rotation_xyzw: [0.0; 4],
                ..DynoPose::default()
            };
            assert_eq!(
                dyno_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &zero_quaternion,
                    dyno_ik_options_default(),
                    output.as_mut_ptr(),
                    4,
                ),
                DynoStatus::InvalidArgument
            );
            let non_finite_translation = DynoPose {
                translation: [f64::NAN, 0.0, 0.0],
                ..DynoPose::default()
            };
            assert_eq!(
                dyno_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &non_finite_translation,
                    dyno_ik_options_default(),
                    output.as_mut_ptr(),
                    4,
                ),
                DynoStatus::InvalidArgument
            );
            let non_finite_quaternion = DynoPose {
                rotation_xyzw: [f64::INFINITY, 0.0, 0.0, 1.0],
                ..DynoPose::default()
            };
            assert_eq!(
                dyno_inverse_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    target,
                    &non_finite_quaternion,
                    dyno_ik_options_default(),
                    output.as_mut_ptr(),
                    4,
                ),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    target,
                    ptr::null(),
                    &DynoPose::default(),
                    &mut twist,
                ),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_forward_acceleration(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    ptr::null(),
                    4,
                    target,
                    &mut twist,
                ),
                DynoStatus::InvalidArgument
            );
            assert_eq!(
                dyno_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    4,
                    &DynoPose::default(),
                    ptr::null(),
                    1,
                    output.as_mut_ptr(),
                    4,
                ),
                DynoStatus::InvalidArgument
            );
            let invalid_load = DynoLoad {
                link_id: usize::MAX,
                ..DynoLoad::default()
            };
            assert_eq!(
                dyno_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    q.as_ptr(),
                    q.as_ptr(),
                    4,
                    &DynoPose::default(),
                    DynoTwist::default(),
                    DynoTwist::default(),
                    &invalid_load,
                    1,
                    output.as_mut_ptr(),
                    4,
                ),
                DynoStatus::InvalidArgument
            );

            dyno_workspace_destroy(workspace);
            dyno_robot_destroy(robot);
        }
    }

    #[test]
    fn abi_catches_panics_clears_errors_and_keeps_them_thread_local() {
        let status = call(|| panic!("intentional test panic"));
        assert_eq!(status, DynoStatus::Panic);
        assert!(last_error().contains("panic caught"));

        // A successful call clears the previous error.
        assert_eq!(dyno_ik_options_default().max_iterations, 100);
        assert_eq!(call(|| Ok(())), DynoStatus::Ok);
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
    fn core_errors_map_to_stable_abi_categories() {
        assert_eq!(
            core_error(dyno::Error::InvalidModel("broken tree".to_owned())).0,
            DynoStatus::ModelError
        );
        assert_eq!(
            core_error(dyno::Error::UnknownLink {
                name: "missing".to_owned(),
            })
            .0,
            DynoStatus::InvalidArgument
        );
        assert_eq!(
            core_error(dyno::Error::IkNotConverged {
                iterations: 1,
                translation_error: 1.0,
                rotation_error: 0.0,
            })
            .0,
            DynoStatus::SolverError
        );
    }
}
