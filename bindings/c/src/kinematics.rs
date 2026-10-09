//! C entry points for poses, spatial motion, Jacobians, and inverse kinematics.
use crate::boundary::{
    CResult, DyniboStatus, call, core_error, input_slice, invalid, output_slice,
    reject_byte_overlap, reject_output_overlap, reject_struct_output_overlap, required_mut,
    required_ref,
};
use crate::handles::{
    DyniboFloatingRobot, DyniboFloatingWorkspace, DyniboRobot, DyniboWorkspace, fixed_parts,
    floating_parts, link,
};
use crate::types::{
    DyniboBaseState, DyniboIkOptions, DyniboPose, DyniboTwist, base_from_c, frame_from_pose,
    pose_from_frame, twist_to_c,
};
use dynibo::InverseKinematicsOptions;

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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
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

fn validate_pose_output(
    q: *const f64,
    n: usize,
    out: *mut DyniboPose,
    len: usize,
    expected: usize,
) -> CResult<()> {
    if len != expected {
        return Err(invalid(format!(
            "expected {expected} output poses, found {len}"
        )));
    }
    if out.is_null() {
        return Err(invalid("output must not be null"));
    }
    let input_bytes = n
        .checked_mul(size_of::<f64>())
        .filter(|n| *n <= isize::MAX as usize)
        .ok_or_else(|| invalid("q length is too large"))?;
    let output_bytes = len
        .checked_mul(size_of::<DyniboPose>())
        .filter(|n| *n <= isize::MAX as usize)
        .ok_or_else(|| invalid("output length is too large"))?;
    reject_byte_overlap(q.cast(), input_bytes, "q", out.cast(), output_bytes)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_forward_kinematics_all(
    r: *const DyniboRobot,
    w: *mut DyniboWorkspace,
    q: *const f64,
    n: usize,
    out: *mut DyniboPose,
    len: usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
        validate_pose_output(q, n, out, len, r.inner.link_count())?;
        let q = unsafe { input_slice(q, n, "q") }?;
        w.inner
            .forward_kinematics_all(q, &mut w.poses)
            .map_err(core_error)?;
        for (i, frame) in w.poses.iter().enumerate() {
            unsafe {
                out.add(i).write(pose_from_frame(frame));
            }
        }
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn dynibo_floating_forward_kinematics_all(
    r: *const DyniboFloatingRobot,
    w: *mut DyniboFloatingWorkspace,
    b: *const DyniboBaseState,
    q: *const f64,
    n: usize,
    out: *mut DyniboPose,
    len: usize,
) -> DyniboStatus {
    call(|| {
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { floating_parts(r, w) }?;
        validate_pose_output(q, n, out, len, r.inner.link_count())?;
        let base = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        w.inner
            .forward_kinematics_all(&base, q, &mut w.poses)
            .map_err(core_error)?;
        for (i, frame) in w.poses.iter().enumerate() {
            unsafe {
                out.add(i).write(pose_from_frame(frame));
            }
        }
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { floating_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { floating_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { floating_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { floating_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { floating_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
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
