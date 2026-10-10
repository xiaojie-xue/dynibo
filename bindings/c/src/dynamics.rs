//! C entry points for generalized forces, inertia, and accelerations.
use crate::boundary::{
    DyniboStatus, call, core_error, input_slice, output_slice, reject_output_overlap, required_ref,
};
use crate::handles::{
    DyniboFloatingRobot, DyniboFloatingWorkspace, DyniboRobot, DyniboWorkspace, fixed_parts,
    floating_parts, loads,
};
use crate::types::{DyniboBaseState, DyniboLoad, base_from_c};

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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (_, w) = unsafe { fixed_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (_, w) = unsafe { floating_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (_, w) = unsafe { fixed_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (_, w) = unsafe { floating_parts(r, w) }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
        reject_output_overlap!(out, on; q, n, "q");
        let q = unsafe { input_slice(q, n, "q") }?;
        // SAFETY: The C caller supplies a readable load array for this call.
        let ls = unsafe {
            loads(
                &r.link_ids,
                &mut w.converted_loads,
                &mut w.load_positions,
                lp,
                ln,
            )
        }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { floating_parts(r, w) }?;
        reject_output_overlap!(out, on; q, n, "q");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        // SAFETY: The C caller supplies a readable load array for this call.
        let ls = unsafe {
            loads(
                &r.link_ids,
                &mut w.converted_loads,
                &mut w.load_positions,
                lp,
                ln,
            )
        }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd"; qdd, n, "qdd");
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let qdd = unsafe { input_slice(qdd, n, "qdd") }?;
        // SAFETY: The C caller supplies a readable load array for this call.
        let ls = unsafe {
            loads(
                &r.link_ids,
                &mut w.converted_loads,
                &mut w.load_positions,
                lp,
                ln,
            )
        }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { floating_parts(r, w) }?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd"; qdd, n, "qdd");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let qdd = unsafe { input_slice(qdd, n, "qdd") }?;
        // SAFETY: The C caller supplies a readable load array for this call.
        let ls = unsafe {
            loads(
                &r.link_ids,
                &mut w.converted_loads,
                &mut w.load_positions,
                lp,
                ln,
            )
        }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { fixed_parts(r, w) }?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd"; f, fn_, "generalized_forces");
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let f = unsafe { input_slice(f, fn_, "generalized_forces") }?;
        // SAFETY: The C caller supplies a readable load array for this call.
        let ls = unsafe {
            loads(
                &r.link_ids,
                &mut w.converted_loads,
                &mut w.load_positions,
                lp,
                ln,
            )
        }?;
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
        // SAFETY: The C caller supplies live handles and exclusive workspace access.
        let (r, w) = unsafe { floating_parts(r, w) }?;
        reject_output_overlap!(out, on; q, n, "q"; qd, n, "qd"; f, fn_, "generalized_forces");
        let b = base_from_c(unsafe { required_ref(b, "base") }?)?;
        let q = unsafe { input_slice(q, n, "q") }?;
        let qd = unsafe { input_slice(qd, n, "qd") }?;
        let f = unsafe { input_slice(f, fn_, "generalized_forces") }?;
        // SAFETY: The C caller supplies a readable load array for this call.
        let ls = unsafe {
            loads(
                &r.link_ids,
                &mut w.converted_loads,
                &mut w.load_positions,
                lp,
                ln,
            )
        }?;
        let out = unsafe { output_slice(out, on, "output") }?;
        w.inner
            .forward_dynamics(&b, q, qd, f, ls, out)
            .map_err(core_error)
    })
}
