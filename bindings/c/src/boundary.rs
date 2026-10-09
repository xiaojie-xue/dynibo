//! Error reporting and raw-buffer safety at the C boundary.
use dynibo::ErrorCategory;
use std::{
    cell::RefCell,
    ffi::c_char,
    panic::{AssertUnwindSafe, catch_unwind},
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
pub(super) type CResult<T> = Result<T, (DyniboStatus, String)>;
pub(super) fn invalid(s: impl Into<String>) -> (DyniboStatus, String) {
    (DyniboStatus::InvalidArgument, s.into())
}
pub(super) fn core_error(e: dynibo::Error) -> (DyniboStatus, String) {
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
pub(super) fn call(f: impl FnOnce() -> CResult<()>) -> DyniboStatus {
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
/// # Safety
/// A non-null pointer must be aligned and point to an initialized `T` that is
/// readable and not mutated for the entire returned lifetime `'a`.
pub(super) unsafe fn required_ref<'a, T>(p: *const T, n: &str) -> CResult<&'a T> {
    unsafe { p.as_ref() }.ok_or_else(|| invalid(format!("{n} must not be null")))
}
/// # Safety
/// A non-null pointer must be aligned and point to an initialized `T` that is
/// exclusively accessible for the entire returned lifetime `'a`.
pub(super) unsafe fn required_mut<'a, T>(p: *mut T, n: &str) -> CResult<&'a mut T> {
    unsafe { p.as_mut() }.ok_or_else(|| invalid(format!("{n} must not be null")))
}
/// # Safety
/// For nonzero `n`, a non-null pointer must describe `n` initialized, aligned
/// values in one allocation, readable without mutation for `'a`. The byte size
/// must fit in `isize`. Overlap with mutable outputs must be rejected first.
pub(super) unsafe fn input_slice<'a>(p: *const f64, n: usize, name: &str) -> CResult<&'a [f64]> {
    if n == 0 {
        Ok(&[])
    } else if p.is_null() {
        Err(invalid(format!("{name} must not be null")))
    } else {
        Ok(unsafe { std::slice::from_raw_parts(p, n) })
    }
}
/// # Safety
/// For nonzero `n`, a non-null pointer must describe `n` aligned, writable values
/// in one allocation, exclusively accessible for `'a`. The byte size must fit
/// in `isize`. Reject overlap before creating this mutable slice.
pub(super) unsafe fn output_slice<'a>(p: *mut f64, n: usize, name: &str) -> CResult<&'a mut [f64]> {
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
pub(super) fn reject_byte_overlap(
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

pub(super) fn reject_f64_overlap(
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

pub(super) fn reject_struct_overlap<T>(
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
        $($crate::boundary::reject_f64_overlap($input, $input_len, $input_name, $output, $output_len)?;)+
    };
}

macro_rules! reject_struct_output_overlap {
    ($output:expr; $($input:expr, $input_len:expr, $input_name:expr);+ $(;)?) => {
        $($crate::boundary::reject_struct_overlap($input, $input_len, $input_name, $output)?;)+
    };
}

pub(super) use {reject_output_overlap, reject_struct_output_overlap};
#[unsafe(no_mangle)]
pub extern "C" fn dynibo_last_error_message() -> *const c_char {
    LAST_ERROR.with(|x| x.borrow().as_ptr().cast())
}
