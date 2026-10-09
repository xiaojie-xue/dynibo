//! Shared lifecycle for fixed- and floating-base Python calculation instances.

use std::sync::{Mutex, MutexGuard};

use dynibo::Frame;
use pyo3::{exceptions::PyRuntimeError, prelude::*, sync::MutexExt};

use super::{catch_panic, lock_error};

pub(super) struct Calculation<R> {
    pub(super) robot: R,
    pub(super) poses: Vec<Frame>,
}

// Keep storage and construction on the Python classes. These generic borrows
// share lifecycle rules without changing object layout or moving the workspace.
// One lock protects both the core instance and its reusable pose buffer;
// `None` is the closed state.
pub(super) fn lock_calculation<'a, R>(
    inner: &'a Mutex<Option<Calculation<R>>>,
    py: Python<'_>,
) -> PyResult<MutexGuard<'a, Option<Calculation<R>>>> {
    let guard = inner.lock_py_attached(py).map_err(|_| lock_error())?;
    if guard.is_none() {
        Err(PyRuntimeError::new_err("robot is closed"))
    } else {
        Ok(guard)
    }
}

pub(super) fn with_calculation<R, T>(
    inner: &Mutex<Option<Calculation<R>>>,
    py: Python<'_>,
    calculate: impl FnOnce(&mut Calculation<R>) -> PyResult<T>,
) -> PyResult<T> {
    let mut guard = lock_calculation(inner, py)?;
    // Keep the guard outside catch_unwind: a caught calculation panic must
    // not poison the lock. Each core operation rebuilds its scratch state.
    catch_panic(|| calculate(guard.as_mut().expect("open robot checked")))
}

pub(super) fn with_robot<R, T>(
    inner: &Mutex<Option<Calculation<R>>>,
    py: Python<'_>,
    calculate: impl FnOnce(&mut R) -> PyResult<T>,
) -> PyResult<T> {
    let mut guard = lock_calculation(inner, py)?;
    catch_panic(|| calculate(&mut guard.as_mut().expect("open robot checked").robot))
}

pub(super) fn close_calculation<R>(
    inner: &Mutex<Option<Calculation<R>>>,
    py: Python<'_>,
) -> PyResult<()> {
    inner.lock_py_attached(py).map_err(|_| lock_error())?.take();
    Ok(())
}
