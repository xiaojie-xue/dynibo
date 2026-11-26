//! Shared lifecycle for fixed- and floating-base Python calculation instances.

use std::sync::{
    Mutex, MutexGuard,
    atomic::{AtomicBool, Ordering},
};

use dynibo::{Frame, IndexedLoad};
use pyo3::{exceptions::PyRuntimeError, prelude::*, sync::MutexExt};

use crate::errors::{catch_panic, lock_error};

pub(super) struct Calculation<R> {
    pub(super) robot: R,
    pub(super) poses: Vec<Frame>,
    pub(super) output: Vec<f64>,
    pub(super) loads: Vec<IndexedLoad>,
}

impl<R> Calculation<R> {
    pub(super) fn new(robot: R, link_count: usize, generalized_count: usize) -> Self {
        let output_len = (7 * link_count)
            .max(generalized_count * generalized_count)
            .max(6 * generalized_count);
        Self {
            robot,
            poses: vec![Frame::identity(); link_count],
            output: vec![0.0; output_len],
            loads: Vec::with_capacity(link_count),
        }
    }
}

pub(super) fn ensure_open(closed: &AtomicBool) -> PyResult<()> {
    if closed.load(Ordering::Acquire) {
        Err(PyRuntimeError::new_err("robot is closed"))
    } else {
        Ok(())
    }
}

// Keep storage and construction on the Python classes. These generic borrows
// share lifecycle rules without changing object layout or moving the workspace.
// One lock protects both the core instance and its reusable scratch buffers;
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
    closed: &AtomicBool,
    py: Python<'_>,
) -> PyResult<()> {
    let mut guard = inner.lock_py_attached(py).map_err(|_| lock_error())?;
    guard.take();
    closed.store(true, Ordering::Release);
    Ok(())
}
