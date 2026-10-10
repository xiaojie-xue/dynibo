//! Python exception types and conversion of core errors and panics.
use dynibo::ErrorCategory;
use pyo3::{
    create_exception,
    exceptions::{PyRuntimeError, PyValueError},
    prelude::*,
};
use std::panic::{AssertUnwindSafe, catch_unwind};

create_exception!(_dynibo, DyniboError, PyRuntimeError);
create_exception!(_dynibo, ModelError, DyniboError);
create_exception!(_dynibo, SolverError, DyniboError);
create_exception!(_dynibo, PanicError, DyniboError);

pub(super) fn core_error(error: dynibo::Error) -> PyErr {
    let message = error.to_string();
    match error.category() {
        ErrorCategory::InvalidInput => PyValueError::new_err(message),
        ErrorCategory::Model => ModelError::new_err(message),
        ErrorCategory::Solver => SolverError::new_err(message),
    }
}

pub(super) fn lock_error() -> PyErr {
    PanicError::new_err("robot workspace lock is poisoned")
}

pub(super) fn catch_panic<T>(calculate: impl FnOnce() -> PyResult<T>) -> PyResult<T> {
    catch_unwind(AssertUnwindSafe(calculate)).map_err(|payload| {
        let message = payload
            .downcast_ref::<String>()
            .map(String::as_str)
            .or_else(|| payload.downcast_ref::<&str>().copied())
            .unwrap_or("native dynibo panic");
        PanicError::new_err(message.to_owned())
    })?
}
