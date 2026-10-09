//! NumPy input adaptation and caller-owned output buffers.
use crate::errors::core_error;
use dynibo::Frame;
use numpy::{AllowTypeChange, PyArray1, PyArrayLike1, PyArrayMethods};
use pyo3::{exceptions::PyValueError, prelude::*};
use std::borrow::Cow;

pub(super) type ArrayInput<'py> = PyArrayLike1<'py, f64, AllowTypeChange>;

pub(super) fn input_slice<'a>(value: &'a ArrayInput<'_>) -> Cow<'a, [f64]> {
    value.as_slice().map_or_else(
        |_| Cow::Owned(value.as_array().iter().copied().collect()),
        Cow::Borrowed,
    )
}

pub(super) fn require_same_length(q: &[f64], other: &[f64], name: &str) -> PyResult<()> {
    if q.len() == other.len() {
        Ok(())
    } else {
        Err(PyValueError::new_err(format!(
            "q and {name} must have the same length"
        )))
    }
}

pub(super) fn calculate_output<'py>(
    py: Python<'py>,
    length: usize,
    out: Option<Bound<'py, PyArray1<f64>>>,
    calculate: impl FnOnce(&mut [f64]) -> dynibo::Result<()> + Send,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let array = out.unwrap_or_else(|| {
        // SAFETY: every dynibo calculation writes the complete output slice
        // before this array can be returned to Python.
        unsafe { PyArray1::new(py, length, false) }
    });
    if array.len()? != length {
        return Err(PyValueError::new_err(format!(
            "out must contain exactly {length} elements"
        )));
    }
    let mut writable = array
        .try_readwrite()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    let slice = writable
        .as_slice_mut()
        .map_err(|_| PyValueError::new_err("out must be a contiguous float64 array"))?;
    py.detach(|| calculate(slice)).map_err(core_error)?;
    drop(writable);
    Ok(array)
}

pub(super) fn write_poses(poses: &[Frame], output: &mut [f64]) {
    for (frame, row) in poses.iter().zip(output.chunks_exact_mut(7)) {
        row[..3].copy_from_slice(frame.translation.vector.as_slice());
        row[3..].copy_from_slice(frame.rotation.coords.as_slice());
    }
}
