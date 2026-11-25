//! NumPy input adaptation and caller-owned output buffers.
use crate::calculation::{Calculation, with_calculation};
use crate::errors::core_error;
use dynibo::Frame;
use numpy::{AllowTypeChange, PyArray1, PyArrayLike1, PyArrayMethods, PyUntypedArray};
use pyo3::{
    exceptions::{PyTypeError, PyValueError},
    prelude::*,
    types::PyAny,
};
use std::sync::Mutex;

pub(super) type ArrayInput<'py> = PyArrayLike1<'py, f64, AllowTypeChange>;

/// Input vector copied into Rust memory while keeping the NumPy readonly borrow
/// alive until output overlap checks complete.
pub(super) struct PreparedInput<'py> {
    _borrow: ArrayInput<'py>,
    values: Vec<f64>,
}

impl PreparedInput<'_> {
    pub(super) fn as_slice(&self) -> &[f64] {
        &self.values
    }

    /// Drops the NumPy readonly borrow and returns the owned Rust copy for use
    /// while the GIL is released.
    pub(super) fn into_vec(self) -> Vec<f64> {
        self.values
    }
}

pub(super) fn prepare_input<'py>(value: &Bound<'py, PyAny>) -> PyResult<PreparedInput<'py>> {
    if let Ok(untyped) = value.cast::<PyUntypedArray>() {
        let kind: String = untyped.getattr("dtype")?.getattr("kind")?.extract()?;
        if !matches!(kind.as_str(), "f" | "i" | "u") {
            return Err(PyTypeError::new_err(
                "array input must have a real numeric dtype",
            ));
        }
    }
    let array: ArrayInput<'py> = value.extract()?;
    let values = if let Ok(slice) = array.as_slice() {
        slice.to_vec()
    } else {
        // NumPy must interpret byte strides and unaligned data before Rust borrows it.
        let contiguous = array.cast_array::<f64>(false)?;
        contiguous
            .to_vec()
            .map_err(|error| PyValueError::new_err(error.to_string()))?
    };
    Ok(PreparedInput {
        _borrow: array,
        values,
    })
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

pub(super) struct ValidatedOutput<'py> {
    array: Option<Bound<'py, PyArray1<f64>>>,
    length: usize,
}

/// Validates caller-supplied `out` while input NumPy borrows remain active so
/// overlapping input/output buffers are rejected before releasing the GIL.
pub(super) fn validate_output<'py>(
    length: usize,
    out: Option<Bound<'py, PyArray1<f64>>>,
) -> PyResult<ValidatedOutput<'py>> {
    if let Some(array) = out {
        if array.len()? != length {
            return Err(PyValueError::new_err(format!(
                "out must contain exactly {length} elements"
            )));
        }
        {
            let mut writable = array
                .try_readwrite()
                .map_err(|error| PyValueError::new_err(error.to_string()))?;
            let _ = writable
                .as_slice_mut()
                .map_err(|_| PyValueError::new_err("out must be a contiguous float64 array"))?;
        }
        Ok(ValidatedOutput {
            array: Some(array),
            length,
        })
    } else {
        Ok(ValidatedOutput {
            array: None,
            length,
        })
    }
}

/// Runs a calculation into an owned Rust scratch buffer with the GIL released,
/// then writes into `out` (or allocates a new array after releasing the mutex).
pub(super) fn run_with_output<'py, R: Send>(
    py: Python<'py>,
    inner: &Mutex<Option<Calculation<R>>>,
    output: ValidatedOutput<'py>,
    calculate: impl FnOnce(&mut Calculation<R>, &mut [f64]) -> dynibo::Result<()> + Send,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let ValidatedOutput { array, length } = output;
    let allocated = with_calculation(inner, py, |state| {
        if state.output.len() < length {
            state.output.resize(length, 0.0);
        }
        let mut scratch = std::mem::take(&mut state.output);
        let result = py.detach(|| calculate(state, &mut scratch[..length]));
        state.output = scratch;
        result.map_err(core_error)?;
        if let Some(ref target) = array {
            let mut writable = target
                .try_readwrite()
                .map_err(|error| PyValueError::new_err(error.to_string()))?;
            let slice = writable
                .as_slice_mut()
                .map_err(|_| PyValueError::new_err("out must be a contiguous float64 array"))?;
            slice.copy_from_slice(&state.output[..length]);
            Ok(None)
        } else {
            Ok(Some(state.output[..length].to_vec()))
        }
    })?;
    if let Some(target) = array {
        Ok(target)
    } else {
        Ok(PyArray1::from_vec(
            py,
            allocated.expect("allocated output returned when out is None"),
        ))
    }
}

pub(super) fn write_poses(poses: &[Frame], output: &mut [f64]) {
    for (frame, row) in poses.iter().zip(output.as_chunks_mut::<7>().0) {
        row[..3].copy_from_slice(frame.translation.vector.as_slice());
        row[3..].copy_from_slice(frame.rotation.coords.as_slice());
    }
}
