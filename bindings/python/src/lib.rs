//! Python module registration; implementation lives in the responsibility modules.
mod arrays;
mod calculation;
mod errors;
mod floating_robot;
mod loads;
mod robot;
mod values;

use errors::{DyniboError, ModelError, PanicError, SolverError};
use floating_robot::PyFloatingRobot;
use loads::{PyLoad, PyLoadBuffer};
use pyo3::{prelude::*, types::PyModule};
use robot::PyRobot;
use values::{PyBaseState, PyIkOptions, PyPose, PyTwist};

#[pymodule]
fn _dynibo(py: Python<'_>, module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyPose>()?;
    module.add_class::<PyTwist>()?;
    module.add_class::<PyBaseState>()?;
    module.add_class::<PyLoad>()?;
    module.add_class::<PyLoadBuffer>()?;
    module.add_class::<PyIkOptions>()?;
    module.add_class::<PyRobot>()?;
    module.add_class::<PyFloatingRobot>()?;
    module.add("DyniboError", py.get_type::<DyniboError>())?;
    module.add("ModelError", py.get_type::<ModelError>())?;
    module.add("SolverError", py.get_type::<SolverError>())?;
    module.add("PanicError", py.get_type::<PanicError>())?;
    Ok(())
}
