//! Python Robot interface, keeping the class layout and construction local.
use crate::arrays::{ArrayInput, calculate_output, input_slice, require_same_length, write_poses};
use crate::calculation::{
    Calculation, close_calculation, lock_calculation, with_calculation, with_robot,
};
use crate::errors::core_error;
use crate::loads::{PyLoadBuffer, convert_loads, target_link};
use crate::values::{PyIkOptions, PyPose, PyTwist};
use dynibo::{Frame, LinkId, Robot as CoreRobot};
use numpy::PyArray1;
use pyo3::{
    exceptions::PyValueError,
    prelude::*,
    types::{PyAny, PyType},
};
use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
};

fn collect_links(robot: &CoreRobot) -> Vec<LinkId> {
    (0..robot.link_count())
        .map(|index| robot.link_id_at(index).expect("enumerated link is valid"))
        .collect()
}

#[pyclass(name = "Robot", module = "dynibo")]
pub(super) struct PyRobot {
    inner: Mutex<Option<Calculation<CoreRobot>>>,
    name: String,
    joint_count: usize,
    generalized_count: usize,
    link_count: usize,
    links: Arc<[LinkId]>,
}

impl PyRobot {
    fn load(path: PathBuf) -> PyResult<Self> {
        let robot = CoreRobot::from_urdf(path).map_err(core_error)?;
        let links = collect_links(&robot).into();
        Ok(Self {
            name: robot.name().to_owned(),
            joint_count: robot.joint_count(),
            generalized_count: robot.generalized_count(),
            link_count: robot.link_count(),
            links,
            inner: Mutex::new(Some(Calculation {
                poses: vec![Frame::identity(); robot.link_count()],
                robot,
            })),
        })
    }
}

#[pymethods]
impl PyRobot {
    #[new]
    fn new(path: PathBuf) -> PyResult<Self> {
        Self::load(path)
    }

    #[classmethod]
    fn from_urdf(_class: &Bound<'_, PyType>, path: PathBuf) -> PyResult<Self> {
        Self::load(path)
    }

    fn fork(&self, py: Python<'_>) -> PyResult<Self> {
        with_robot(&self.inner, py, |robot| {
            Ok(Self {
                inner: Mutex::new(Some(Calculation {
                    robot: robot.fork(),
                    poses: vec![Frame::identity(); self.link_count],
                })),
                name: self.name.clone(),
                joint_count: self.joint_count,
                generalized_count: self.generalized_count,
                link_count: self.link_count,
                links: Arc::clone(&self.links),
            })
        })
    }

    fn load_buffer(&self, py: Python<'_>) -> PyResult<PyLoadBuffer> {
        with_robot(&self.inner, py, |robot| {
            Ok(PyLoadBuffer {
                inner: robot.load_buffer(),
                links: self.links.clone(),
            })
        })
    }

    fn close(&self, py: Python<'_>) -> PyResult<()> {
        close_calculation(&self.inner, py)
    }

    fn __enter__<'py>(slf: PyRef<'py, Self>, py: Python<'py>) -> PyResult<PyRef<'py, Self>> {
        drop(lock_calculation(&slf.inner, py)?);
        Ok(slf)
    }

    fn __exit__(
        &self,
        py: Python<'_>,
        _exc_type: &Bound<'_, PyAny>,
        _exc_value: &Bound<'_, PyAny>,
        _traceback: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        self.close(py)
    }

    #[getter]
    fn name(&self, py: Python<'_>) -> PyResult<String> {
        drop(lock_calculation(&self.inner, py)?);
        Ok(self.name.clone())
    }

    #[getter]
    fn joint_count(&self, py: Python<'_>) -> PyResult<usize> {
        drop(lock_calculation(&self.inner, py)?);
        Ok(self.joint_count)
    }

    #[getter]
    fn generalized_count(&self, py: Python<'_>) -> PyResult<usize> {
        drop(lock_calculation(&self.inner, py)?);
        Ok(self.generalized_count)
    }

    #[getter]
    fn link_count(&self, py: Python<'_>) -> PyResult<usize> {
        drop(lock_calculation(&self.inner, py)?);
        Ok(self.link_count)
    }

    fn link_id(&self, py: Python<'_>, name: &str) -> PyResult<usize> {
        with_robot(&self.inner, py, |robot| {
            let id = robot.link_id(name).map_err(core_error)?;
            self.links
                .iter()
                .position(|candidate| *candidate == id)
                .ok_or_else(|| PyValueError::new_err("link does not belong to this robot"))
        })
    }

    fn set_base_frame(&self, py: Python<'_>, frame: PyRef<'_, PyPose>) -> PyResult<()> {
        let frame = frame.to_frame()?;
        with_robot(&self.inner, py, |robot| {
            robot.set_base_frame(frame).map_err(core_error)
        })
    }

    #[pyo3(signature = (q, out=None))]
    fn forward_kinematics_all<'py>(
        &self,
        py: Python<'py>,
        q: ArrayInput<'py>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        with_calculation(&self.inner, py, |state| {
            calculate_output(py, 7 * self.link_count, out, |output| {
                state.robot.forward_kinematics_all(&q, &mut state.poses)?;
                write_poses(&state.poses, output);
                Ok(())
            })
        })
    }

    fn forward_kinematics(
        &self,
        py: Python<'_>,
        q: ArrayInput<'_>,
        target: usize,
    ) -> PyResult<PyPose> {
        let q = input_slice(&q);
        let target = target_link(&self.links, target)?;
        with_robot(&self.inner, py, |robot| {
            py.detach(|| robot.forward_kinematics(&q, target))
                .map(|frame| PyPose::from_frame(&frame))
                .map_err(core_error)
        })
    }

    #[pyo3(signature = (q, target, out=None))]
    fn jacobian<'py>(
        &self,
        py: Python<'py>,
        q: ArrayInput<'py>,
        target: usize,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let target = target_link(&self.links, target)?;
        with_robot(&self.inner, py, |robot| {
            calculate_output(py, 6 * self.generalized_count, out, |output| {
                robot.jacobian(&q, target, output)
            })
        })
    }

    #[pyo3(signature = (q, qd, target, out=None))]
    fn jacobian_derivative<'py>(
        &self,
        py: Python<'py>,
        q: ArrayInput<'py>,
        qd: ArrayInput<'py>,
        target: usize,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        require_same_length(&q, &qd, "qd")?;
        let target = target_link(&self.links, target)?;
        with_robot(&self.inner, py, |robot| {
            calculate_output(py, 6 * self.generalized_count, out, |output| {
                robot.jacobian_derivative(&q, &qd, target, output)
            })
        })
    }

    #[pyo3(signature = (q, qd, target, tool=None))]
    fn forward_velocity_kinematics(
        &self,
        py: Python<'_>,
        q: ArrayInput<'_>,
        qd: ArrayInput<'_>,
        target: usize,
        tool: Option<PyRef<'_, PyPose>>,
    ) -> PyResult<PyTwist> {
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        require_same_length(&q, &qd, "qd")?;
        let target = target_link(&self.links, target)?;
        let tool = tool.map_or_else(|| Ok(Frame::identity()), |value| value.to_frame())?;
        with_robot(&self.inner, py, |robot| {
            py.detach(|| robot.forward_velocity_kinematics(&q, &qd, target, &tool))
                .map(PyTwist::from_core)
                .map_err(core_error)
        })
    }

    fn forward_acceleration_kinematics(
        &self,
        py: Python<'_>,
        q: ArrayInput<'_>,
        qd: ArrayInput<'_>,
        qdd: ArrayInput<'_>,
        target: usize,
    ) -> PyResult<PyTwist> {
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        let qdd = input_slice(&qdd);
        require_same_length(&q, &qd, "qd")?;
        require_same_length(&q, &qdd, "qdd")?;
        let target = target_link(&self.links, target)?;
        with_robot(&self.inner, py, |robot| {
            py.detach(|| robot.forward_acceleration_kinematics(&q, &qd, &qdd, target))
                .map(PyTwist::from_core)
                .map_err(core_error)
        })
    }

    #[pyo3(signature = (q, loads=None, out=None))]
    fn gravity<'py>(
        &self,
        py: Python<'py>,
        q: ArrayInput<'py>,
        loads: Option<Bound<'py, PyAny>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let load_guard = convert_loads(&self.links, loads.as_ref())?;
        let loads = load_guard.as_slice();
        with_robot(&self.inner, py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.gravity(&q, loads, output)
            })
        })
    }

    #[pyo3(signature = (q, qd, qdd, loads=None, out=None))]
    fn inverse_dynamics<'py>(
        &self,
        py: Python<'py>,
        q: ArrayInput<'py>,
        qd: ArrayInput<'py>,
        qdd: ArrayInput<'py>,
        loads: Option<Bound<'py, PyAny>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        let qdd = input_slice(&qdd);
        require_same_length(&q, &qd, "qd")?;
        require_same_length(&q, &qdd, "qdd")?;
        let load_guard = convert_loads(&self.links, loads.as_ref())?;
        let loads = load_guard.as_slice();
        with_robot(&self.inner, py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.inverse_dynamics(&q, &qd, &qdd, loads, output)
            })
        })
    }

    #[pyo3(signature = (q, qd, forces, loads=None, out=None))]
    fn forward_dynamics<'py>(
        &self,
        py: Python<'py>,
        q: ArrayInput<'py>,
        qd: ArrayInput<'py>,
        forces: ArrayInput<'py>,
        loads: Option<Bound<'py, PyAny>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        let forces = input_slice(&forces);
        require_same_length(&q, &qd, "qd")?;
        let load_guard = convert_loads(&self.links, loads.as_ref())?;
        let loads = load_guard.as_slice();
        with_robot(&self.inner, py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.forward_dynamics(&q, &qd, &forces, loads, output)
            })
        })
    }

    #[pyo3(signature = (q, out=None))]
    fn mass_matrix<'py>(
        &self,
        py: Python<'py>,
        q: ArrayInput<'py>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        with_robot(&self.inner, py, |robot| {
            calculate_output(
                py,
                self.generalized_count * self.generalized_count,
                out,
                |output| robot.mass_matrix(&q, output),
            )
        })
    }

    #[pyo3(signature = (q, qd, out=None))]
    fn velocity_product_forces<'py>(
        &self,
        py: Python<'py>,
        q: ArrayInput<'py>,
        qd: ArrayInput<'py>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        require_same_length(&q, &qd, "qd")?;
        with_robot(&self.inner, py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.velocity_product_forces(&q, &qd, output)
            })
        })
    }

    #[pyo3(signature = (q, target, desired, options=None, out=None))]
    fn inverse_kinematics<'py>(
        &self,
        py: Python<'py>,
        q: ArrayInput<'py>,
        target: usize,
        desired: PyRef<'py, PyPose>,
        options: Option<PyRef<'py, PyIkOptions>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let target = target_link(&self.links, target)?;
        let desired = desired.to_frame()?;
        let options = options.map_or_else(PyIkOptions::default, |value| *value);
        with_robot(&self.inner, py, |robot| {
            calculate_output(py, self.joint_count, out, |output| {
                robot.inverse_kinematics(&q, target, &desired, options.into(), output)
            })
        })
    }
}
