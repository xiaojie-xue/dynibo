//! Python Robot interface, keeping the class layout and construction local.
use crate::arrays::{
    prepare_input, require_same_length, run_with_output, validate_output, write_poses,
};
use crate::calculation::{
    Calculation, close_calculation, ensure_open, lock_calculation, with_robot,
};
use crate::errors::core_error;
use crate::loads::{PyLoadBuffer, convert_loads, target_link};
use crate::values::{PyIkOptions, PyPose, PyTwist};
use dynibo::{Frame, Robot as CoreRobot};
use dynibo_binding_support::LinkTable;
use nalgebra::Vector3;
use numpy::PyArray1;
use pyo3::{
    exceptions::PyValueError,
    prelude::*,
    types::{PyAny, PyType},
};
use std::{
    path::PathBuf,
    sync::{Mutex, atomic::AtomicBool},
};

#[pyclass(name = "Robot", module = "dynibo")]
pub(super) struct PyRobot {
    inner: Mutex<Option<Calculation<CoreRobot>>>,
    closed: AtomicBool,
    name: String,
    joint_count: usize,
    generalized_count: usize,
    link_count: usize,
    links: LinkTable,
}

impl PyRobot {
    fn load(path: PathBuf) -> PyResult<Self> {
        let robot = CoreRobot::from_urdf(path).map_err(core_error)?;
        let links = LinkTable::from_robot(&robot);
        let link_count = robot.link_count();
        let generalized_count = robot.generalized_count();
        Ok(Self {
            name: robot.name().to_owned(),
            joint_count: robot.joint_count(),
            generalized_count,
            link_count,
            links,
            closed: AtomicBool::new(false),
            inner: Mutex::new(Some(Calculation::new(robot, link_count, generalized_count))),
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
                inner: Mutex::new(Some(Calculation::new(
                    robot.fork(),
                    self.link_count,
                    self.generalized_count,
                ))),
                closed: AtomicBool::new(false),
                name: self.name.clone(),
                joint_count: self.joint_count,
                generalized_count: self.generalized_count,
                link_count: self.link_count,
                links: self.links.clone(),
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
        close_calculation(&self.inner, &self.closed, py)
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
    fn name(&self) -> PyResult<String> {
        ensure_open(&self.closed)?;
        Ok(self.name.clone())
    }

    #[getter]
    fn joint_count(&self) -> PyResult<usize> {
        ensure_open(&self.closed)?;
        Ok(self.joint_count)
    }

    #[getter]
    fn generalized_count(&self) -> PyResult<usize> {
        ensure_open(&self.closed)?;
        Ok(self.generalized_count)
    }

    #[getter]
    fn link_count(&self) -> PyResult<usize> {
        ensure_open(&self.closed)?;
        Ok(self.link_count)
    }

    fn link_id(&self, py: Python<'_>, name: &str) -> PyResult<usize> {
        with_robot(&self.inner, py, |robot| {
            let id = robot.link_id(name).map_err(core_error)?;
            self.links
                .index_of(id)
                .ok_or_else(|| PyValueError::new_err("link does not belong to this robot"))
        })
    }

    fn set_base_frame(&self, py: Python<'_>, frame: PyRef<'_, PyPose>) -> PyResult<()> {
        let frame = frame.to_frame()?;
        with_robot(&self.inner, py, |robot| {
            robot.set_base_frame(frame).map_err(core_error)
        })
    }

    #[getter]
    fn gravity_vector(&self, py: Python<'_>) -> PyResult<(f64, f64, f64)> {
        with_robot(&self.inner, py, |robot| {
            let g = robot.gravity_vector();
            Ok((g.x, g.y, g.z))
        })
    }

    fn set_gravity(&self, py: Python<'_>, gravity: [f64; 3]) -> PyResult<()> {
        with_robot(&self.inner, py, |robot| {
            robot
                .set_gravity(Vector3::from(gravity))
                .map_err(core_error)
        })
    }

    #[pyo3(signature = (q, out=None))]
    fn forward_kinematics_all<'py>(
        &self,
        py: Python<'py>,
        q: &Bound<'py, PyAny>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?;
        let output = validate_output(7 * self.link_count, out)?;
        let q = q.into_vec();
        run_with_output(py, &self.inner, output, |state, scratch| {
            state.robot.forward_kinematics_all(&q, &mut state.poses)?;
            write_poses(&state.poses, scratch);
            Ok(())
        })
    }

    fn forward_kinematics(
        &self,
        py: Python<'_>,
        q: &Bound<'_, PyAny>,
        target: usize,
    ) -> PyResult<PyPose> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?.into_vec();
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
        q: &Bound<'py, PyAny>,
        target: usize,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?;
        let target = target_link(&self.links, target)?;
        let output = validate_output(6 * self.generalized_count, out)?;
        let q = q.into_vec();
        run_with_output(py, &self.inner, output, |state, scratch| {
            state.robot.jacobian(&q, target, scratch)
        })
    }

    #[pyo3(signature = (q, qd, target, out=None))]
    fn jacobian_derivative<'py>(
        &self,
        py: Python<'py>,
        q: &Bound<'py, PyAny>,
        qd: &Bound<'py, PyAny>,
        target: usize,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?;
        let qd = prepare_input(qd)?;
        require_same_length(q.as_slice(), qd.as_slice(), "qd")?;
        let target = target_link(&self.links, target)?;
        let output = validate_output(6 * self.generalized_count, out)?;
        let (q, qd) = (q.into_vec(), qd.into_vec());
        run_with_output(py, &self.inner, output, |state, scratch| {
            state.robot.jacobian_derivative(&q, &qd, target, scratch)
        })
    }

    #[pyo3(signature = (q, qd, target, tool=None))]
    fn forward_velocity_kinematics(
        &self,
        py: Python<'_>,
        q: &Bound<'_, PyAny>,
        qd: &Bound<'_, PyAny>,
        target: usize,
        tool: Option<PyRef<'_, PyPose>>,
    ) -> PyResult<PyTwist> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?.into_vec();
        let qd = prepare_input(qd)?.into_vec();
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
        q: &Bound<'_, PyAny>,
        qd: &Bound<'_, PyAny>,
        qdd: &Bound<'_, PyAny>,
        target: usize,
    ) -> PyResult<PyTwist> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?.into_vec();
        let qd = prepare_input(qd)?.into_vec();
        let qdd = prepare_input(qdd)?.into_vec();
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
        q: &Bound<'py, PyAny>,
        loads: Option<Bound<'py, PyAny>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?;
        let loads = convert_loads(&self.links, loads.as_ref())?;
        let output = validate_output(self.generalized_count, out)?;
        let q = q.into_vec();
        run_with_output(py, &self.inner, output, |state, scratch| {
            state.robot.gravity(&q, &loads, scratch)
        })
    }

    #[pyo3(signature = (q, qd, qdd, loads=None, out=None))]
    fn inverse_dynamics<'py>(
        &self,
        py: Python<'py>,
        q: &Bound<'py, PyAny>,
        qd: &Bound<'py, PyAny>,
        qdd: &Bound<'py, PyAny>,
        loads: Option<Bound<'py, PyAny>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?;
        let qd = prepare_input(qd)?;
        let qdd = prepare_input(qdd)?;
        require_same_length(q.as_slice(), qd.as_slice(), "qd")?;
        require_same_length(q.as_slice(), qdd.as_slice(), "qdd")?;
        let loads = convert_loads(&self.links, loads.as_ref())?;
        let output = validate_output(self.generalized_count, out)?;
        let (q, qd, qdd) = (q.into_vec(), qd.into_vec(), qdd.into_vec());
        run_with_output(py, &self.inner, output, |state, scratch| {
            state.robot.inverse_dynamics(&q, &qd, &qdd, &loads, scratch)
        })
    }

    #[pyo3(signature = (q, qd, forces, loads=None, out=None))]
    fn forward_dynamics<'py>(
        &self,
        py: Python<'py>,
        q: &Bound<'py, PyAny>,
        qd: &Bound<'py, PyAny>,
        forces: &Bound<'py, PyAny>,
        loads: Option<Bound<'py, PyAny>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?;
        let qd = prepare_input(qd)?;
        let forces = prepare_input(forces)?;
        require_same_length(q.as_slice(), qd.as_slice(), "qd")?;
        let loads = convert_loads(&self.links, loads.as_ref())?;
        let output = validate_output(self.generalized_count, out)?;
        let (q, qd, forces) = (q.into_vec(), qd.into_vec(), forces.into_vec());
        run_with_output(py, &self.inner, output, |state, scratch| {
            state
                .robot
                .forward_dynamics(&q, &qd, &forces, &loads, scratch)
        })
    }

    #[pyo3(signature = (q, out=None))]
    fn mass_matrix<'py>(
        &self,
        py: Python<'py>,
        q: &Bound<'py, PyAny>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?;
        let output = validate_output(self.generalized_count * self.generalized_count, out)?;
        let q = q.into_vec();
        run_with_output(py, &self.inner, output, |state, scratch| {
            state.robot.mass_matrix(&q, scratch)
        })
    }

    #[pyo3(signature = (q, qd, out=None))]
    fn velocity_product_forces<'py>(
        &self,
        py: Python<'py>,
        q: &Bound<'py, PyAny>,
        qd: &Bound<'py, PyAny>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?;
        let qd = prepare_input(qd)?;
        require_same_length(q.as_slice(), qd.as_slice(), "qd")?;
        let output = validate_output(self.generalized_count, out)?;
        let (q, qd) = (q.into_vec(), qd.into_vec());
        run_with_output(py, &self.inner, output, |state, scratch| {
            state.robot.velocity_product_forces(&q, &qd, scratch)
        })
    }

    #[pyo3(signature = (q, target, desired, options=None, out=None))]
    fn inverse_kinematics<'py>(
        &self,
        py: Python<'py>,
        q: &Bound<'py, PyAny>,
        target: usize,
        desired: PyRef<'py, PyPose>,
        options: Option<PyRef<'py, PyIkOptions>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        ensure_open(&self.closed)?;
        let q = prepare_input(q)?;
        let target = target_link(&self.links, target)?;
        let desired = desired.to_frame()?;
        let options = options.map_or_else(PyIkOptions::default, |value| *value);
        let output = validate_output(self.joint_count, out)?;
        let q = q.into_vec();
        run_with_output(py, &self.inner, output, |state, scratch| {
            state
                .robot
                .inverse_kinematics(&q, target, &desired, options.into(), scratch)
        })
    }
}
