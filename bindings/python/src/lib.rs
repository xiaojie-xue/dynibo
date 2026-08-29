use std::{
    borrow::Cow,
    panic::{AssertUnwindSafe, catch_unwind},
    path::PathBuf,
    sync::{Mutex, MutexGuard},
};

use dynibo::{
    BaseState as CoreBaseState, ErrorCategory, FloatingRobot as CoreFloatingRobot, Frame,
    IndexedLoad, InverseKinematicsOptions, LinkId, Robot as CoreRobot, Twist as CoreTwist, Wrench,
};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};
use numpy::{AllowTypeChange, PyArray1, PyArrayLike1, PyArrayMethods};
use pyo3::{
    create_exception,
    exceptions::{PyRuntimeError, PyTypeError, PyValueError},
    prelude::*,
    sync::MutexExt,
    types::{PyAny, PyBool, PyModule, PyType},
};

type ArrayInput<'py> = PyArrayLike1<'py, f64, AllowTypeChange>;

create_exception!(_dynibo, DyniboError, PyRuntimeError);
create_exception!(_dynibo, ModelError, DyniboError);
create_exception!(_dynibo, SolverError, DyniboError);
create_exception!(_dynibo, PanicError, DyniboError);

fn core_error(error: dynibo::Error) -> PyErr {
    let message = error.to_string();
    match error.category() {
        ErrorCategory::InvalidInput => PyValueError::new_err(message),
        ErrorCategory::Model => ModelError::new_err(message),
        ErrorCategory::Solver => SolverError::new_err(message),
    }
}

fn lock_error() -> PyErr {
    PanicError::new_err("robot workspace lock is poisoned")
}

fn catch_panic<T>(calculate: impl FnOnce() -> PyResult<T>) -> PyResult<T> {
    catch_unwind(AssertUnwindSafe(calculate)).map_err(|payload| {
        let message = payload
            .downcast_ref::<String>()
            .map(String::as_str)
            .or_else(|| payload.downcast_ref::<&str>().copied())
            .unwrap_or("native dynibo panic");
        PanicError::new_err(message.to_owned())
    })?
}

fn input_slice<'a>(value: &'a ArrayInput<'_>) -> Cow<'a, [f64]> {
    value.as_slice().map_or_else(
        |_| Cow::Owned(value.as_array().iter().copied().collect()),
        Cow::Borrowed,
    )
}

fn require_same_length(q: &[f64], other: &[f64], name: &str) -> PyResult<()> {
    if q.len() == other.len() {
        Ok(())
    } else {
        Err(PyValueError::new_err(format!(
            "q and {name} must have the same length"
        )))
    }
}

fn calculate_output<'py>(
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

fn checked_frame(translation: [f64; 3], rotation_xyzw: [f64; 4]) -> PyResult<Frame> {
    let [x, y, z, w] = rotation_xyzw;
    let norm_squared = x * x + y * y + z * z + w * w;
    if !translation.iter().all(|value| value.is_finite())
        || !norm_squared.is_finite()
        || norm_squared <= 1.0e-24
    {
        return Err(PyValueError::new_err(
            "pose contains non-finite values or a zero quaternion",
        ));
    }
    Ok(Frame::from_parts(
        Translation3::from(Vector3::from(translation)),
        UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
    ))
}

#[pyclass(name = "Pose", module = "dynibo", frozen, eq, skip_from_py_object)]
#[derive(Clone, Debug, PartialEq)]
struct PyPose {
    translation: [f64; 3],
    rotation_xyzw: [f64; 4],
}

impl PyPose {
    fn identity() -> Self {
        Self {
            translation: [0.0; 3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }

    fn to_frame(&self) -> PyResult<Frame> {
        checked_frame(self.translation, self.rotation_xyzw)
    }

    fn from_frame(frame: &Frame) -> Self {
        let quaternion = frame.rotation.quaternion();
        Self {
            translation: frame.translation.vector.into(),
            rotation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
        }
    }
}

#[pymethods]
impl PyPose {
    #[new]
    #[pyo3(signature = (translation=(0.0, 0.0, 0.0).into(), rotation_xyzw=(0.0, 0.0, 0.0, 1.0).into()))]
    fn new(translation: [f64; 3], rotation_xyzw: [f64; 4]) -> Self {
        Self {
            translation,
            rotation_xyzw,
        }
    }

    #[getter]
    fn translation(&self) -> (f64, f64, f64) {
        self.translation.into()
    }

    #[getter]
    fn rotation_xyzw(&self) -> (f64, f64, f64, f64) {
        self.rotation_xyzw.into()
    }
}

#[pyclass(name = "Twist", module = "dynibo", frozen, eq, skip_from_py_object)]
#[derive(Clone, Debug, PartialEq)]
struct PyTwist {
    angular: [f64; 3],
    linear: [f64; 3],
}

impl PyTwist {
    fn zero() -> Self {
        Self {
            angular: [0.0; 3],
            linear: [0.0; 3],
        }
    }

    fn to_core(&self) -> CoreTwist {
        CoreTwist::new(Vector3::from(self.angular), Vector3::from(self.linear))
    }

    fn from_core(value: CoreTwist) -> Self {
        Self {
            angular: value.angular.into(),
            linear: value.linear.into(),
        }
    }
}

#[pymethods]
impl PyTwist {
    #[new]
    #[pyo3(signature = (angular=(0.0, 0.0, 0.0).into(), linear=(0.0, 0.0, 0.0).into()))]
    fn new(angular: [f64; 3], linear: [f64; 3]) -> Self {
        Self { angular, linear }
    }

    #[getter]
    fn angular(&self) -> (f64, f64, f64) {
        self.angular.into()
    }

    #[getter]
    fn linear(&self) -> (f64, f64, f64) {
        self.linear.into()
    }
}

#[pyclass(name = "BaseState", module = "dynibo", frozen, eq, skip_from_py_object)]
#[derive(Clone, Debug, PartialEq)]
struct PyBaseState {
    frame: PyPose,
    velocity: PyTwist,
    acceleration: PyTwist,
}

impl PyBaseState {
    fn to_core(&self) -> PyResult<CoreBaseState> {
        CoreBaseState::new(
            self.frame.to_frame()?,
            self.velocity.to_core(),
            self.acceleration.to_core(),
        )
        .map_err(core_error)
    }
}

#[pymethods]
impl PyBaseState {
    #[new]
    #[pyo3(signature = (frame=None, velocity=None, acceleration=None))]
    fn new(
        frame: Option<PyRef<'_, PyPose>>,
        velocity: Option<PyRef<'_, PyTwist>>,
        acceleration: Option<PyRef<'_, PyTwist>>,
    ) -> Self {
        Self {
            frame: frame.map_or_else(PyPose::identity, |value| value.clone()),
            velocity: velocity.map_or_else(PyTwist::zero, |value| value.clone()),
            acceleration: acceleration.map_or_else(PyTwist::zero, |value| value.clone()),
        }
    }

    #[getter]
    fn frame(&self) -> PyPose {
        self.frame.clone()
    }

    #[getter]
    fn velocity(&self) -> PyTwist {
        self.velocity.clone()
    }

    #[getter]
    fn acceleration(&self) -> PyTwist {
        self.acceleration.clone()
    }
}

#[pyclass(name = "Load", module = "dynibo", frozen, eq, skip_from_py_object)]
#[derive(Clone, Debug, PartialEq)]
struct PyLoad {
    link_id: usize,
    torque: [f64; 3],
    force: [f64; 3],
}

#[pymethods]
impl PyLoad {
    #[new]
    #[pyo3(signature = (link_id, torque=(0.0, 0.0, 0.0).into(), force=(0.0, 0.0, 0.0).into()))]
    fn new(link_id: usize, torque: [f64; 3], force: [f64; 3]) -> Self {
        Self {
            link_id,
            torque,
            force,
        }
    }

    #[getter]
    fn link_id(&self) -> usize {
        self.link_id
    }

    #[getter]
    fn torque(&self) -> (f64, f64, f64) {
        self.torque.into()
    }

    #[getter]
    fn force(&self) -> (f64, f64, f64) {
        self.force.into()
    }
}

#[pyclass(name = "IkOptions", module = "dynibo", frozen, eq, skip_from_py_object)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct PyIkOptions {
    max_iterations: usize,
    translation_tolerance: f64,
    rotation_tolerance: f64,
    damping: f64,
    max_step_norm: f64,
}

impl Default for PyIkOptions {
    fn default() -> Self {
        let value = InverseKinematicsOptions::default();
        Self {
            max_iterations: value.max_iterations,
            translation_tolerance: value.translation_tolerance,
            rotation_tolerance: value.rotation_tolerance,
            damping: value.damping,
            max_step_norm: value.max_step_norm,
        }
    }
}

impl From<PyIkOptions> for InverseKinematicsOptions {
    fn from(value: PyIkOptions) -> Self {
        Self {
            max_iterations: value.max_iterations,
            translation_tolerance: value.translation_tolerance,
            rotation_tolerance: value.rotation_tolerance,
            damping: value.damping,
            max_step_norm: value.max_step_norm,
        }
    }
}

#[pymethods]
impl PyIkOptions {
    #[new]
    #[pyo3(signature = (max_iterations=None, translation_tolerance=1.0e-6, rotation_tolerance=1.0e-6, damping=1.0e-3, max_step_norm=0.5))]
    fn new(
        max_iterations: Option<&Bound<'_, PyAny>>,
        translation_tolerance: f64,
        rotation_tolerance: f64,
        damping: f64,
        max_step_norm: f64,
    ) -> PyResult<Self> {
        let max_iterations = if let Some(value) = max_iterations {
            if value.is_instance_of::<PyBool>() {
                return Err(PyTypeError::new_err("max_iterations must be an integer"));
            }
            value
                .extract::<isize>()
                .map_err(|_| PyTypeError::new_err("max_iterations must be an integer"))?
        } else {
            100
        };
        if max_iterations <= 0 {
            return Err(PyValueError::new_err(
                "max_iterations must be greater than zero",
            ));
        }
        Ok(Self {
            max_iterations: max_iterations as usize,
            translation_tolerance,
            rotation_tolerance,
            damping,
            max_step_norm,
        })
    }

    #[getter]
    fn max_iterations(&self) -> usize {
        self.max_iterations
    }

    #[getter]
    fn translation_tolerance(&self) -> f64 {
        self.translation_tolerance
    }

    #[getter]
    fn rotation_tolerance(&self) -> f64 {
        self.rotation_tolerance
    }

    #[getter]
    fn damping(&self) -> f64 {
        self.damping
    }

    #[getter]
    fn max_step_norm(&self) -> f64 {
        self.max_step_norm
    }
}

fn collect_links(robot: &CoreRobot) -> Vec<LinkId> {
    (0..robot.link_count())
        .map(|index| robot.link_id_at(index).expect("enumerated link is valid"))
        .collect()
}

fn collect_floating_links(robot: &CoreFloatingRobot) -> Vec<LinkId> {
    (0..robot.link_count())
        .map(|index| robot.link_id_at(index).expect("enumerated link is valid"))
        .collect()
}

fn target_link(links: &[LinkId], target: usize) -> PyResult<LinkId> {
    links
        .get(target)
        .copied()
        .ok_or_else(|| PyValueError::new_err(format!("invalid link id {target}")))
}

fn convert_loads(
    links: &[LinkId],
    loads: Option<&[PyRef<'_, PyLoad>]>,
) -> PyResult<Vec<IndexedLoad>> {
    loads
        .unwrap_or_default()
        .iter()
        .map(|load| {
            Ok(IndexedLoad {
                link: target_link(links, load.link_id)?,
                wrench: Wrench::new(Vector3::from(load.torque), Vector3::from(load.force)),
            })
        })
        .collect()
}

#[pyclass(name = "Robot", module = "dynibo")]
struct PyRobot {
    inner: Mutex<Option<CoreRobot>>,
    name: String,
    joint_count: usize,
    generalized_count: usize,
    link_count: usize,
    links: Vec<LinkId>,
}

impl PyRobot {
    fn load(path: PathBuf) -> PyResult<Self> {
        let robot = CoreRobot::from_urdf(path).map_err(core_error)?;
        let links = collect_links(&robot);
        Ok(Self {
            name: robot.name().to_owned(),
            joint_count: robot.joint_count(),
            generalized_count: robot.generalized_count(),
            link_count: robot.link_count(),
            links,
            inner: Mutex::new(Some(robot)),
        })
    }

    fn robot(&self, py: Python<'_>) -> PyResult<MutexGuard<'_, Option<CoreRobot>>> {
        let guard = self.inner.lock_py_attached(py).map_err(|_| lock_error())?;
        if guard.is_none() {
            Err(PyRuntimeError::new_err("robot is closed"))
        } else {
            Ok(guard)
        }
    }

    fn with_robot<T>(
        &self,
        py: Python<'_>,
        calculate: impl FnOnce(&mut CoreRobot) -> PyResult<T>,
    ) -> PyResult<T> {
        let mut guard = self.robot(py)?;
        catch_panic(|| calculate(guard.as_mut().expect("open robot checked")))
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

    fn close(&self, py: Python<'_>) -> PyResult<()> {
        self.inner
            .lock_py_attached(py)
            .map_err(|_| lock_error())?
            .take();
        Ok(())
    }

    fn __enter__<'py>(slf: PyRef<'py, Self>, py: Python<'py>) -> PyResult<PyRef<'py, Self>> {
        drop(slf.robot(py)?);
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
        drop(self.robot(py)?);
        Ok(self.name.clone())
    }

    #[getter]
    fn joint_count(&self, py: Python<'_>) -> PyResult<usize> {
        drop(self.robot(py)?);
        Ok(self.joint_count)
    }

    #[getter]
    fn generalized_count(&self, py: Python<'_>) -> PyResult<usize> {
        drop(self.robot(py)?);
        Ok(self.generalized_count)
    }

    #[getter]
    fn link_count(&self, py: Python<'_>) -> PyResult<usize> {
        drop(self.robot(py)?);
        Ok(self.link_count)
    }

    fn link_id(&self, py: Python<'_>, name: &str) -> PyResult<usize> {
        self.with_robot(py, |robot| {
            let id = robot.link_id(name).map_err(core_error)?;
            self.links
                .iter()
                .position(|candidate| *candidate == id)
                .ok_or_else(|| PyValueError::new_err("link does not belong to this robot"))
        })
    }

    fn set_base_frame(&self, py: Python<'_>, frame: PyRef<'_, PyPose>) -> PyResult<()> {
        let frame = frame.to_frame()?;
        self.with_robot(py, |robot| robot.set_base_frame(frame).map_err(core_error))
    }

    fn forward_kinematics(
        &self,
        py: Python<'_>,
        q: ArrayInput<'_>,
        target: usize,
    ) -> PyResult<PyPose> {
        let q = input_slice(&q);
        let target = target_link(&self.links, target)?;
        self.with_robot(py, |robot| {
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
        self.with_robot(py, |robot| {
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
        self.with_robot(py, |robot| {
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
        self.with_robot(py, |robot| {
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
        self.with_robot(py, |robot| {
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
        loads: Option<Vec<PyRef<'py, PyLoad>>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let loads = convert_loads(&self.links, loads.as_deref())?;
        self.with_robot(py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.gravity(&q, &loads, output)
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
        loads: Option<Vec<PyRef<'py, PyLoad>>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        let qdd = input_slice(&qdd);
        require_same_length(&q, &qd, "qd")?;
        require_same_length(&q, &qdd, "qdd")?;
        let loads = convert_loads(&self.links, loads.as_deref())?;
        self.with_robot(py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.inverse_dynamics(&q, &qd, &qdd, &loads, output)
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
        loads: Option<Vec<PyRef<'py, PyLoad>>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        let forces = input_slice(&forces);
        require_same_length(&q, &qd, "qd")?;
        let loads = convert_loads(&self.links, loads.as_deref())?;
        self.with_robot(py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.forward_dynamics(&q, &qd, &forces, &loads, output)
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
        self.with_robot(py, |robot| {
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
        self.with_robot(py, |robot| {
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
        self.with_robot(py, |robot| {
            calculate_output(py, self.joint_count, out, |output| {
                robot.inverse_kinematics(&q, target, &desired, options.into(), output)
            })
        })
    }
}

#[pyclass(name = "FloatingRobot", module = "dynibo")]
struct PyFloatingRobot {
    inner: Mutex<Option<CoreFloatingRobot>>,
    name: String,
    joint_count: usize,
    generalized_count: usize,
    link_count: usize,
    links: Vec<LinkId>,
}

impl PyFloatingRobot {
    fn load(path: PathBuf) -> PyResult<Self> {
        let robot = CoreFloatingRobot::from_urdf(path).map_err(core_error)?;
        let links = collect_floating_links(&robot);
        Ok(Self {
            name: robot.name().to_owned(),
            joint_count: robot.joint_count(),
            generalized_count: robot.generalized_count(),
            link_count: robot.link_count(),
            links,
            inner: Mutex::new(Some(robot)),
        })
    }

    fn robot(&self, py: Python<'_>) -> PyResult<MutexGuard<'_, Option<CoreFloatingRobot>>> {
        let guard = self.inner.lock_py_attached(py).map_err(|_| lock_error())?;
        if guard.is_none() {
            Err(PyRuntimeError::new_err("robot is closed"))
        } else {
            Ok(guard)
        }
    }

    fn with_robot<T>(
        &self,
        py: Python<'_>,
        calculate: impl FnOnce(&mut CoreFloatingRobot) -> PyResult<T>,
    ) -> PyResult<T> {
        let mut guard = self.robot(py)?;
        catch_panic(|| calculate(guard.as_mut().expect("open robot checked")))
    }
}

#[pymethods]
impl PyFloatingRobot {
    #[new]
    fn new(path: PathBuf) -> PyResult<Self> {
        Self::load(path)
    }

    #[classmethod]
    fn from_urdf(_class: &Bound<'_, PyType>, path: PathBuf) -> PyResult<Self> {
        Self::load(path)
    }

    fn close(&self, py: Python<'_>) -> PyResult<()> {
        self.inner
            .lock_py_attached(py)
            .map_err(|_| lock_error())?
            .take();
        Ok(())
    }

    fn __enter__<'py>(slf: PyRef<'py, Self>, py: Python<'py>) -> PyResult<PyRef<'py, Self>> {
        drop(slf.robot(py)?);
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
        drop(self.robot(py)?);
        Ok(self.name.clone())
    }

    #[getter]
    fn joint_count(&self, py: Python<'_>) -> PyResult<usize> {
        drop(self.robot(py)?);
        Ok(self.joint_count)
    }

    #[getter]
    fn generalized_count(&self, py: Python<'_>) -> PyResult<usize> {
        drop(self.robot(py)?);
        Ok(self.generalized_count)
    }

    #[getter]
    fn link_count(&self, py: Python<'_>) -> PyResult<usize> {
        drop(self.robot(py)?);
        Ok(self.link_count)
    }

    fn link_id(&self, py: Python<'_>, name: &str) -> PyResult<usize> {
        self.with_robot(py, |robot| {
            let id = robot.link_id(name).map_err(core_error)?;
            self.links
                .iter()
                .position(|candidate| *candidate == id)
                .ok_or_else(|| PyValueError::new_err("link does not belong to this robot"))
        })
    }

    fn forward_kinematics(
        &self,
        py: Python<'_>,
        base: PyRef<'_, PyBaseState>,
        q: ArrayInput<'_>,
        target: usize,
    ) -> PyResult<PyPose> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        let target = target_link(&self.links, target)?;
        self.with_robot(py, |robot| {
            py.detach(|| robot.forward_kinematics(&base, &q, target))
                .map(|frame| PyPose::from_frame(&frame))
                .map_err(core_error)
        })
    }

    #[pyo3(signature = (base, q, target, out=None))]
    fn jacobian<'py>(
        &self,
        py: Python<'py>,
        base: PyRef<'py, PyBaseState>,
        q: ArrayInput<'py>,
        target: usize,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        let target = target_link(&self.links, target)?;
        self.with_robot(py, |robot| {
            calculate_output(py, 6 * self.generalized_count, out, |output| {
                robot.jacobian(&base, &q, target, output)
            })
        })
    }

    #[pyo3(signature = (base, q, qd, target, out=None))]
    fn jacobian_derivative<'py>(
        &self,
        py: Python<'py>,
        base: PyRef<'py, PyBaseState>,
        q: ArrayInput<'py>,
        qd: ArrayInput<'py>,
        target: usize,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        require_same_length(&q, &qd, "qd")?;
        let target = target_link(&self.links, target)?;
        self.with_robot(py, |robot| {
            calculate_output(py, 6 * self.generalized_count, out, |output| {
                robot.jacobian_derivative(&base, &q, &qd, target, output)
            })
        })
    }

    #[pyo3(signature = (base, q, qd, target, tool=None))]
    fn forward_velocity_kinematics(
        &self,
        py: Python<'_>,
        base: PyRef<'_, PyBaseState>,
        q: ArrayInput<'_>,
        qd: ArrayInput<'_>,
        target: usize,
        tool: Option<PyRef<'_, PyPose>>,
    ) -> PyResult<PyTwist> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        require_same_length(&q, &qd, "qd")?;
        let target = target_link(&self.links, target)?;
        let tool = tool.map_or_else(|| Ok(Frame::identity()), |value| value.to_frame())?;
        self.with_robot(py, |robot| {
            py.detach(|| robot.forward_velocity_kinematics(&base, &q, &qd, target, &tool))
                .map(PyTwist::from_core)
                .map_err(core_error)
        })
    }

    fn forward_acceleration_kinematics(
        &self,
        py: Python<'_>,
        base: PyRef<'_, PyBaseState>,
        q: ArrayInput<'_>,
        qd: ArrayInput<'_>,
        qdd: ArrayInput<'_>,
        target: usize,
    ) -> PyResult<PyTwist> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        let qdd = input_slice(&qdd);
        require_same_length(&q, &qd, "qd")?;
        require_same_length(&q, &qdd, "qdd")?;
        let target = target_link(&self.links, target)?;
        self.with_robot(py, |robot| {
            py.detach(|| robot.forward_acceleration_kinematics(&base, &q, &qd, &qdd, target))
                .map(PyTwist::from_core)
                .map_err(core_error)
        })
    }

    #[pyo3(signature = (base, q, loads=None, out=None))]
    fn gravity<'py>(
        &self,
        py: Python<'py>,
        base: PyRef<'py, PyBaseState>,
        q: ArrayInput<'py>,
        loads: Option<Vec<PyRef<'py, PyLoad>>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        let loads = convert_loads(&self.links, loads.as_deref())?;
        self.with_robot(py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.gravity(&base, &q, &loads, output)
            })
        })
    }

    #[pyo3(signature = (base, q, qd, qdd, loads=None, out=None))]
    #[allow(clippy::too_many_arguments)]
    fn inverse_dynamics<'py>(
        &self,
        py: Python<'py>,
        base: PyRef<'py, PyBaseState>,
        q: ArrayInput<'py>,
        qd: ArrayInput<'py>,
        qdd: ArrayInput<'py>,
        loads: Option<Vec<PyRef<'py, PyLoad>>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        let qdd = input_slice(&qdd);
        require_same_length(&q, &qd, "qd")?;
        require_same_length(&q, &qdd, "qdd")?;
        let loads = convert_loads(&self.links, loads.as_deref())?;
        self.with_robot(py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.inverse_dynamics(&base, &q, &qd, &qdd, &loads, output)
            })
        })
    }

    #[pyo3(signature = (base, q, qd, forces, loads=None, out=None))]
    #[allow(clippy::too_many_arguments)]
    fn forward_dynamics<'py>(
        &self,
        py: Python<'py>,
        base: PyRef<'py, PyBaseState>,
        q: ArrayInput<'py>,
        qd: ArrayInput<'py>,
        forces: ArrayInput<'py>,
        loads: Option<Vec<PyRef<'py, PyLoad>>>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        let forces = input_slice(&forces);
        require_same_length(&q, &qd, "qd")?;
        let loads = convert_loads(&self.links, loads.as_deref())?;
        self.with_robot(py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.forward_dynamics(&base, &q, &qd, &forces, &loads, output)
            })
        })
    }

    #[pyo3(signature = (base, q, out=None))]
    fn mass_matrix<'py>(
        &self,
        py: Python<'py>,
        base: PyRef<'py, PyBaseState>,
        q: ArrayInput<'py>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        self.with_robot(py, |robot| {
            calculate_output(
                py,
                self.generalized_count * self.generalized_count,
                out,
                |output| robot.mass_matrix(&base, &q, output),
            )
        })
    }

    #[pyo3(signature = (base, q, qd, out=None))]
    fn velocity_product_forces<'py>(
        &self,
        py: Python<'py>,
        base: PyRef<'py, PyBaseState>,
        q: ArrayInput<'py>,
        qd: ArrayInput<'py>,
        out: Option<Bound<'py, PyArray1<f64>>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let base = base.to_core()?;
        let q = input_slice(&q);
        let qd = input_slice(&qd);
        require_same_length(&q, &qd, "qd")?;
        self.with_robot(py, |robot| {
            calculate_output(py, self.generalized_count, out, |output| {
                robot.velocity_product_forces(&base, &q, &qd, output)
            })
        })
    }
}

#[pymodule]
fn _dynibo(py: Python<'_>, module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyPose>()?;
    module.add_class::<PyTwist>()?;
    module.add_class::<PyBaseState>()?;
    module.add_class::<PyLoad>()?;
    module.add_class::<PyIkOptions>()?;
    module.add_class::<PyRobot>()?;
    module.add_class::<PyFloatingRobot>()?;
    module.add("DyniboError", py.get_type::<DyniboError>())?;
    module.add("ModelError", py.get_type::<ModelError>())?;
    module.add("SolverError", py.get_type::<SolverError>())?;
    module.add("PanicError", py.get_type::<PanicError>())?;
    Ok(())
}
