//! Python spatial values and inverse-kinematics options.
use crate::errors::core_error;
use dynibo::{BaseState as CoreBaseState, Frame, InverseKinematicsOptions, Twist as CoreTwist};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};
use pyo3::{
    exceptions::{PyTypeError, PyValueError},
    prelude::*,
    types::{PyAny, PyBool},
};

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
pub(super) struct PyPose {
    translation: [f64; 3],
    rotation_xyzw: [f64; 4],
}

impl PyPose {
    pub(super) fn identity() -> Self {
        Self {
            translation: [0.0; 3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }

    pub(super) fn to_frame(&self) -> PyResult<Frame> {
        checked_frame(self.translation, self.rotation_xyzw)
    }

    pub(super) fn from_frame(frame: &Frame) -> Self {
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
pub(super) struct PyTwist {
    angular: [f64; 3],
    linear: [f64; 3],
}

impl PyTwist {
    pub(super) fn zero() -> Self {
        Self {
            angular: [0.0; 3],
            linear: [0.0; 3],
        }
    }

    pub(super) fn to_core(&self) -> CoreTwist {
        CoreTwist::new(Vector3::from(self.angular), Vector3::from(self.linear))
    }

    pub(super) fn from_core(value: CoreTwist) -> Self {
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
pub(super) struct PyBaseState {
    frame: PyPose,
    velocity: PyTwist,
    acceleration: PyTwist,
}

impl PyBaseState {
    pub(super) fn to_core(&self) -> PyResult<CoreBaseState> {
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

#[pyclass(name = "IkOptions", module = "dynibo", frozen, eq, skip_from_py_object)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct PyIkOptions {
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
