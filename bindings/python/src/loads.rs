//! Python load values, reusable load buffers, and input conversion.
use crate::errors::core_error;
use dynibo::{IndexedLoad, LinkId, LoadBuffer as CoreLoadBuffer, Wrench};
use nalgebra::Vector3;
use pyo3::{exceptions::PyValueError, prelude::*, types::PyAny};
use std::sync::Arc;

#[pyclass(name = "Load", module = "dynibo", frozen, eq, skip_from_py_object)]
#[derive(Clone, Debug, PartialEq)]
pub(super) struct PyLoad {
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

pub(super) fn target_link(links: &[LinkId], target: usize) -> PyResult<LinkId> {
    links
        .get(target)
        .copied()
        .ok_or_else(|| PyValueError::new_err(format!("invalid link id {target}")))
}

/// Reusable native loads, created by a robot so integer IDs have a model scope.
#[pyclass(name = "LoadBuffer", module = "dynibo")]
pub(super) struct PyLoadBuffer {
    pub(super) inner: CoreLoadBuffer,
    pub(super) links: Arc<[LinkId]>,
}

#[pymethods]
impl PyLoadBuffer {
    #[pyo3(signature = (link_id, torque=(0.0, 0.0, 0.0).into(), force=(0.0, 0.0, 0.0).into()))]
    fn set(&mut self, link_id: usize, torque: [f64; 3], force: [f64; 3]) -> PyResult<()> {
        self.inner
            .set(
                target_link(&self.links, link_id)?,
                Wrench::new(Vector3::from(torque), Vector3::from(force)),
            )
            .map_err(core_error)
    }

    #[pyo3(signature = (link_id, torque=(0.0, 0.0, 0.0).into(), force=(0.0, 0.0, 0.0).into()))]
    fn add(&mut self, link_id: usize, torque: [f64; 3], force: [f64; 3]) -> PyResult<()> {
        self.inner
            .add(
                target_link(&self.links, link_id)?,
                Wrench::new(Vector3::from(torque), Vector3::from(force)),
            )
            .map_err(core_error)
    }

    fn remove(&mut self, link_id: usize) -> PyResult<()> {
        self.inner
            .remove(target_link(&self.links, link_id)?)
            .map_err(core_error)
    }

    fn clear(&mut self) {
        self.inner.clear();
    }
    fn __len__(&self) -> usize {
        self.inner.len()
    }
}

pub(super) enum LoadInput<'py> {
    Empty,
    List(Vec<IndexedLoad>),
    Buffer(PyRef<'py, PyLoadBuffer>),
}

impl LoadInput<'_> {
    pub(super) fn as_slice(&self) -> &[IndexedLoad] {
        match self {
            Self::Empty => &[],
            Self::List(loads) => loads,
            Self::Buffer(buffer) => buffer.inner.as_slice(),
        }
    }
}

pub(super) fn convert_loads<'py>(
    links: &[LinkId],
    loads: Option<&Bound<'py, PyAny>>,
) -> PyResult<LoadInput<'py>> {
    let Some(loads) = loads else {
        return Ok(LoadInput::Empty);
    };
    if let Ok(buffer) = loads.cast::<PyLoadBuffer>() {
        let buffer = buffer.try_borrow()?;
        if buffer.links.first() != links.first() {
            return Err(core_error(dynibo::Error::InvalidLinkId));
        }
        return Ok(LoadInput::Buffer(buffer));
    }
    let values = loads.extract::<Vec<PyRef<'py, PyLoad>>>()?;
    let converted = values
        .iter()
        .map(|load| {
            let link = target_link(links, load.link_id)?;
            let wrench = Wrench::new(Vector3::from(load.torque), Vector3::from(load.force));
            if !wrench.is_finite() {
                return Err(core_error(dynibo::Error::NonFiniteInput { input: "load" }));
            }
            Ok(IndexedLoad { link, wrench })
        })
        .collect::<PyResult<Vec<_>>>()?;
    Ok(LoadInput::List(converted))
}
