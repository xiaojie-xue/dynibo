//! Python load values, model-scoped link IDs, reusable load buffers, and conversion.
use crate::errors::core_error;
use dynibo::{IndexedLoad, LinkId, LoadBuffer as CoreLoadBuffer, Wrench};
use dynibo_binding_support::LinkTable;
use nalgebra::Vector3;
use pyo3::{
    exceptions::{PyDeprecationWarning, PyTypeError, PyValueError},
    prelude::*,
    types::{PyAny, PyBool, PyDict},
};

const PLAIN_INT_WARNING: &std::ffi::CStr =
    c"passing a plain int as link_id is deprecated; use robot.link_id(...) instead";

/// Opaque model-identity token attached to Python `LinkId` instances.
#[pyclass(name = "_ModelTag", module = "dynibo", frozen, eq, skip_from_py_object)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct PyModelTag(pub(super) LinkId);

pub(super) fn make_py_link_id(
    py: Python<'_>,
    links: &LinkTable,
    index: usize,
) -> PyResult<Py<PyAny>> {
    let root = links
        .root()
        .ok_or_else(|| PyValueError::new_err("model has no links"))?;
    make_py_link_id_with_root(py, root, index)
}

fn make_py_link_id_with_root(py: Python<'_>, root: LinkId, index: usize) -> PyResult<Py<PyAny>> {
    let dynibo_mod = py.import("dynibo")?;
    let link_id_cls = dynibo_mod.getattr("LinkId")?;
    let kwargs = PyDict::new(py);
    kwargs.set_item("_model_tag", Py::new(py, PyModelTag(root))?)?;
    Ok(link_id_cls.call((index,), Some(&kwargs))?.unbind())
}

fn extract_link_parts(value: &Bound<'_, PyAny>) -> PyResult<(usize, Option<LinkId>)> {
    if value.is_instance_of::<PyBool>() {
        return Err(PyTypeError::new_err("link_id must be an integer"));
    }
    let raw_index: isize = value
        .extract()
        .map_err(|_| PyTypeError::new_err("link_id must be an integer"))?;
    if raw_index < 0 {
        return Err(PyValueError::new_err(format!(
            "invalid link id {raw_index}"
        )));
    }
    let model_tag = if let Ok(attr) = value.getattr("_model_tag") {
        if attr.is_none() {
            None
        } else {
            let tag: PyRef<'_, PyModelTag> = attr
                .extract()
                .map_err(|_| PyValueError::new_err("invalid link id model tag"))?;
            Some(tag.0)
        }
    } else {
        None
    };
    Ok((raw_index as usize, model_tag))
}

/// Resolves a Python `LinkId` (or deprecated plain `int`) against `links`.
pub(super) fn target_link(
    py: Python<'_>,
    links: &LinkTable,
    target: &Bound<'_, PyAny>,
) -> PyResult<LinkId> {
    let (index, model_tag) = extract_link_parts(target)?;
    resolve_tagged_index(py, links, index, model_tag)
}

fn resolve_tagged_index(
    py: Python<'_>,
    links: &LinkTable,
    index: usize,
    model_tag: Option<LinkId>,
) -> PyResult<LinkId> {
    if let Some(root) = model_tag
        && links.root() != Some(root)
    {
        return Err(core_error(dynibo::Error::InvalidLinkId));
    }
    let resolved = links
        .resolve(index)
        .ok_or_else(|| PyValueError::new_err(format!("invalid link id {index}")))?;
    if model_tag.is_none() {
        PyErr::warn(
            py,
            &py.get_type::<PyDeprecationWarning>(),
            PLAIN_INT_WARNING,
            1,
        )?;
    }
    Ok(resolved)
}

#[pyclass(name = "Load", module = "dynibo", frozen, eq, skip_from_py_object)]
#[derive(Clone, Debug, PartialEq)]
pub(super) struct PyLoad {
    link_id: usize,
    model_tag: Option<LinkId>,
    torque: [f64; 3],
    force: [f64; 3],
}

#[pymethods]
impl PyLoad {
    #[new]
    #[pyo3(signature = (link_id, torque=(0.0, 0.0, 0.0).into(), force=(0.0, 0.0, 0.0).into()))]
    fn new(link_id: &Bound<'_, PyAny>, torque: [f64; 3], force: [f64; 3]) -> PyResult<Self> {
        let (index, model_tag) = extract_link_parts(link_id)?;
        Ok(Self {
            link_id: index,
            model_tag,
            torque,
            force,
        })
    }

    #[getter]
    fn link_id(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        if let Some(root) = self.model_tag {
            make_py_link_id_with_root(py, root, self.link_id)
        } else {
            Ok(self.link_id.into_pyobject(py)?.into_any().unbind())
        }
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

/// Reusable native loads, created by a robot so link IDs have a model scope.
#[pyclass(name = "LoadBuffer", module = "dynibo")]
pub(super) struct PyLoadBuffer {
    pub(super) inner: CoreLoadBuffer,
    pub(super) links: LinkTable,
}

#[pymethods]
impl PyLoadBuffer {
    #[pyo3(signature = (link_id, torque=(0.0, 0.0, 0.0).into(), force=(0.0, 0.0, 0.0).into()))]
    fn set(
        &mut self,
        py: Python<'_>,
        link_id: &Bound<'_, PyAny>,
        torque: [f64; 3],
        force: [f64; 3],
    ) -> PyResult<()> {
        self.inner
            .set(
                target_link(py, &self.links, link_id)?,
                Wrench::new(Vector3::from(torque), Vector3::from(force)),
            )
            .map_err(core_error)
    }

    #[pyo3(signature = (link_id, torque=(0.0, 0.0, 0.0).into(), force=(0.0, 0.0, 0.0).into()))]
    fn add(
        &mut self,
        py: Python<'_>,
        link_id: &Bound<'_, PyAny>,
        torque: [f64; 3],
        force: [f64; 3],
    ) -> PyResult<()> {
        self.inner
            .add(
                target_link(py, &self.links, link_id)?,
                Wrench::new(Vector3::from(torque), Vector3::from(force)),
            )
            .map_err(core_error)
    }

    fn remove(&mut self, py: Python<'_>, link_id: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner
            .remove(target_link(py, &self.links, link_id)?)
            .map_err(core_error)
    }

    fn clear(&mut self) {
        self.inner.clear();
    }
    fn __len__(&self) -> usize {
        self.inner.len()
    }
}

pub(super) fn convert_loads(
    py: Python<'_>,
    links: &LinkTable,
    loads: Option<&Bound<'_, PyAny>>,
) -> PyResult<Vec<IndexedLoad>> {
    let Some(loads) = loads else {
        return Ok(Vec::new());
    };
    if let Ok(buffer) = loads.cast::<PyLoadBuffer>() {
        let buffer = buffer.try_borrow()?;
        if !buffer.links.same_model(links) {
            return Err(core_error(dynibo::Error::InvalidLinkId));
        }
        return Ok(buffer.inner.as_slice().to_vec());
    }
    let values = loads.extract::<Vec<PyRef<'_, PyLoad>>>()?;
    values
        .iter()
        .map(|load| {
            let link = resolve_tagged_index(py, links, load.link_id, load.model_tag)?;
            let wrench = Wrench::new(Vector3::from(load.torque), Vector3::from(load.force));
            if !wrench.is_finite() {
                return Err(core_error(dynibo::Error::NonFiniteInput { input: "load" }));
            }
            Ok(IndexedLoad { link, wrench })
        })
        .collect()
}
