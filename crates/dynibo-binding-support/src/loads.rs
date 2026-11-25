use dynibo::{IndexedLoad, LoadBuffer, Wrench};
use nalgebra::Vector3;

use crate::LinkTable;

/// Error returned while validating or aggregating indexed external loads.
#[derive(Debug)]
pub enum LoadInputError {
    InvalidLinkId(usize),
    Core(dynibo::Error),
}

/// Validates and aggregates raw `(link_index, torque, force)` items into a
/// reusable [`LoadBuffer`] without allocating on the hot path.
///
/// Always leaves `buffer` clean on failure so subsequent calls cannot observe
/// partial entries.
pub fn aggregate_loads<'a, T>(
    buffer: &'a mut LoadBuffer,
    links: &LinkTable,
    items: &[T],
    extract: impl Fn(&T) -> (usize, [f64; 3], [f64; 3]),
) -> Result<&'a [IndexedLoad], LoadInputError> {
    buffer.clear();
    if items.is_empty() {
        return Ok(buffer.as_slice());
    }
    // Validate every link index and wrench before accumulating so malformed
    // inputs are always reported as invalid arguments rather than overflow.
    for item in items {
        let (link_index, torque, force) = extract(item);
        if links.resolve(link_index).is_none() {
            return Err(LoadInputError::InvalidLinkId(link_index));
        }
        let wrench = Wrench::new(Vector3::from(torque), Vector3::from(force));
        if !wrench.is_finite() {
            return Err(LoadInputError::Core(dynibo::Error::NonFiniteInput {
                input: "load",
            }));
        }
    }
    for item in items {
        let (link_index, torque, force) = extract(item);
        let link = links
            .resolve(link_index)
            .expect("link index validated above");
        let wrench = Wrench::new(Vector3::from(torque), Vector3::from(force));
        if let Err(error) = buffer.add(link, wrench) {
            buffer.clear();
            return Err(LoadInputError::Core(error));
        }
    }
    Ok(buffer.as_slice())
}
