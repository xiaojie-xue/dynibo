use nalgebra::{Matrix3, Vector3};

use crate::{Error, Result};

/// Identity and immutable inertial properties of one URDF link.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Link {
    name: String,
    dynamics: LinkDynamics,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct LinkDynamics {
    pub(crate) mass: f64,
    pub(crate) center_of_mass: Vector3<f64>,
    pub(crate) inertia: Matrix3<f64>,
    pub(crate) first_moment: Vector3<f64>,
    pub(crate) origin_inertia: Matrix3<f64>,
}

impl Link {
    /// Creates a link from its immutable inertial properties.
    pub(crate) fn new(
        name: impl Into<String>,
        mass: f64,
        center_of_mass: Vector3<f64>,
        inertia: Matrix3<f64>,
    ) -> Result<Self> {
        let name = name.into();
        let first_moment = mass * center_of_mass;
        // Weight before squaring so small masses can offset large distances.
        let weighted_center = mass.sqrt() * center_of_mass;
        let (x, y, z) = (weighted_center.x, weighted_center.y, weighted_center.z);
        let origin_inertia = inertia
            + Matrix3::new(
                y * y + z * z,
                -x * y,
                -x * z,
                -x * y,
                x * x + z * z,
                -y * z,
                -x * z,
                -y * z,
                x * x + y * y,
            );
        if !inertia
            .iter()
            .chain(first_moment.iter())
            .chain(origin_inertia.iter())
            .all(|value| value.is_finite())
        {
            return Err(Error::InvalidModel(format!(
                "link {name} derived inertial properties must be finite"
            )));
        }
        Ok(Self {
            name,
            dynamics: LinkDynamics {
                mass,
                center_of_mass,
                inertia,
                first_moment,
                origin_inertia,
            },
        })
    }

    /// Returns the link name loaded from the URDF.
    pub(crate) fn name(&self) -> &str {
        &self.name
    }

    /// Returns the link mass in kilograms.
    pub(crate) const fn mass(&self) -> f64 {
        self.dynamics.mass
    }

    /// Returns the center of mass expressed in the link frame, in metres.
    pub(crate) const fn center_of_mass(&self) -> &Vector3<f64> {
        &self.dynamics.center_of_mass
    }

    /// Returns the rotational inertia about the center of mass in the link frame.
    pub(crate) const fn inertia(&self) -> &Matrix3<f64> {
        &self.dynamics.inertia
    }

    pub(crate) const fn dynamics(&self) -> LinkDynamics {
        self.dynamics
    }
}

#[cfg(test)]
#[cfg_attr(coverage_nightly, coverage(off))]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::{Matrix3, Vector3};

    use super::Link;

    #[test]
    fn link_precomputes_first_moment_and_origin_inertia() {
        let mass = 2.5;
        let center = Vector3::new(0.3, -0.2, 0.4);
        let inertia = Matrix3::new(1.2, 0.1, -0.2, 0.1, 1.5, 0.05, -0.2, 0.05, 1.8);
        let link = Link::new("body", mass, center, inertia).unwrap();
        let dynamics = link.dynamics();

        let expected_first_moment = mass * center;
        let expected_origin_inertia = inertia
            + mass * (center.norm_squared() * Matrix3::identity() - center * center.transpose());
        assert_relative_eq!(
            dynamics.first_moment,
            expected_first_moment,
            epsilon = 1.0e-14
        );
        assert_relative_eq!(
            dynamics.origin_inertia,
            expected_origin_inertia,
            epsilon = 1.0e-14
        );
    }
}
