use std::ops::{Add, AddAssign};

use nalgebra::{Matrix3, SMatrix, Vector3};

use crate::JointType;

use super::{Frame, Twist, Wrench, transform::cross_matrix};

pub(crate) type Matrix6 = SMatrix<f64, 6, 6>;

/// Compact 10-parameter rigid-body spatial inertia expressed about a link origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SpatialInertia {
    pub(crate) mass: f64,
    pub(crate) first_moment: Vector3<f64>,
    pub(crate) origin_inertia: Matrix3<f64>,
}

impl SpatialInertia {
    pub(crate) fn zeros() -> Self {
        Self {
            mass: 0.0,
            first_moment: Vector3::zeros(),
            origin_inertia: Matrix3::zeros(),
        }
    }

    pub(crate) const fn new(
        mass: f64,
        first_moment: Vector3<f64>,
        origin_inertia: Matrix3<f64>,
    ) -> Self {
        Self {
            mass,
            first_moment,
            origin_inertia,
        }
    }

    #[inline]
    pub(crate) fn apply(&self, motion: Twist) -> Wrench {
        Wrench::new(
            self.origin_inertia * motion.angular + self.first_moment.cross(&motion.linear),
            motion.angular.cross(&self.first_moment) + self.mass * motion.linear,
        )
    }

    #[inline]
    pub(crate) fn apply_joint(&self, joint_type: JointType, axis: Vector3<f64>) -> Wrench {
        match joint_type {
            JointType::Revolute => {
                Wrench::new(self.origin_inertia * axis, axis.cross(&self.first_moment))
            }
            JointType::Prismatic => Wrench::new(self.first_moment.cross(&axis), self.mass * axis),
            JointType::Fixed => Wrench::zeros(),
        }
    }

    #[inline]
    pub(crate) fn transform_to_parent(&self, parent_from_child: &Frame) -> Self {
        let translation = parent_from_child.translation.vector;
        let rotation = parent_from_child.rotation.to_rotation_matrix();
        let rotated_moment = rotation * self.first_moment;
        let rotated_inertia = rotation * self.origin_inertia * rotation.transpose();
        // Weight translation by sqrt(mass) before squaring so zero or tiny masses
        // translated by large distances do not produce NaN or spurious infinity.
        let weighted_translation = self.mass.sqrt() * translation;
        let (x, y, z) = (
            weighted_translation.x,
            weighted_translation.y,
            weighted_translation.z,
        );
        let parallel_axis = Matrix3::new(
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
        // R I_o R^T + m([t]x^T [t]x) + 2 (t . h) I - t h^T - h t^T with h = R h_child.
        let origin_inertia = rotated_inertia
            + parallel_axis
            + (2.0 * translation.dot(&rotated_moment)) * Matrix3::identity()
            - translation * rotated_moment.transpose()
            - rotated_moment * translation.transpose();
        Self {
            mass: self.mass,
            first_moment: self.mass * translation + rotated_moment,
            origin_inertia,
        }
    }

    #[inline]
    pub(crate) fn to_matrix6(self) -> Matrix6 {
        let moment_cross = cross_matrix(self.first_moment);
        let mut output = Matrix6::zeros();
        output
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&self.origin_inertia);
        output.fixed_view_mut::<3, 3>(0, 3).copy_from(&moment_cross);
        output
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-moment_cross));
        output
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(self.mass * Matrix3::identity()));
        output
    }
}

impl Add for SpatialInertia {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            mass: self.mass + rhs.mass,
            first_moment: self.first_moment + rhs.first_moment,
            origin_inertia: self.origin_inertia + rhs.origin_inertia,
        }
    }
}

impl AddAssign for SpatialInertia {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.mass += rhs.mass;
        self.first_moment += rhs.first_moment;
        self.origin_inertia += rhs.origin_inertia;
    }
}

/// Dense 6x6 symmetric articulated-body spatial inertia used by ABA.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ArticulatedInertia(pub(crate) Matrix6);

impl ArticulatedInertia {
    pub(crate) fn zeros() -> Self {
        Self(Matrix6::zeros())
    }

    #[inline]
    pub(crate) fn from_spatial(inertia: SpatialInertia) -> Self {
        Self(inertia.to_matrix6())
    }

    #[inline]
    pub(crate) fn apply(&self, motion: Twist) -> Wrench {
        Wrench::from_vector(self.0 * motion.to_vector())
    }

    #[inline]
    pub(crate) fn apply_joint(&self, joint_type: JointType, axis: Vector3<f64>) -> Wrench {
        let offset = match joint_type {
            JointType::Revolute => 0,
            JointType::Prismatic => 3,
            JointType::Fixed => return Wrench::zeros(),
        };
        // Match only exact cardinal axes, including their negative directions.
        // Nearly aligned and arbitrary axes must retain all their components.
        let aligned = if axis.x.abs() == 1.0 && axis.y == 0.0 && axis.z == 0.0 {
            Some((0, axis.x))
        } else if axis.y.abs() == 1.0 && axis.x == 0.0 && axis.z == 0.0 {
            Some((1, axis.y))
        } else if axis.z.abs() == 1.0 && axis.x == 0.0 && axis.y == 0.0 {
            Some((2, axis.z))
        } else {
            None
        };
        let product = if let Some((index, sign)) = aligned {
            self.0.column(offset + index) * sign
        } else {
            self.0.fixed_columns::<3>(offset) * axis
        };
        Wrench::from_vector(product)
    }

    #[inline]
    pub(crate) fn transform_to_parent(&self, parent_from_child: &Frame) -> Self {
        // Angular-first symmetric inertia is [A B; B^T D]. Rotate the three
        // independent blocks, then translate them with P = [translation]x.
        // This is X^T I X without constructing a dense spatial transform X.
        let rotation = parent_from_child.rotation.to_rotation_matrix().into_inner();
        let angular = rotation * self.0.fixed_view::<3, 3>(0, 0) * rotation.transpose();
        let coupling = rotation * self.0.fixed_view::<3, 3>(0, 3) * rotation.transpose();
        let linear = rotation * self.0.fixed_view::<3, 3>(3, 3) * rotation.transpose();
        let position_cross = cross_matrix(parent_from_child.translation.vector);
        let translated_coupling = coupling + position_cross * linear;
        let translated_angular =
            angular + position_cross * coupling.transpose() - translated_coupling * position_cross;
        let mut output = Matrix6::zeros();
        output
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&translated_angular);
        output
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&translated_coupling);
        output
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&translated_coupling.transpose());
        output.fixed_view_mut::<3, 3>(3, 3).copy_from(&linear);
        Self(output)
    }
}

impl AddAssign for ArticulatedInertia {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

#[cfg(test)]
#[cfg_attr(coverage_nightly, coverage(off))]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::{SVector, Translation3, UnitQuaternion};

    use super::*;
    use crate::spatial::transform::{force_to_parent, motion_to_child};

    type Vector6 = SVector<f64, 6>;

    fn dense_motion_transform(frame: &Frame) -> Matrix6 {
        Matrix6::from_fn(|row, column| {
            let mut basis = Vector6::zeros();
            basis[column] = 1.0;
            motion_to_child(frame, Twist::from_vector(basis)).to_vector()[row]
        })
    }

    #[test]
    fn spatial_inertia_transform_and_apply_match_dense_algebra() {
        let frame = Frame::from_parts(
            Translation3::new(0.25, -0.4, 0.15),
            UnitQuaternion::from_euler_angles(0.3, -0.2, 0.5),
        );
        let inertia = SpatialInertia::new(
            2.4,
            Vector3::new(0.3, -0.5, 0.2),
            Matrix3::new(1.2, 0.1, -0.05, 0.1, 1.5, 0.08, -0.05, 0.08, 1.8),
        );
        let dense_x = dense_motion_transform(&frame);
        let expected_parent = dense_x.transpose() * inertia.to_matrix6() * dense_x;
        let actual_parent = inertia.transform_to_parent(&frame).to_matrix6();
        assert_relative_eq!(
            actual_parent,
            expected_parent,
            epsilon = 1e-12,
            max_relative = 1e-12
        );

        let twist = Twist::new(Vector3::new(0.4, -0.2, 0.7), Vector3::new(-0.3, 0.5, 0.1));
        assert_relative_eq!(
            inertia.apply(twist).to_vector(),
            inertia.to_matrix6() * twist.to_vector(),
            epsilon = 1e-12
        );

        let wrench = Wrench::new(Vector3::new(1.1, -0.7, 0.3), Vector3::new(0.5, 2.2, -1.4));
        assert_relative_eq!(
            force_to_parent(&frame, wrench).to_vector(),
            dense_x.transpose() * wrench.to_vector(),
            epsilon = 1e-12
        );
    }

    #[test]
    fn block_inertia_transform_matches_dense_spatial_congruence() {
        for sample in 0..16 {
            let t = (sample + 1) as f64;
            let factor = Matrix6::from_fn(|row, column| {
                (t * 0.13 + (row * 6 + column) as f64 * 0.27).sin()
                    + if row == column { 2.0 } else { 0.0 }
            });
            let inertia = ArticulatedInertia(factor * factor.transpose());
            let frame = Frame::from_parts(
                Translation3::new(0.13 * t, -0.07 * t, 0.03 * t),
                UnitQuaternion::from_euler_angles(0.11 * t, -0.17 * t, 0.23 * t),
            );
            let dense_transform = dense_motion_transform(&frame);
            let expected = dense_transform.transpose() * inertia.0 * dense_transform;
            let actual = inertia.transform_to_parent(&frame).0;
            assert_relative_eq!(actual, expected, epsilon = 1e-11, max_relative = 1e-12);
        }
    }

    #[test]
    fn specialized_joint_product_preserves_signed_and_non_cardinal_axes() {
        let factor = Matrix6::from_fn(|row, column| {
            ((row * 6 + column + 1) as f64 * 0.19).cos() + if row == column { 3.0 } else { 0.0 }
        });
        let inertia = ArticulatedInertia(factor * factor.transpose());
        let axes = [
            Vector3::x(),
            -Vector3::x(),
            Vector3::y(),
            -Vector3::y(),
            Vector3::z(),
            -Vector3::z(),
            Vector3::new(1.0, 1e-9, 0.0).normalize(),
            Vector3::new(0.0, -1.0, 1e-9).normalize(),
            Vector3::new(1e-9, 0.0, 1.0).normalize(),
            Vector3::new(0.3, -0.4, 0.5).normalize(),
        ];
        for joint_type in [JointType::Revolute, JointType::Prismatic, JointType::Fixed] {
            for axis in axes {
                let motion = match joint_type {
                    JointType::Revolute => Twist::new(axis, Vector3::zeros()),
                    JointType::Prismatic => Twist::new(Vector3::zeros(), axis),
                    JointType::Fixed => Twist::zeros(),
                };
                let expected = inertia.0 * motion.to_vector();
                let actual = inertia.apply_joint(joint_type, axis).to_vector();
                assert_relative_eq!(actual, expected, epsilon = 1e-12, max_relative = 1e-12);
            }
        }
        let spatial = SpatialInertia::new(2.0, Vector3::new(0.1, -0.2, 0.3), Matrix3::identity());
        assert_eq!(
            spatial.apply_joint(JointType::Fixed, Vector3::x()),
            Wrench::zeros()
        );
        let doubled = spatial + spatial;
        assert_relative_eq!(
            doubled.to_matrix6(),
            spatial.to_matrix6() * 2.0,
            epsilon = 1e-12
        );
    }

    #[test]
    fn parallel_axis_translation_avoids_overflow_and_nan_for_small_or_zero_mass() {
        let large_shift = Frame::from_parts(
            Translation3::new(1e200, 0.0, 0.0),
            UnitQuaternion::identity(),
        );

        let massless = SpatialInertia::new(0.0, Vector3::zeros(), Matrix3::zeros());
        let shifted_massless = massless.transform_to_parent(&large_shift);
        assert!(
            shifted_massless
                .origin_inertia
                .iter()
                .all(|v| v.is_finite())
        );
        assert_eq!(shifted_massless.origin_inertia, Matrix3::zeros());

        let tiny = SpatialInertia::new(1e-300, Vector3::zeros(), Matrix3::zeros());
        let shifted_tiny = tiny.transform_to_parent(&large_shift);
        assert!(shifted_tiny.origin_inertia.iter().all(|v| v.is_finite()));
        assert_relative_eq!(shifted_tiny.origin_inertia[(0, 0)], 0.0, epsilon = 0.0);
        assert_relative_eq!(
            shifted_tiny.origin_inertia[(1, 1)],
            1e100,
            max_relative = 1e-14
        );
        assert_relative_eq!(
            shifted_tiny.origin_inertia[(2, 2)],
            1e100,
            max_relative = 1e-14
        );
    }
}
