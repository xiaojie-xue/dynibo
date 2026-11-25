use nalgebra::{Matrix3, Vector3};

use super::{Frame, Twist, Wrench};

#[inline]
pub(crate) fn cross_matrix(value: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -value.z, value.y, value.z, 0.0, -value.x, -value.y, value.x, 0.0,
    )
}

#[inline]
pub(crate) fn motion_to_child(parent_from_child: &Frame, value: Twist) -> Twist {
    let rotation_inverse = parent_from_child.rotation.inverse();
    Twist::new(
        rotation_inverse * value.angular,
        rotation_inverse
            * (value.linear + value.angular.cross(&parent_from_child.translation.vector)),
    )
}

#[inline]
pub(crate) fn force_to_parent(parent_from_child: &Frame, wrench: Wrench) -> Wrench {
    let force = parent_from_child.rotation * wrench.force;
    Wrench::new(
        parent_from_child.rotation * wrench.torque
            + parent_from_child.translation.vector.cross(&force),
        force,
    )
}
