use dynibo::Frame;
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};

pub const INVALID_POSE_MESSAGE: &str = "pose contains non-finite values or a zero quaternion";

/// Validates and converts a translation + `(x, y, z, w)` quaternion into a [`Frame`].
pub fn frame_from_xyzw(
    translation: [f64; 3],
    rotation_xyzw: [f64; 4],
) -> Result<Frame, &'static str> {
    let [x, y, z, w] = rotation_xyzw;
    let norm_squared = x * x + y * y + z * z + w * w;
    if !translation.iter().all(|value| value.is_finite())
        || !norm_squared.is_finite()
        || norm_squared <= 1.0e-24
    {
        return Err(INVALID_POSE_MESSAGE);
    }
    Ok(Frame::from_parts(
        Translation3::from(Vector3::from(translation)),
        UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
    ))
}

/// Converts a [`Frame`] into a translation + `(x, y, z, w)` quaternion.
pub fn xyzw_from_frame(frame: &Frame) -> ([f64; 3], [f64; 4]) {
    let q = frame.rotation.quaternion();
    (frame.translation.vector.into(), [q.i, q.j, q.k, q.w])
}
