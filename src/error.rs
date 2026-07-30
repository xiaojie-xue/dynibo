use std::fmt;

/// Errors returned while constructing or evaluating a robot model.
#[derive(Debug)]
pub enum Error {
    /// The URDF file could not be read or parsed.
    Urdf(urdf_rs::UrdfError),
    /// The URDF describes an invalid or unsupported kinematic graph.
    InvalidModel(String),
    /// A joint uses a motion type not supported by this crate.
    UnsupportedJoint(String),
    /// The compile-time vector dimension does not match the loaded model.
    WrongJointCount {
        /// Number of joints in the loaded model.
        expected: usize,
        /// Compile-time dimension supplied by the caller.
        actual: usize,
    },
    /// A joint axis is too small to normalize.
    InvalidJointAxis {
        /// Name of the joint with the invalid axis.
        joint: String,
    },
    /// No link with the requested name exists in the model.
    UnknownLink {
        /// Link name requested by the caller.
        name: String,
    },
    /// A link belongs to a different model or is not model-owned.
    InvalidLink {
        /// Name of the rejected link.
        name: String,
    },
    /// One of the inverse-kinematics options is zero, negative, or non-finite.
    InvalidOptions(&'static str),
    /// An inverse-kinematics input contains a non-finite value.
    NonFiniteInput {
        /// Name of the input containing a non-finite value.
        input: &'static str,
    },
    /// The inverse-kinematics damped linear system could not be solved.
    NumericalFailure {
        /// One-based iteration at which the numerical solve failed.
        iteration: usize,
    },
    /// A converged inverse-kinematics solution violates a joint limit.
    JointLimitViolation {
        /// Zero-based index of the joint.
        joint_index: usize,
        /// Name of the joint.
        joint: String,
        /// Position produced by the solver.
        position: f64,
        /// Minimum permitted position.
        lower: f64,
        /// Maximum permitted position.
        upper: f64,
    },
    /// Inverse kinematics did not reach the requested pose within its iteration budget.
    NotConverged {
        /// Number of joint updates attempted.
        iterations: usize,
        /// Final Euclidean translation error, in metres.
        translation_error: f64,
        /// Final rotation-vector norm, in radians.
        rotation_error: f64,
    },
}

impl fmt::Display for Error {
    /// Formats a human-readable description of the model or calculation error.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Urdf(error) => write!(f, "failed to parse URDF: {error}"),
            Self::InvalidModel(message) => write!(f, "invalid robot model: {message}"),
            Self::UnsupportedJoint(joint) => write!(f, "unsupported joint type for {joint}"),
            Self::WrongJointCount { expected, actual } => {
                write!(f, "expected {expected} joints, found {actual}")
            }
            Self::InvalidJointAxis { joint } => write!(f, "joint {joint} has an invalid axis"),
            Self::UnknownLink { name } => write!(f, "link {name} does not exist in the model"),
            Self::InvalidLink { name } => {
                write!(f, "link {name} does not belong to this robot model")
            }
            Self::InvalidOptions(message) => {
                write!(f, "invalid inverse-kinematics options: {message}")
            }
            Self::NonFiniteInput { input } => {
                write!(f, "inverse-kinematics {input} contains a non-finite value")
            }
            Self::NumericalFailure { iteration } => write!(
                f,
                "inverse-kinematics linear solve failed at iteration {iteration}"
            ),
            Self::JointLimitViolation {
                joint_index,
                joint,
                position,
                lower,
                upper,
            } => write!(
                f,
                "inverse-kinematics solution {position:.6e} for joint {joint_index} ({joint}) \
                 is outside URDF limits [{lower:.6e}, {upper:.6e}]"
            ),
            Self::NotConverged {
                iterations,
                translation_error,
                rotation_error,
            } => write!(
                f,
                "inverse kinematics did not converge after {iterations} iterations \
                 (translation error {translation_error:.6e}, rotation error {rotation_error:.6e})"
            ),
        }
    }
}

impl std::error::Error for Error {
    /// Returns the underlying URDF error, when present.
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Urdf(error) => Some(error),
            _ => None,
        }
    }
}

impl From<urdf_rs::UrdfError> for Error {
    /// Wraps a URDF parser error in the crate's general error type.
    fn from(value: urdf_rs::UrdfError) -> Self {
        Self::Urdf(value)
    }
}

/// Result type returned by robot-model construction and calculations.
pub type Result<T> = std::result::Result<T, Error>;
