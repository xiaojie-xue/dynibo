use std::fmt;

/// Failures specific to the iterative inverse-kinematics solver.
#[derive(Clone, Debug, PartialEq)]
pub enum InverseKinematicsError {
    /// One of the solver options is zero, negative, or non-finite.
    InvalidOptions(&'static str),
    /// The initial joint vector or target frame contains a non-finite value.
    NonFiniteInput { input: &'static str },
    /// The damped linear system could not be solved at an iteration.
    NumericalFailure { iteration: usize },
    /// A converged solution violates a joint limit loaded from the URDF.
    JointLimitViolation {
        joint_index: usize,
        joint: String,
        position: f64,
        lower: f64,
        upper: f64,
    },
    /// The requested pose was not reached within the iteration budget.
    NotConverged {
        iterations: usize,
        translation_error: f64,
        rotation_error: f64,
    },
}

impl fmt::Display for InverseKinematicsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
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

impl std::error::Error for InverseKinematicsError {}

/// Errors returned while constructing or evaluating a robot model.
#[derive(Debug)]
pub enum Error {
    Urdf(urdf_rs::UrdfError),
    InvalidModel(String),
    UnsupportedJoint(String),
    WrongJointCount { expected: usize, actual: usize },
    InvalidJointAxis { joint: String },
    InvalidLinkId { index: usize, link_count: usize },
    InverseKinematics(InverseKinematicsError),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Urdf(error) => write!(f, "failed to parse URDF: {error}"),
            Self::InvalidModel(message) => write!(f, "invalid robot model: {message}"),
            Self::UnsupportedJoint(joint) => write!(f, "unsupported joint type for {joint}"),
            Self::WrongJointCount { expected, actual } => {
                write!(f, "expected {expected} joints, found {actual}")
            }
            Self::InvalidJointAxis { joint } => write!(f, "joint {joint} has an invalid axis"),
            Self::InvalidLinkId { index, link_count } => {
                write!(
                    f,
                    "link index {index} is outside the {link_count}-link model"
                )
            }
            Self::InverseKinematics(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Urdf(error) => Some(error),
            Self::InverseKinematics(error) => Some(error),
            _ => None,
        }
    }
}

impl From<InverseKinematicsError> for Error {
    fn from(value: InverseKinematicsError) -> Self {
        Self::InverseKinematics(value)
    }
}

impl From<urdf_rs::UrdfError> for Error {
    fn from(value: urdf_rs::UrdfError) -> Self {
        Self::Urdf(value)
    }
}

pub type Result<T> = std::result::Result<T, Error>;
