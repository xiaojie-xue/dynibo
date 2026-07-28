use std::fmt;

/// Errors returned while constructing or evaluating a robot model.
#[derive(Debug)]
pub enum Error {
    Urdf(urdf_rs::UrdfError),
    InvalidModel(String),
    UnsupportedJoint(String),
    WrongJointCount { expected: usize, actual: usize },
    InvalidJointAxis { joint: String },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Urdf(error) => write!(f, "failed to parse URDF: {error}"),
            Self::InvalidModel(message) => write!(f, "invalid robot model: {message}"),
            Self::UnsupportedJoint(joint) => write!(f, "unsupported joint type for {joint}"),
            Self::WrongJointCount { expected, actual } => {
                write!(f, "expected {expected} movable joints, found {actual}")
            }
            Self::InvalidJointAxis { joint } => write!(f, "joint {joint} has an invalid axis"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Urdf(error) => Some(error),
            _ => None,
        }
    }
}

impl From<urdf_rs::UrdfError> for Error {
    fn from(value: urdf_rs::UrdfError) -> Self {
        Self::Urdf(value)
    }
}

pub type Result<T> = std::result::Result<T, Error>;
