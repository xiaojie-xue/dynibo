use dynibo::{ErrorCategory, Frame, Twist};

use super::{
    context::TestContext,
    numeric::{KINEMATICS, Tolerance, assert_slice_close},
};

#[derive(Debug)]
pub enum Observation {
    Frame(Frame),
    Twist(Twist),
    Vector(Vec<f64>),
    Matrix {
        rows: usize,
        columns: usize,
        values: Vec<f64>,
    },
}

#[derive(Debug, PartialEq, Eq)]
pub struct ObservedError {
    pub category: ErrorCategory,
    pub message: String,
}

pub type ObservedResult = Result<Observation, ObservedError>;

#[track_caller]
pub fn assert_observation_finite(result: &ObservedResult, context: &TestContext) {
    let values: Vec<f64> = match result {
        Ok(Observation::Frame(frame)) => frame.to_homogeneous().as_slice().to_vec(),
        Ok(Observation::Twist(twist)) => twist.to_vector().as_slice().to_vec(),
        Ok(Observation::Vector(values)) | Ok(Observation::Matrix { values, .. }) => values.clone(),
        Err(error) => panic!("algorithm failed: {context} error={error:?}"),
    };
    if let Some((index, value)) = values
        .iter()
        .copied()
        .enumerate()
        .find(|(_, value)| !value.is_finite())
    {
        panic!("non-finite observation: {context} index={index} value={value}");
    }
}

#[track_caller]
pub fn assert_observation_close(
    actual: &ObservedResult,
    expected: &ObservedResult,
    tolerance: Tolerance,
    context: &TestContext,
) {
    match (actual, expected) {
        (Ok(Observation::Frame(actual)), Ok(Observation::Frame(expected))) => {
            assert_slice_close(
                actual.to_homogeneous().as_slice(),
                expected.to_homogeneous().as_slice(),
                KINEMATICS,
                context,
            );
        }
        (Ok(Observation::Twist(actual)), Ok(Observation::Twist(expected))) => {
            assert_slice_close(
                actual.to_vector().as_slice(),
                expected.to_vector().as_slice(),
                tolerance,
                context,
            );
        }
        (Ok(Observation::Vector(actual)), Ok(Observation::Vector(expected))) => {
            assert_slice_close(actual, expected, tolerance, context);
        }
        (
            Ok(Observation::Matrix {
                rows: actual_rows,
                columns: actual_columns,
                values: actual,
            }),
            Ok(Observation::Matrix {
                rows: expected_rows,
                columns: expected_columns,
                values: expected,
            }),
        ) => {
            assert_eq!(
                (actual_rows, actual_columns),
                (expected_rows, expected_columns),
                "matrix shape mismatch: {context}"
            );
            assert_slice_close(actual, expected, tolerance, context);
        }
        (Err(actual), Err(expected)) => {
            assert_eq!(
                actual.category, expected.category,
                "error mismatch: {context}"
            );
            assert_eq!(
                actual.message, expected.message,
                "error mismatch: {context}"
            );
        }
        _ => panic!("observation kind mismatch: {context} actual={actual:?} expected={expected:?}"),
    }
}
