//! Shared fixtures and adapters for this integration-test target.

#[path = "../support/pinocchio.rs"]
mod pinocchio;

use std::path::PathBuf;

use dynibo::{
    BaseState, FloatingRobot, Frame, IndexedLoad, InverseKinematicsOptions, Robot, Twist, Wrench,
};
use nalgebra::{Quaternion, Rotation3, Translation3, UnitQuaternion, Vector3};
use pinocchio::PinocchioContext;

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/oracle_mixed.urdf")
}

fn tree_fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf")
}

fn deterministic_state(sample: usize) -> ([f64; 4], [f64; 4], [f64; 4]) {
    let wave =
        |joint: usize, phase: f64| ((sample + 1) as f64 * (joint + 2) as f64 * 0.619 + phase).sin();
    (
        [
            1.4 * wave(0, 0.1),
            5.0 * wave(1, 0.2),
            0.3 * wave(2, 0.3),
            2.7 * wave(3, 0.4),
        ],
        std::array::from_fn(|joint| 0.8 * wave(joint, 0.9)),
        std::array::from_fn(|joint| 1.1 * wave(joint, 1.7)),
    )
}

fn deterministic_mixed_state(sample: usize) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let (q, qd, qdd) = deterministic_state(sample);
    (
        [q[0], q[2], q[3]],
        [qd[0], qd[2], qd[3]],
        [qdd[0], qdd[2], qdd[3]],
    )
}

fn deterministic_tree_state(sample: usize) -> ([f64; 7], [f64; 7], [f64; 7]) {
    let values = |phase: f64, amplitude: f64| {
        std::array::from_fn(|joint| {
            let argument = (sample + 1) as f64 * (joint + 3) as f64 * 0.731 + phase;
            amplitude * argument.sin()
        })
    };
    (values(0.0, 0.9), values(0.7, 0.8), values(1.3, 1.1))
}

fn serial_fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf")
}

fn assert_close(actual: &[f64], expected: &[f64], absolute: f64, relative: f64, context: &str) {
    assert_eq!(actual.len(), expected.len());
    for (index, (&actual, &expected)) in actual.iter().zip(expected).enumerate() {
        let tolerance = absolute + relative * actual.abs().max(expected.abs());
        assert!(
            (actual - expected).abs() <= tolerance,
            "{context}: element {index}: actual={actual:.16e}, expected={expected:.16e}, tolerance={tolerance:.3e}"
        );
    }
}

fn binding_reference(key: &str) -> Vec<f64> {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/pinocchio_reference_v1.tsv");
    let contents = std::fs::read_to_string(path).unwrap();
    contents
        .lines()
        .find_map(|line| {
            let mut fields = line.split('\t');
            (fields.next() == Some(key))
                .then(|| fields.map(|value| value.parse::<f64>().unwrap()).collect())
        })
        .unwrap_or_else(|| panic!("binding reference key {key} must exist"))
}

mod dynamics;
mod floating;
mod kinematics;
mod references;
