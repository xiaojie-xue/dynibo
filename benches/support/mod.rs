use std::{hint::black_box, path::PathBuf};

use criterion::BenchmarkGroup;
use criterion::measurement::WallTime;
use dynibo::{BaseState, FloatingRobot, Frame, LinkId, Robot, Twist};
use nalgebra::Vector3;

#[cfg(feature = "pinocchio-bench")]
pub mod pinocchio;

#[derive(Clone, Copy)]
pub struct Model {
    pub name: &'static str,
    pub path: &'static str,
    pub target: &'static str,
    pub floating: bool,
    pub joints: usize,
}

pub const MODELS: [Model; 2] = [
    Model {
        name: "franka_fixed",
        path: "examples/data/franka/franka_fer.urdf",
        target: "fer_link8",
        floating: false,
        joints: 7,
    },
    Model {
        name: "g1_floating",
        path: "examples/data/unitree-g1/g1_29dof_mode_11.urdf",
        target: "left_rubber_hand",
        floating: true,
        joints: 29,
    },
];
pub const OPERATIONS: [&str; 3] = ["jacobian", "rnea", "aba"];

pub enum BenchmarkRobot {
    Fixed(Robot),
    Floating(FloatingRobot),
}

pub struct Case {
    pub model: Model,
    pub robot: BenchmarkRobot,
    pub base: BaseState,
    pub target: LinkId,
    pub names: Vec<String>,
    pub q: Vec<f64>,
    pub qd: Vec<f64>,
    pub qdd: Vec<f64>,
    pub forces: Vec<f64>,
}

impl Case {
    pub fn new(model: Model) -> Self {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(model.path);
        let robot = if model.floating {
            BenchmarkRobot::Floating(FloatingRobot::from_urdf(&path).unwrap())
        } else {
            BenchmarkRobot::Fixed(Robot::from_urdf(&path).unwrap())
        };
        let (n, g, target) = match &robot {
            BenchmarkRobot::Fixed(r) => (
                r.joint_count(),
                r.generalized_count(),
                r.link_id(model.target).unwrap(),
            ),
            BenchmarkRobot::Floating(r) => (
                r.joint_count(),
                r.generalized_count(),
                r.link_id(model.target).unwrap(),
            ),
        };
        assert_eq!(n, model.joints);
        assert_eq!(g, n + if model.floating { 6 } else { 0 });
        let mut names = Vec::new();
        let mut q = Vec::new();
        for i in 0..n {
            let (name, lower, upper) = match &robot {
                BenchmarkRobot::Fixed(r) => (
                    r.joint_name(i).unwrap(),
                    r.joint_lower_limit(i).unwrap(),
                    r.joint_upper_limit(i).unwrap(),
                ),
                BenchmarkRobot::Floating(r) => (
                    r.joint_name(i).unwrap(),
                    r.joint_lower_limit(i).unwrap(),
                    r.joint_upper_limit(i).unwrap(),
                ),
            };
            names.push(name.to_owned());
            // Deterministic, strictly within each joint's position limits.
            q.push((lower + upper) * 0.5 + (upper - lower) * 0.1 * (0.37 * (i + 1) as f64).sin());
        }
        let base = BaseState::new(
            Frame::translation(0.2, -0.1, 0.8),
            Twist::new(Vector3::new(0.1, -0.05, 0.08), Vector3::new(0.2, 0.0, -0.1)),
            Twist::new(
                Vector3::new(0.02, 0.03, -0.01),
                Vector3::new(0.1, -0.2, 0.05),
            ),
        )
        .unwrap();
        let mut case = Self {
            model,
            robot,
            base,
            target,
            names,
            q,
            qd: (0..n)
                .map(|i| (0.23 * (i + 1) as f64).cos() * 0.4)
                .collect(),
            qdd: (0..n)
                .map(|i| (0.41 * (i + 1) as f64).sin() * 0.3)
                .collect(),
            forces: vec![0.0; g],
        };
        let mut forces = vec![0.0; g];
        case.calculate("rnea", &mut forces);
        case.forces = forces;
        let mut acceleration = vec![0.0; g];
        case.calculate("aba", &mut acceleration);
        let mut expected = Vec::new();
        if model.floating {
            expected.extend_from_slice(case.base.acceleration().to_vector().as_slice());
        }
        expected.extend_from_slice(&case.qdd);
        assert_close(&acceleration, &expected, "RNEA/ABA round trip");
        // Benchmark ABA with an unactuated base. No contact constraints are solved.
        if model.floating {
            case.forces[..6].fill(0.0);
        }
        case
    }

    pub fn g(&self) -> usize {
        self.forces.len()
    }

    pub fn calculate(&mut self, operation: &str, output: &mut [f64]) {
        let q = black_box(self.q.as_slice());
        let qd = black_box(self.qd.as_slice());
        let qdd = black_box(self.qdd.as_slice());
        let forces = black_box(self.forces.as_slice());
        let base = black_box(&self.base);
        let output = black_box(output);
        match (&mut self.robot, operation) {
            (BenchmarkRobot::Fixed(r), "jacobian") => r.jacobian(q, self.target, output),
            (BenchmarkRobot::Floating(r), "jacobian") => r.jacobian(base, q, self.target, output),
            (BenchmarkRobot::Fixed(r), "rnea") => r.inverse_dynamics(q, qd, qdd, &[], output),
            (BenchmarkRobot::Floating(r), "rnea") => {
                r.inverse_dynamics(base, q, qd, qdd, &[], output)
            }
            (BenchmarkRobot::Fixed(r), "aba") => r.forward_dynamics(q, qd, forces, &[], output),
            (BenchmarkRobot::Floating(r), "aba") => {
                r.forward_dynamics(base, q, qd, forces, &[], output)
            }
            _ => unreachable!(),
        }
        .unwrap();
    }
}

pub fn assert_close(actual: &[f64], expected: &[f64], label: &str) -> f64 {
    assert_eq!(actual.len(), expected.len());
    let mut max_error = 0.0_f64;
    for (a, b) in actual.iter().zip(expected) {
        let error = (a - b).abs();
        assert!(
            a.is_finite() && b.is_finite() && error <= 1e-8 + 1e-8 * b.abs(),
            "{label}: actual {a}, expected {b}, error {error}"
        );
        max_error = max_error.max(error);
    }
    max_error
}

pub fn bench_dynibo(group: &mut BenchmarkGroup<'_, WallTime>, case: &mut Case, operation: &str) {
    let mut output = vec![0.0; case.g() * if operation == "jacobian" { 6 } else { 1 }];
    group.bench_function("dynibo", |b| {
        b.iter(|| {
            case.calculate(operation, &mut output);
            black_box(&output);
        })
    });
}
