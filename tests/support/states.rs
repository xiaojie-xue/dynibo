use dynibo::{BaseState, Frame, Twist};
use nalgebra::{Translation3, UnitQuaternion, Vector3};

#[derive(Clone, Debug)]
pub struct JointState {
    pub q: Vec<f64>,
    pub qd: Vec<f64>,
    pub qdd: Vec<f64>,
    pub tau: Vec<f64>,
}

pub fn deterministic_joint_state(joint_count: usize, sample: usize) -> JointState {
    let signal = |phase: f64, amplitude: f64| {
        (0..joint_count)
            .map(|joint| {
                let argument = (sample + 1) as f64 * (joint + 2) as f64 * 0.371 + phase;
                amplitude * (1.0 + 0.04 * joint as f64) * argument.sin()
            })
            .collect()
    };
    JointState {
        q: signal(0.11, 0.65),
        qd: signal(0.73, 0.8),
        qdd: signal(1.19, 0.9),
        tau: signal(1.61, 8.0),
    }
}

pub fn deterministic_base_state(sample: usize) -> BaseState {
    let phase = sample as f64 + 1.0;
    BaseState::new(
        Frame::from_parts(
            Translation3::new(0.2, -0.3, 0.4),
            UnitQuaternion::from_euler_angles(
                0.25 * (phase * 0.23).sin(),
                -0.2 * (phase * 0.31).cos(),
                0.18 * (phase * 0.17).sin(),
            ),
        ),
        Twist::new(
            Vector3::new(0.21, -0.17, 0.13),
            Vector3::new(-0.3, 0.2, 0.1),
        ),
        Twist::new(
            Vector3::new(-0.11, 0.14, 0.09),
            Vector3::new(0.35, -0.22, 0.18),
        ),
    )
    .expect("deterministic floating state must be finite")
}

pub fn base_with_acceleration(base: &BaseState, acceleration: Twist) -> BaseState {
    BaseState::new(*base.frame(), base.velocity(), acceleration)
        .expect("derived base state must be finite")
}

pub fn generalized_acceleration(base: Twist, joints: &[f64]) -> Vec<f64> {
    base.angular
        .iter()
        .chain(base.linear.iter())
        .copied()
        .chain(joints.iter().copied())
        .collect()
}
