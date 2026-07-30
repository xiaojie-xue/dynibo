use std::{hint::black_box, path::PathBuf};

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use dyno::{Frame, JointVector, Motion, RobotArm, Wrench};

struct BenchmarkCase<const N: usize> {
    arm: RobotArm,
    q: JointVector<N>,
    qd: JointVector<N>,
    qdd: JointVector<N>,
    base: Frame,
}

impl<const N: usize> BenchmarkCase<N> {
    fn new(relative_urdf_path: &str) -> Self {
        let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative_urdf_path);
        let arm = RobotArm::from_urdf(urdf_path).expect("Dyno must load the benchmark URDF");
        assert_eq!(arm.joint_count(), N);

        Self {
            arm,
            q: JointVector::<N>::from_fn(|row, _| (0.37 * (row + 1) as f64).sin() * 0.5),
            qd: JointVector::<N>::from_fn(|row, _| (0.23 * (row + 1) as f64).cos() * 0.4),
            qdd: JointVector::<N>::from_fn(|row, _| (0.41 * (row + 1) as f64).sin() * 0.3),
            base: Frame::identity(),
        }
    }
}

fn benchmark_case<const N: usize>(c: &mut Criterion, case: &BenchmarkCase<N>) {
    let size = format!("{N}dof");

    let mut fk = c.benchmark_group(format!("forward_kinematics/{size}"));
    fk.throughput(Throughput::Elements(1));
    fk.bench_with_input(BenchmarkId::from_parameter("dyno"), &case.q, |b, q| {
        b.iter(|| black_box(case.arm.forward_kinematics(black_box(q)).unwrap()));
    });
    fk.finish();

    let mut jacobian = c.benchmark_group(format!("end_jacobian/{size}"));
    jacobian.throughput(Throughput::Elements(1));
    jacobian.bench_with_input(BenchmarkId::from_parameter("dyno"), &case.q, |b, q| {
        b.iter(|| black_box(case.arm.jacobian(black_box(q)).unwrap()));
    });
    jacobian.finish();

    let mut acceleration = c.benchmark_group(format!("forward_acceleration/{size}"));
    acceleration.throughput(Throughput::Elements(1));
    acceleration.bench_function("dyno", |b| {
        b.iter(|| {
            black_box(
                case.arm
                    .forward_acceleration_kinematics(
                        black_box(&case.q),
                        black_box(&case.qd),
                        black_box(&case.qdd),
                    )
                    .unwrap(),
            )
        });
    });
    acceleration.finish();

    let mut gravity = c.benchmark_group(format!("gravity/{size}"));
    gravity.throughput(Throughput::Elements(1));
    gravity.bench_with_input(BenchmarkId::from_parameter("dyno"), &case.q, |b, q| {
        b.iter(|| {
            black_box(
                case.arm
                    .gravity(black_box(q), &case.base, Wrench::zeros())
                    .unwrap(),
            )
        });
    });
    gravity.finish();

    let mut rnea = c.benchmark_group(format!("rnea/{size}"));
    rnea.throughput(Throughput::Elements(1));
    rnea.bench_function("dyno", |b| {
        b.iter(|| {
            black_box(
                case.arm
                    .inverse_dynamics(
                        black_box(&case.q),
                        black_box(&case.qd),
                        black_box(&case.qdd),
                        &case.base,
                        Motion::zeros(),
                        Motion::zeros(),
                        Wrench::zeros(),
                    )
                    .unwrap(),
            )
        });
    });
    rnea.finish();
}

fn benchmark_core(c: &mut Criterion) {
    benchmark_case(c, &BenchmarkCase::<4>::new("tests/data/test_arm.urdf"));
    benchmark_case(
        c,
        &BenchmarkCase::<40>::new("benches/data/test_arm_40.urdf"),
    );
}

criterion_group!(benches, benchmark_core);
criterion_main!(benches);
