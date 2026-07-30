use std::{hint::black_box, path::PathBuf};

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use dyno::{ExternalWrench, Frame, JointVector, LinkId, Motion, RobotArm, Wrench};
use nalgebra::Vector3;

struct BenchmarkCase<const N: usize> {
    arm: RobotArm,
    target: LinkId,
    q: JointVector<N>,
    qd: JointVector<N>,
    qdd: JointVector<N>,
    base: Frame,
}

struct TreeBenchmarkCase<const N: usize> {
    case: BenchmarkCase<N>,
    target: LinkId,
    external_wrenches: [ExternalWrench; 2],
}

impl<const N: usize> TreeBenchmarkCase<N> {
    fn new(relative_urdf_path: &str, target_name: &str, other_leaf_name: &str) -> Self {
        let case = BenchmarkCase::new(relative_urdf_path);
        let target = case
            .arm
            .link_id(target_name)
            .expect("target link must exist");
        let other_leaf = case
            .arm
            .link_id(other_leaf_name)
            .expect("other leaf link must exist");
        assert_eq!(case.arm.leaf_links().len(), 2);

        Self {
            case,
            target,
            external_wrenches: [
                ExternalWrench {
                    link: target,
                    wrench: Wrench::new(
                        Vector3::new(0.1, -0.2, 0.3),
                        Vector3::new(1.0, 0.5, -0.25),
                    ),
                },
                ExternalWrench {
                    link: other_leaf,
                    wrench: Wrench::new(
                        Vector3::new(-0.3, 0.2, 0.1),
                        Vector3::new(-0.5, 0.75, 0.4),
                    ),
                },
            ],
        }
    }
}

impl<const N: usize> BenchmarkCase<N> {
    fn new(relative_urdf_path: &str) -> Self {
        let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative_urdf_path);
        let arm = RobotArm::from_urdf(urdf_path).expect("Dyno must load the benchmark URDF");
        assert_eq!(arm.joint_count(), N);
        let target = arm.leaf_links()[0];

        Self {
            arm,
            target,
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
        b.iter(|| {
            black_box(
                case.arm
                    .forward_kinematics(black_box(q), case.target)
                    .unwrap(),
            )
        });
    });
    fk.finish();

    let mut jacobian = c.benchmark_group(format!("end_jacobian/{size}"));
    jacobian.throughput(Throughput::Elements(1));
    jacobian.bench_with_input(BenchmarkId::from_parameter("dyno"), &case.q, |b, q| {
        b.iter(|| black_box(case.arm.jacobian(black_box(q), case.target).unwrap()));
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
                        case.target,
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
                    .gravity(black_box(q), &case.base, black_box(&[]))
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
                        black_box(&[]),
                    )
                    .unwrap(),
            )
        });
    });
    rnea.finish();
}

fn benchmark_tree_case<const N: usize>(c: &mut Criterion, tree: &TreeBenchmarkCase<N>) {
    let case = &tree.case;
    let size = format!("{N}joint_2leaf");

    let mut fk = c.benchmark_group(format!("tree_forward_kinematics/{size}"));
    fk.throughput(Throughput::Elements(1));
    fk.bench_function("selected_leaf", |b| {
        b.iter(|| {
            black_box(
                case.arm
                    .forward_kinematics(black_box(&case.q), tree.target)
                    .unwrap(),
            )
        });
    });
    fk.finish();

    let mut jacobian = c.benchmark_group(format!("tree_jacobian/{size}"));
    jacobian.throughput(Throughput::Elements(1));
    jacobian.bench_function("selected_leaf", |b| {
        b.iter(|| black_box(case.arm.jacobian(black_box(&case.q), tree.target).unwrap()));
    });
    jacobian.finish();

    let mut acceleration = c.benchmark_group(format!("tree_forward_acceleration/{size}"));
    acceleration.throughput(Throughput::Elements(1));
    acceleration.bench_function("selected_leaf", |b| {
        b.iter(|| {
            black_box(
                case.arm
                    .forward_acceleration_kinematics(
                        black_box(&case.q),
                        black_box(&case.qd),
                        black_box(&case.qdd),
                        tree.target,
                    )
                    .unwrap(),
            )
        });
    });
    acceleration.finish();

    let mut gravity = c.benchmark_group(format!("tree_gravity/{size}"));
    gravity.throughput(Throughput::Elements(1));
    gravity.bench_function("two_leaf_loads", |b| {
        b.iter(|| {
            black_box(
                case.arm
                    .gravity(
                        black_box(&case.q),
                        &case.base,
                        black_box(&tree.external_wrenches),
                    )
                    .unwrap(),
            )
        });
    });
    gravity.finish();

    let mut rnea = c.benchmark_group(format!("tree_rnea/{size}"));
    rnea.throughput(Throughput::Elements(1));
    rnea.bench_function("two_leaf_loads", |b| {
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
                        black_box(&tree.external_wrenches),
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
    benchmark_tree_case(
        c,
        &TreeBenchmarkCase::<7>::new("benches/data/test_tree_7.urdf", "left_tool", "right_tool"),
    );
}

criterion_group!(benches, benchmark_core);
criterion_main!(benches);
