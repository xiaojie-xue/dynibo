use std::{hint::black_box, path::PathBuf};

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use dyno::{Frame, IndexedLoad, InverseKinematicsOptions, LinkId, Robot, Twist, Wrench};
use nalgebra::Vector3;

struct BenchmarkCase {
    arm: Robot,
    target: LinkId,
    q: Vec<f64>,
    qd: Vec<f64>,
    qdd: Vec<f64>,
    base: Frame,
}

impl BenchmarkCase {
    fn new(relative_urdf_path: &str) -> Self {
        let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative_urdf_path);
        let arm = Robot::from_urdf(urdf_path).expect("Dyno must load the benchmark URDF");
        let target_name = arm
            .leaf_links()
            .next()
            .expect("benchmark robot must have a leaf link")
            .name()
            .to_owned();
        let target = arm.link_id(&target_name).unwrap();
        let n = arm.joint_count();
        Self {
            arm,
            target,
            q: (0..n)
                .map(|index| (0.37 * (index + 1) as f64).sin() * 0.5)
                .collect(),
            qd: (0..n)
                .map(|index| (0.23 * (index + 1) as f64).cos() * 0.4)
                .collect(),
            qdd: (0..n)
                .map(|index| (0.41 * (index + 1) as f64).sin() * 0.3)
                .collect(),
            base: Frame::identity(),
        }
    }
}

fn benchmark_case(c: &mut Criterion, case: &BenchmarkCase) {
    let n = case.arm.joint_count();
    let size = format!("{n}joint");

    let mut fk = c.benchmark_group(format!("forward_kinematics/{size}"));
    fk.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    fk.bench_function("dyno", |b| {
        b.iter(|| {
            black_box(
                case.arm
                    .forward_kinematics(black_box(&case.q), case.target, &mut workspace)
                    .unwrap(),
            )
        });
    });
    fk.finish();

    let mut jacobian = c.benchmark_group(format!("jacobian/{size}"));
    jacobian.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    let mut jacobian_output = vec![0.0; 6 * n];
    jacobian.bench_function("dyno", |b| {
        b.iter(|| {
            case.arm
                .jacobian(
                    black_box(&case.q),
                    case.target,
                    &mut workspace,
                    black_box(&mut jacobian_output),
                )
                .unwrap();
            black_box(&jacobian_output);
        });
    });
    jacobian.finish();

    let mut velocity = c.benchmark_group(format!("forward_velocity/{size}"));
    velocity.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    velocity.bench_function("dyno", |b| {
        b.iter(|| {
            black_box(
                case.arm
                    .forward_velocity_kinematics(
                        black_box(&case.q),
                        black_box(&case.qd),
                        case.target,
                        &case.base,
                        &Frame::identity(),
                        &mut workspace,
                    )
                    .unwrap(),
            )
        });
    });
    velocity.finish();

    let mut acceleration = c.benchmark_group(format!("forward_acceleration/{size}"));
    acceleration.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    acceleration.bench_function("dyno", |b| {
        b.iter(|| {
            black_box(
                case.arm
                    .forward_acceleration_kinematics(
                        black_box(&case.q),
                        black_box(&case.qd),
                        black_box(&case.qdd),
                        case.target,
                        &mut workspace,
                    )
                    .unwrap(),
            )
        });
    });
    acceleration.finish();

    let mut gravity = c.benchmark_group(format!("gravity/{size}"));
    gravity.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    let mut output = vec![0.0; n];
    gravity.bench_function("dyno", |b| {
        b.iter(|| {
            case.arm
                .gravity(
                    black_box(&case.q),
                    &case.base,
                    black_box(&[]),
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    gravity.finish();

    let mut rnea = c.benchmark_group(format!("rnea/{size}"));
    rnea.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    let mut output = vec![0.0; n];
    rnea.bench_function("dyno", |b| {
        b.iter(|| {
            case.arm
                .inverse_dynamics(
                    black_box(&case.q),
                    black_box(&case.qd),
                    black_box(&case.qdd),
                    &case.base,
                    Twist::zeros(),
                    Twist::zeros(),
                    black_box(&[]),
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    rnea.finish();
}

fn benchmark_tree_case(c: &mut Criterion) {
    let case = BenchmarkCase::new("benches/data/test_tree_7.urdf");
    benchmark_case(c, &case);
    let left = case.arm.link_id("left_tool").unwrap();
    let right = case.arm.link_id("right_tool").unwrap();
    let loads = [
        IndexedLoad {
            link: left,
            wrench: Wrench::new(Vector3::new(0.1, -0.2, 0.3), Vector3::new(1.0, 0.5, -0.25)),
        },
        IndexedLoad {
            link: right,
            wrench: Wrench::new(Vector3::new(-0.3, 0.2, 0.1), Vector3::new(-0.5, 0.75, 0.4)),
        },
    ];
    let mut output = vec![0.0; case.arm.joint_count()];

    let mut gravity = c.benchmark_group("tree_gravity/7joint_2leaf");
    gravity.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    gravity.bench_function("two_leaf_loads", |b| {
        b.iter(|| {
            case.arm
                .gravity(
                    black_box(&case.q),
                    &case.base,
                    black_box(&loads),
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    gravity.finish();

    let mut rnea = c.benchmark_group("tree_rnea/7joint_2leaf");
    rnea.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    rnea.bench_function("two_leaf_loads", |b| {
        b.iter(|| {
            case.arm
                .inverse_dynamics(
                    black_box(&case.q),
                    black_box(&case.qd),
                    black_box(&case.qdd),
                    &case.base,
                    Twist::zeros(),
                    Twist::zeros(),
                    black_box(&loads),
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    rnea.finish();
}

fn benchmark_target_depths(c: &mut Criterion) {
    let case = BenchmarkCase::new("benches/data/test_arm_40.urdf");
    let targets = [
        ("root", case.arm.link_id("test_base_link").unwrap()),
        ("depth_1", case.arm.link_id("test_link_1").unwrap()),
        ("depth_10", case.arm.link_id("test_link_10").unwrap()),
        ("depth_40", case.arm.link_id("test_link_40").unwrap()),
    ];

    let mut fk = c.benchmark_group("target_depth/40joint/forward_kinematics");
    for &(depth, target) in &targets {
        let mut workspace = case.arm.workspace();
        fk.bench_with_input(BenchmarkId::from_parameter(depth), &target, |b, &target| {
            b.iter(|| {
                black_box(
                    case.arm
                        .forward_kinematics(black_box(&case.q), target, &mut workspace)
                        .unwrap(),
                )
            });
        });
    }
    fk.finish();

    let mut jacobian = c.benchmark_group("target_depth/40joint/jacobian");
    for &(depth, target) in &targets {
        let mut workspace = case.arm.workspace();
        let mut output = vec![0.0; 6 * case.arm.joint_count()];
        jacobian.bench_with_input(BenchmarkId::from_parameter(depth), &target, |b, &target| {
            b.iter(|| {
                case.arm
                    .jacobian(
                        black_box(&case.q),
                        target,
                        &mut workspace,
                        black_box(&mut output),
                    )
                    .unwrap();
                black_box(&output);
            });
        });
    }
    jacobian.finish();

    let mut velocity = c.benchmark_group("target_depth/40joint/forward_velocity");
    for &(depth, target) in &targets {
        let mut workspace = case.arm.workspace();
        velocity.bench_with_input(BenchmarkId::from_parameter(depth), &target, |b, &target| {
            b.iter(|| {
                black_box(
                    case.arm
                        .forward_velocity_kinematics(
                            black_box(&case.q),
                            black_box(&case.qd),
                            target,
                            &case.base,
                            &Frame::identity(),
                            &mut workspace,
                        )
                        .unwrap(),
                )
            });
        });
    }
    velocity.finish();

    let mut acceleration = c.benchmark_group("target_depth/40joint/forward_acceleration");
    for &(depth, target) in &targets {
        let mut workspace = case.arm.workspace();
        acceleration.bench_with_input(BenchmarkId::from_parameter(depth), &target, |b, &target| {
            b.iter(|| {
                black_box(
                    case.arm
                        .forward_acceleration_kinematics(
                            black_box(&case.q),
                            black_box(&case.qd),
                            black_box(&case.qdd),
                            target,
                            &mut workspace,
                        )
                        .unwrap(),
                )
            });
        });
    }
    acceleration.finish();

    let mut inverse_kinematics = c.benchmark_group("target_depth/40joint/inverse_kinematics");
    for &(depth, target) in &targets[1..] {
        let mut setup_workspace = case.arm.workspace();
        let desired = case
            .arm
            .forward_kinematics(&case.q, target, &mut setup_workspace)
            .unwrap();
        let initial = vec![0.0; case.arm.joint_count()];
        let mut output = vec![0.0; case.arm.joint_count()];
        let mut workspace = case.arm.workspace();
        inverse_kinematics.bench_with_input(
            BenchmarkId::from_parameter(depth),
            &target,
            |b, &target| {
                b.iter(|| {
                    case.arm
                        .inverse_kinematics(
                            black_box(&initial),
                            target,
                            &desired,
                            InverseKinematicsOptions::default(),
                            &mut workspace,
                            black_box(&mut output),
                        )
                        .unwrap();
                    black_box(&output);
                });
            },
        );
    }
    inverse_kinematics.finish();
}

fn benchmark_inverse_kinematics(c: &mut Criterion, case: &BenchmarkCase) {
    let mut setup_workspace = case.arm.workspace();
    let desired = case
        .arm
        .forward_kinematics(&case.q, case.target, &mut setup_workspace)
        .unwrap();
    let initial = vec![0.0; case.arm.joint_count()];
    let mut output = vec![0.0; case.arm.joint_count()];
    let mut workspace = case.arm.workspace();
    let mut ik = c.benchmark_group("inverse_kinematics/4joint");
    ik.throughput(Throughput::Elements(1));
    ik.bench_function("dyno", |b| {
        b.iter(|| {
            case.arm
                .inverse_kinematics(
                    black_box(&initial),
                    case.target,
                    &desired,
                    InverseKinematicsOptions::default(),
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    ik.finish();
}

fn benchmark_core(c: &mut Criterion) {
    let case_4 = BenchmarkCase::new("tests/data/test_arm.urdf");
    benchmark_case(c, &case_4);
    benchmark_inverse_kinematics(c, &case_4);
    benchmark_case(c, &BenchmarkCase::new("benches/data/test_arm_40.urdf"));
    benchmark_tree_case(c);
    benchmark_target_depths(c);
}

criterion_group!(benches, benchmark_core);
criterion_main!(benches);
