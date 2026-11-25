// The shared module also contains the optional comparison-only Pinocchio helpers.
#[allow(dead_code)]
mod support;

use criterion::{Criterion, criterion_group, criterion_main};
use dynibo::Frame;
use std::hint::black_box;
use support::{BenchmarkRobot, Case, MODELS, Model, OPERATIONS, bench_dynibo};

fn benchmark_all_poses(c: &mut Criterion, case: &mut Case) {
    let links = match &case.robot {
        BenchmarkRobot::Fixed(r) => (0..r.link_count())
            .map(|i| r.link_id_at(i).unwrap())
            .collect::<Vec<_>>(),
        BenchmarkRobot::Floating(r) => (0..r.link_count())
            .map(|i| r.link_id_at(i).unwrap())
            .collect::<Vec<_>>(),
    };
    let mut output = vec![Frame::identity(); links.len()];
    let mut group = c.benchmark_group(format!("{}/fk_all", case.model.name));
    group.bench_function("individual", |b| {
        b.iter(|| {
            for (id, pose) in links.iter().zip(&mut output) {
                *pose = match &mut case.robot {
                    BenchmarkRobot::Fixed(r) => r.forward_kinematics(black_box(&case.q), *id),
                    BenchmarkRobot::Floating(r) => {
                        r.forward_kinematics(black_box(&case.base), black_box(&case.q), *id)
                    }
                }
                .unwrap();
            }
            black_box(&output);
        })
    });
    group.bench_function("batch", |b| {
        b.iter(|| {
            match &mut case.robot {
                BenchmarkRobot::Fixed(r) => {
                    r.forward_kinematics_all(black_box(&case.q), black_box(&mut output))
                }
                BenchmarkRobot::Floating(r) => r.forward_kinematics_all(
                    black_box(&case.base),
                    black_box(&case.q),
                    black_box(&mut output),
                ),
            }
            .unwrap();
            black_box(&output);
        })
    });
    group.finish();
}

fn benchmark_core(c: &mut Criterion) {
    let long_chain = Model {
        name: "chain40_fixed",
        path: "tests/data/test_arm_40.urdf",
        target: "test_link_40",
        floating: false,
        joints: 40,
    };
    for model in MODELS.into_iter().chain([long_chain]) {
        let mut case = Case::new(model);
        // The same validated cases as the Pinocchio comparison, without requiring C++.
        eprintln!(
            "{}: {} joints, {} generalized velocities",
            case.model.name,
            case.names.len(),
            case.g()
        );
        for operation in OPERATIONS {
            let mut group = c.benchmark_group(format!("{}/{operation}", model.name));
            bench_dynibo(&mut group, &mut case, operation);
            group.finish();
        }
        benchmark_all_poses(c, &mut case);
    }
}

criterion_group!(benches, benchmark_core);
criterion_main!(benches);
