// The shared module also contains the optional comparison-only Pinocchio helpers.
#[allow(dead_code)]
mod support;

use criterion::{Criterion, criterion_group, criterion_main};
use support::{Case, MODELS, OPERATIONS, bench_dynibo};

fn benchmark_core(c: &mut Criterion) {
    for model in MODELS {
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
    }
}

criterion_group!(benches, benchmark_core);
criterion_main!(benches);
