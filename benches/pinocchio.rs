mod support;

use criterion::{Criterion, criterion_group, criterion_main};
use std::{hint::black_box, time::Duration};
use support::{Case, MODELS, OPERATIONS, bench_dynibo, pinocchio::Pinocchio};

fn benchmark_pinocchio(c: &mut Criterion) {
    for model in MODELS {
        let mut case = Case::new(model);
        let mut pin = Pinocchio::new(&case);
        // Check complete outputs before any timing, including base coordinate conversion.
        pin.validate(&mut case);
        for operation in OPERATIONS {
            let mut group = c.benchmark_group(format!("{}/{operation}", model.name));
            bench_dynibo(&mut group, &mut case, operation);
            let mut output = vec![0.0; case.g() * if operation == "jacobian" { 6 } else { 1 }];
            group.bench_function("pinocchio", |b| {
                b.iter(|| {
                    pin.calculate(operation, &mut output);
                    black_box(&output);
                })
            });
            group.finish();
        }
    }
}

criterion_group! {
    name = benches;
    config = Criterion::default().sample_size(50)
        .warm_up_time(Duration::from_secs(1)).measurement_time(Duration::from_secs(3));
    targets = benchmark_pinocchio
}
criterion_main!(benches);
