use std::{
    ffi::CString,
    hint::black_box,
    path::{Path, PathBuf},
    ptr::NonNull,
};

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use dyno::{Frame, JointVector, Motion, RobotArm, Wrench};

unsafe extern "C" {
    fn dyno_pinocchio_create(urdf_path: *const std::ffi::c_char) -> *mut std::ffi::c_void;
    fn dyno_pinocchio_destroy(context: *mut std::ffi::c_void);
    fn dyno_pinocchio_dof(context: *const std::ffi::c_void) -> usize;
    fn dyno_pinocchio_noop(context: *const std::ffi::c_void, q: *const f64) -> f64;
    fn dyno_pinocchio_forward_kinematics(context: *mut std::ffi::c_void, q: *const f64) -> f64;
    fn dyno_pinocchio_jacobian(context: *mut std::ffi::c_void, q: *const f64) -> f64;
    fn dyno_pinocchio_gravity(context: *mut std::ffi::c_void, q: *const f64) -> f64;
    fn dyno_pinocchio_rnea(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
    ) -> f64;
}

struct PinocchioContext(NonNull<std::ffi::c_void>);

impl PinocchioContext {
    fn new(urdf_path: &std::path::Path) -> Self {
        let path = CString::new(urdf_path.to_string_lossy().as_bytes())
            .expect("URDF path must not contain a NUL byte");
        // SAFETY: `path` is a valid, NUL-terminated string for the duration of the call.
        let context = unsafe { dyno_pinocchio_create(path.as_ptr()) };
        Self(NonNull::new(context).expect("Pinocchio failed to load the benchmark URDF"))
    }

    fn dof(&self) -> usize {
        // SAFETY: the context is owned by `self` and remains valid until `drop`.
        unsafe { dyno_pinocchio_dof(self.0.as_ptr()) }
    }
}

impl Drop for PinocchioContext {
    fn drop(&mut self) {
        // SAFETY: this pointer was returned by `dyno_pinocchio_create` and is destroyed once.
        unsafe { dyno_pinocchio_destroy(self.0.as_ptr()) };
    }
}

struct BenchmarkCase<const N: usize> {
    arm: RobotArm<N>,
    pinocchio: PinocchioContext,
    q: JointVector<N>,
    qd: JointVector<N>,
    qdd: JointVector<N>,
    base: Frame,
}

impl<const N: usize> BenchmarkCase<N> {
    fn new(relative_urdf_path: impl AsRef<Path>) -> Self {
        let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative_urdf_path.as_ref());
        let arm =
            RobotArm::<N>::from_urdf_file(&urdf_path).expect("dyno must load the benchmark URDF");
        let pinocchio = PinocchioContext::new(&urdf_path);
        assert_eq!(
            pinocchio.dof(),
            N,
            "Pinocchio and Dyno must load the same number of DoF"
        );

        Self {
            arm,
            pinocchio,
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
        b.iter(|| black_box(case.arm.forward_kinematics(black_box(q))));
    });
    fk.bench_with_input(BenchmarkId::from_parameter("pinocchio"), &case.q, |b, q| {
        b.iter(|| {
            // SAFETY: context and input vector remain valid for the duration of every call.
            black_box(unsafe {
                dyno_pinocchio_forward_kinematics(case.pinocchio.0.as_ptr(), black_box(q).as_ptr())
            })
        });
    });
    fk.finish();

    let mut jacobian = c.benchmark_group(format!("end_jacobian/{size}"));
    jacobian.throughput(Throughput::Elements(1));
    jacobian.bench_with_input(BenchmarkId::from_parameter("dyno"), &case.q, |b, q| {
        b.iter(|| black_box(case.arm.jacobian(black_box(q))))
    });
    jacobian.bench_with_input(BenchmarkId::from_parameter("pinocchio"), &case.q, |b, q| {
        b.iter(|| {
            // SAFETY: context and input vector remain valid for the duration of every call.
            black_box(unsafe {
                dyno_pinocchio_jacobian(case.pinocchio.0.as_ptr(), black_box(q).as_ptr())
            })
        });
    });
    jacobian.finish();

    let mut gravity = c.benchmark_group(format!("gravity/{size}"));
    gravity.throughput(Throughput::Elements(1));
    gravity.bench_with_input(BenchmarkId::from_parameter("dyno"), &case.q, |b, q| {
        b.iter(|| {
            black_box(
                case.arm
                    .gravity_torque(black_box(q), &case.base, Wrench::zeros()),
            )
        });
    });
    gravity.bench_with_input(BenchmarkId::from_parameter("pinocchio"), &case.q, |b, q| {
        b.iter(|| {
            // SAFETY: context and input vector remain valid for the duration of every call.
            black_box(unsafe {
                dyno_pinocchio_gravity(case.pinocchio.0.as_ptr(), black_box(q).as_ptr())
            })
        });
    });
    gravity.finish();

    let mut rnea = c.benchmark_group(format!("rnea/{size}"));
    rnea.throughput(Throughput::Elements(1));
    rnea.bench_function("dyno", |b| {
        b.iter(|| {
            black_box(case.arm.inverse_dynamics(
                black_box(&case.q),
                black_box(&case.qd),
                black_box(&case.qdd),
                &case.base,
                Motion::zeros(),
                Motion::zeros(),
                Wrench::zeros(),
            ))
        });
    });
    rnea.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: context and input vectors remain valid for the duration of every call.
            black_box(unsafe {
                dyno_pinocchio_rnea(
                    case.pinocchio.0.as_ptr(),
                    black_box(case.q.as_ptr()),
                    black_box(case.qd.as_ptr()),
                    black_box(case.qdd.as_ptr()),
                )
            })
        });
    });
    rnea.finish();
}

fn benchmark_pinocchio(c: &mut Criterion) {
    let case_4 = BenchmarkCase::<4>::new("tests/data/test_arm.urdf");

    let mut overhead = c.benchmark_group("ffi_overhead");
    overhead.throughput(Throughput::Elements(1));
    overhead.bench_function("pinocchio_c_abi", |b| {
        b.iter(|| {
            // SAFETY: context and input vector remain valid for the duration of every call.
            black_box(unsafe {
                dyno_pinocchio_noop(case_4.pinocchio.0.as_ptr(), case_4.q.as_ptr())
            })
        });
    });
    overhead.finish();

    benchmark_case(c, &case_4);
    benchmark_case(
        c,
        &BenchmarkCase::<40>::new("benches/data/test_arm_40.urdf"),
    );
}

criterion_group!(benches, benchmark_pinocchio);
criterion_main!(benches);
