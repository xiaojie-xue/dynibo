use std::{
    ffi::CString,
    hint::black_box,
    path::{Path, PathBuf},
    ptr::NonNull,
};

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use dyno::{Frame, LinkId, Robot, Twist};

unsafe extern "C" {
    fn dyno_pinocchio_create(urdf_path: *const std::ffi::c_char) -> *mut std::ffi::c_void;
    fn dyno_pinocchio_create_for_joint(
        urdf_path: *const std::ffi::c_char,
        end_joint_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
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

    fn new_for_joint(urdf_path: &std::path::Path, end_joint_name: &str) -> Self {
        let path = CString::new(urdf_path.to_string_lossy().as_bytes())
            .expect("URDF path must not contain a NUL byte");
        let joint_name =
            CString::new(end_joint_name).expect("joint name must not contain a NUL byte");
        // SAFETY: both arguments are valid, NUL-terminated strings for the duration of the call.
        let context =
            unsafe { dyno_pinocchio_create_for_joint(path.as_ptr(), joint_name.as_ptr()) };
        Self(NonNull::new(context).expect("Pinocchio failed to load the requested end joint"))
    }

    fn dof(&self) -> usize {
        // SAFETY: the context is owned by `self` and remains valid until `drop`.
        unsafe { dyno_pinocchio_dof(self.0.as_ptr()) }
    }
}

struct TreeBenchmarkCase<const N: usize> {
    arm: Robot,
    pinocchio: PinocchioContext,
    target: LinkId,
    q: [f64; N],
    qd: [f64; N],
    qdd: [f64; N],
    base: Frame,
}

impl<const N: usize> TreeBenchmarkCase<N> {
    fn new(relative_urdf_path: impl AsRef<Path>, target_link: &str, target_joint: &str) -> Self {
        let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative_urdf_path.as_ref());
        let arm = Robot::from_urdf(&urdf_path).expect("Dyno must load the tree benchmark URDF");
        let target = arm
            .link_id(target_link)
            .expect("target link must exist in the tree benchmark URDF");
        let pinocchio = PinocchioContext::new_for_joint(&urdf_path, target_joint);
        assert_eq!(pinocchio.dof(), N);
        assert_eq!(arm.joint_count(), N);

        Self {
            arm,
            pinocchio,
            target,
            q: [0.2; N],
            qd: [-0.1; N],
            qdd: [0.15; N],
            base: Frame::identity(),
        }
    }
}

impl Drop for PinocchioContext {
    fn drop(&mut self) {
        // SAFETY: this pointer was returned by `dyno_pinocchio_create` and is destroyed once.
        unsafe { dyno_pinocchio_destroy(self.0.as_ptr()) };
    }
}

struct BenchmarkCase<const N: usize> {
    arm: Robot,
    pinocchio: PinocchioContext,
    target: LinkId,
    q: [f64; N],
    qd: [f64; N],
    qdd: [f64; N],
    base: Frame,
}

impl<const N: usize> BenchmarkCase<N> {
    fn new(relative_urdf_path: impl AsRef<Path>) -> Self {
        let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative_urdf_path.as_ref());
        let arm = Robot::from_urdf(&urdf_path).expect("dyno must load the benchmark URDF");
        let pinocchio = PinocchioContext::new(&urdf_path);
        assert_eq!(
            pinocchio.dof(),
            N,
            "Pinocchio and Dyno must load the same number of DoF"
        );
        let leaf = arm
            .leaf_links()
            .next()
            .expect("benchmark robot must have a leaf link");
        let target = arm
            .link_id(leaf.name())
            .expect("leaf link must belong to the benchmark robot");

        Self {
            arm,
            pinocchio,
            target,
            q: std::array::from_fn(|row| (0.37 * (row + 1) as f64).sin() * 0.5),
            qd: std::array::from_fn(|row| (0.23 * (row + 1) as f64).cos() * 0.4),
            qdd: std::array::from_fn(|row| (0.41 * (row + 1) as f64).sin() * 0.3),
            base: Frame::identity(),
        }
    }
}

fn benchmark_case<const N: usize>(c: &mut Criterion, case: &BenchmarkCase<N>) {
    let size = format!("{N}dof");

    let mut fk = c.benchmark_group(format!("forward_kinematics/{size}"));
    fk.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    fk.bench_with_input(BenchmarkId::from_parameter("dyno"), &case.q, |b, q| {
        b.iter(|| {
            black_box(
                case.arm
                    .forward_kinematics(black_box(q), case.target, &mut workspace)
                    .unwrap(),
            )
        });
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
    let mut workspace = case.arm.workspace();
    let mut output = vec![0.0; 6 * N];
    jacobian.bench_with_input(BenchmarkId::from_parameter("dyno"), &case.q, |b, q| {
        b.iter(|| {
            case.arm
                .jacobian(
                    black_box(q),
                    case.target,
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        })
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
    let mut output = [0.0; N];
    gravity.bench_with_input(BenchmarkId::from_parameter("dyno"), &case.q, |b, q| {
        b.iter(|| {
            case.arm
                .gravity(
                    black_box(q),
                    &case.base,
                    black_box(&[]),
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(output);
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
    let mut workspace = case.arm.workspace();
    let mut output = [0.0; N];
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
            black_box(output);
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

fn benchmark_tree_case<const N: usize>(c: &mut Criterion, case: &TreeBenchmarkCase<N>) {
    let size = format!("tree_{N}joint_2leaf");

    let mut fk = c.benchmark_group(format!("tree_forward_kinematics/{size}"));
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
    fk.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: context and input vector remain valid for the duration of every call.
            black_box(unsafe {
                dyno_pinocchio_forward_kinematics(
                    case.pinocchio.0.as_ptr(),
                    black_box(case.q.as_ptr()),
                )
            })
        });
    });
    fk.finish();

    let mut jacobian = c.benchmark_group(format!("tree_jacobian/{size}"));
    jacobian.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    let mut output = vec![0.0; 6 * N];
    jacobian.bench_function("dyno", |b| {
        b.iter(|| {
            case.arm
                .jacobian(
                    black_box(&case.q),
                    case.target,
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    jacobian.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: context and input vector remain valid for the duration of every call.
            black_box(unsafe {
                dyno_pinocchio_jacobian(case.pinocchio.0.as_ptr(), black_box(case.q.as_ptr()))
            })
        });
    });
    jacobian.finish();

    let mut gravity = c.benchmark_group(format!("tree_gravity/{size}"));
    gravity.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    let mut output = [0.0; N];
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
            black_box(output);
        });
    });
    gravity.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: context and input vector remain valid for the duration of every call.
            black_box(unsafe {
                dyno_pinocchio_gravity(case.pinocchio.0.as_ptr(), black_box(case.q.as_ptr()))
            })
        });
    });
    gravity.finish();

    let mut rnea = c.benchmark_group(format!("tree_rnea/{size}"));
    rnea.throughput(Throughput::Elements(1));
    let mut workspace = case.arm.workspace();
    let mut output = [0.0; N];
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
            black_box(output);
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
    benchmark_case(c, &BenchmarkCase::<40>::new("tests/data/test_arm_40.urdf"));
    benchmark_tree_case(
        c,
        &TreeBenchmarkCase::<7>::new("tests/data/test_tree_7.urdf", "right_tool", "right_wrist"),
    );
}

criterion_group!(benches, benchmark_pinocchio);
criterion_main!(benches);
