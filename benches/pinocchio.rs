use std::{
    ffi::CString,
    hint::black_box,
    path::{Path, PathBuf},
    ptr::NonNull,
};

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use dynibo::{BaseMode, LinkId, Robot};

unsafe extern "C" {
    fn dynibo_pinocchio_create_for_frame(
        urdf_path: *const std::ffi::c_char,
        frame_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
    fn dynibo_pinocchio_create_floating_for_frame(
        urdf_path: *const std::ffi::c_char,
        frame_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
    fn dynibo_pinocchio_destroy(context: *mut std::ffi::c_void);
    fn dynibo_pinocchio_dof(context: *const std::ffi::c_void) -> usize;
    fn dynibo_pinocchio_configuration_size(context: *const std::ffi::c_void) -> usize;
    fn dynibo_pinocchio_neutral_configuration(context: *const std::ffi::c_void, q: *mut f64);
    fn dynibo_pinocchio_noop(context: *const std::ffi::c_void, q: *const f64) -> f64;
    fn dynibo_pinocchio_forward_kinematics(context: *mut std::ffi::c_void, q: *const f64) -> f64;
    fn dynibo_pinocchio_jacobian(context: *mut std::ffi::c_void, q: *const f64) -> f64;
    fn dynibo_pinocchio_gravity(context: *mut std::ffi::c_void, q: *const f64) -> f64;
    fn dynibo_pinocchio_rnea(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
    ) -> f64;
    fn dynibo_pinocchio_crba(context: *mut std::ffi::c_void, q: *const f64) -> f64;
    fn dynibo_pinocchio_velocity_product(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
    ) -> f64;
    fn dynibo_pinocchio_jacobian_time_variation(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
    ) -> f64;
}

struct PinocchioContext(NonNull<std::ffi::c_void>);

impl PinocchioContext {
    fn new_for_frame(urdf_path: &Path, frame_name: &str, base_mode: BaseMode) -> Self {
        let path = CString::new(urdf_path.to_string_lossy().as_bytes())
            .expect("URDF path must not contain a NUL byte");
        let frame_name = CString::new(frame_name).expect("frame name must not contain a NUL byte");
        // SAFETY: both arguments are valid, NUL-terminated strings for the duration of the call.
        let context = unsafe {
            match base_mode {
                BaseMode::Fixed => {
                    dynibo_pinocchio_create_for_frame(path.as_ptr(), frame_name.as_ptr())
                }
                BaseMode::Floating => {
                    dynibo_pinocchio_create_floating_for_frame(path.as_ptr(), frame_name.as_ptr())
                }
            }
        };
        Self(NonNull::new(context).expect("Pinocchio failed to load the benchmark URDF"))
    }

    fn dof(&self) -> usize {
        // SAFETY: the context is owned by `self` and remains valid until `drop`.
        unsafe { dynibo_pinocchio_dof(self.0.as_ptr()) }
    }

    fn configuration_size(&self) -> usize {
        // SAFETY: the context is owned by `self` and remains valid until `drop`.
        unsafe { dynibo_pinocchio_configuration_size(self.0.as_ptr()) }
    }

    fn neutral_configuration(&self) -> Vec<f64> {
        let mut configuration = vec![0.0; self.configuration_size()];
        // SAFETY: the output vector matches the configuration size reported by the context.
        unsafe {
            dynibo_pinocchio_neutral_configuration(self.0.as_ptr(), configuration.as_mut_ptr())
        };
        configuration
    }
}

impl Drop for PinocchioContext {
    fn drop(&mut self) {
        // SAFETY: this pointer was returned by the bridge and is destroyed exactly once.
        unsafe { dynibo_pinocchio_destroy(self.0.as_ptr()) };
    }
}

struct BenchmarkCase<const N: usize> {
    robot: Robot,
    pinocchio: PinocchioContext,
    target: LinkId,
    q: [f64; N],
    qd: [f64; N],
    qdd: [f64; N],
    pinocchio_q: Vec<f64>,
    pinocchio_qd: Vec<f64>,
    pinocchio_qdd: Vec<f64>,
}

impl<const N: usize> BenchmarkCase<N> {
    fn new(relative_urdf_path: impl AsRef<Path>, target_link: &str, base_mode: BaseMode) -> Self {
        let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative_urdf_path.as_ref());
        let robot = Robot::from_urdf_with_base(&urdf_path, base_mode)
            .expect("Dynibo must load the benchmark URDF");
        let target = robot
            .link_id(target_link)
            .expect("target link must exist in the benchmark URDF");
        let pinocchio = PinocchioContext::new_for_frame(&urdf_path, target_link, base_mode);

        assert_eq!(robot.joint_count(), N);
        assert_eq!(pinocchio.dof(), robot.generalized_count());

        let q = std::array::from_fn(|row| (0.37 * (row + 1) as f64).sin() * 0.5);
        let qd = std::array::from_fn(|row| (0.23 * (row + 1) as f64).cos() * 0.4);
        let qdd = std::array::from_fn(|row| (0.41 * (row + 1) as f64).sin() * 0.3);

        let mut pinocchio_q = pinocchio.neutral_configuration();
        let configuration_offset = pinocchio_q.len() - N;
        pinocchio_q[configuration_offset..].copy_from_slice(&q);
        let velocity_offset = robot.generalized_count() - N;
        let mut pinocchio_qd = vec![0.0; robot.generalized_count()];
        pinocchio_qd[velocity_offset..].copy_from_slice(&qd);
        let mut pinocchio_qdd = vec![0.0; robot.generalized_count()];
        pinocchio_qdd[velocity_offset..].copy_from_slice(&qdd);

        Self {
            robot,
            pinocchio,
            target,
            q,
            qd,
            qdd,
            pinocchio_q,
            pinocchio_qd,
            pinocchio_qdd,
        }
    }
}

fn benchmark_case<const N: usize>(c: &mut Criterion, model: &str, case: &BenchmarkCase<N>) {
    let generalized_count = case.robot.generalized_count();

    let mut fk = c.benchmark_group(format!("forward_kinematics/{model}"));
    fk.throughput(Throughput::Elements(1));
    let mut workspace = case.robot.workspace();
    fk.bench_function("dynibo", |b| {
        b.iter(|| {
            black_box(
                case.robot
                    .forward_kinematics(black_box(&case.q), case.target, &mut workspace)
                    .unwrap(),
            )
        });
    });
    fk.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: the context and configuration remain valid for every call.
            black_box(unsafe {
                dynibo_pinocchio_forward_kinematics(
                    case.pinocchio.0.as_ptr(),
                    black_box(case.pinocchio_q.as_ptr()),
                )
            })
        });
    });
    fk.finish();

    let mut jacobian = c.benchmark_group(format!("end_jacobian/{model}"));
    jacobian.throughput(Throughput::Elements(1));
    let mut workspace = case.robot.workspace();
    let mut output = vec![0.0; 6 * generalized_count];
    jacobian.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
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
            // SAFETY: the context and configuration remain valid for every call.
            black_box(unsafe {
                dynibo_pinocchio_jacobian(
                    case.pinocchio.0.as_ptr(),
                    black_box(case.pinocchio_q.as_ptr()),
                )
            })
        });
    });
    jacobian.finish();

    let mut acceleration = c.benchmark_group(format!("forward_acceleration/{model}"));
    acceleration.throughput(Throughput::Elements(1));
    let mut workspace = case.robot.workspace();
    acceleration.bench_function("dynibo", |b| {
        b.iter(|| {
            black_box(
                case.robot
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

    let mut gravity = c.benchmark_group(format!("gravity/{model}"));
    gravity.throughput(Throughput::Elements(1));
    let mut workspace = case.robot.workspace();
    let mut output = vec![0.0; generalized_count];
    gravity.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .gravity(
                    black_box(&case.q),
                    black_box(&[]),
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    gravity.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: the context and configuration remain valid for every call.
            black_box(unsafe {
                dynibo_pinocchio_gravity(
                    case.pinocchio.0.as_ptr(),
                    black_box(case.pinocchio_q.as_ptr()),
                )
            })
        });
    });
    gravity.finish();

    let mut rnea = c.benchmark_group(format!("rnea/{model}"));
    rnea.throughput(Throughput::Elements(1));
    let mut workspace = case.robot.workspace();
    let mut output = vec![0.0; generalized_count];
    rnea.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .inverse_dynamics(
                    black_box(&case.q),
                    black_box(&case.qd),
                    black_box(&case.qdd),
                    black_box(&[]),
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    rnea.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: the context and state vectors remain valid for every call.
            black_box(unsafe {
                dynibo_pinocchio_rnea(
                    case.pinocchio.0.as_ptr(),
                    black_box(case.pinocchio_q.as_ptr()),
                    black_box(case.pinocchio_qd.as_ptr()),
                    black_box(case.pinocchio_qdd.as_ptr()),
                )
            })
        });
    });
    rnea.finish();

    let mut jacobian_derivative = c.benchmark_group(format!("end_jacobian_derivative/{model}"));
    jacobian_derivative.throughput(Throughput::Elements(1));
    let mut workspace = case.robot.workspace();
    let mut output = vec![0.0; 6 * generalized_count];
    jacobian_derivative.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .jacobian_derivative(
                    black_box(&case.q),
                    black_box(&case.qd),
                    case.target,
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    jacobian_derivative.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: the context and state vectors remain valid for every call.
            black_box(unsafe {
                dynibo_pinocchio_jacobian_time_variation(
                    case.pinocchio.0.as_ptr(),
                    black_box(case.pinocchio_q.as_ptr()),
                    black_box(case.pinocchio_qd.as_ptr()),
                )
            })
        });
    });
    jacobian_derivative.finish();

    let mut mass = c.benchmark_group(format!("mass_matrix/{model}"));
    mass.throughput(Throughput::Elements(1));
    let mut workspace = case.robot.workspace();
    let mut output = vec![0.0; generalized_count * generalized_count];
    mass.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .mass_matrix(black_box(&case.q), &mut workspace, black_box(&mut output))
                .unwrap();
            black_box(&output);
        });
    });
    mass.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: the context and configuration remain valid for every call.
            black_box(unsafe {
                dynibo_pinocchio_crba(
                    case.pinocchio.0.as_ptr(),
                    black_box(case.pinocchio_q.as_ptr()),
                )
            })
        });
    });
    mass.finish();

    let mut velocity_product = c.benchmark_group(format!("velocity_product_forces/{model}"));
    velocity_product.throughput(Throughput::Elements(1));
    let mut workspace = case.robot.workspace();
    let mut output = vec![0.0; generalized_count];
    velocity_product.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .velocity_product_forces(
                    black_box(&case.q),
                    black_box(&case.qd),
                    &mut workspace,
                    black_box(&mut output),
                )
                .unwrap();
            black_box(&output);
        });
    });
    velocity_product.bench_function("pinocchio", |b| {
        b.iter(|| {
            // SAFETY: the context and state vectors remain valid for every call.
            black_box(unsafe {
                dynibo_pinocchio_velocity_product(
                    case.pinocchio.0.as_ptr(),
                    black_box(case.pinocchio_q.as_ptr()),
                    black_box(case.pinocchio_qd.as_ptr()),
                )
            })
        });
    });
    velocity_product.finish();
}

fn benchmark_pinocchio(c: &mut Criterion) {
    let fixed_serial = BenchmarkCase::<40>::new(
        "tests/data/test_arm_40.urdf",
        "test_link_40",
        BaseMode::Fixed,
    );
    let floating_serial = BenchmarkCase::<40>::new(
        "tests/data/test_arm_40.urdf",
        "test_link_40",
        BaseMode::Floating,
    );
    let fixed_tree =
        BenchmarkCase::<7>::new("tests/data/test_tree_7.urdf", "right_tool", BaseMode::Fixed);
    let floating_tree = BenchmarkCase::<7>::new(
        "tests/data/test_tree_7.urdf",
        "right_tool",
        BaseMode::Floating,
    );

    let mut overhead = c.benchmark_group("ffi_overhead");
    overhead.throughput(Throughput::Elements(1));
    overhead.bench_function("pinocchio_c_abi", |b| {
        b.iter(|| {
            // SAFETY: the context and configuration remain valid for every call.
            black_box(unsafe {
                dynibo_pinocchio_noop(
                    floating_serial.pinocchio.0.as_ptr(),
                    floating_serial.pinocchio_q.as_ptr(),
                )
            })
        });
    });
    overhead.finish();

    benchmark_case(c, "fixed_serial_40joint", &fixed_serial);
    benchmark_case(c, "floating_serial_40joint", &floating_serial);
    benchmark_case(c, "fixed_tree_7joint_2leaf", &fixed_tree);
    benchmark_case(c, "floating_tree_7joint_2leaf", &floating_tree);
}

criterion_group!(benches, benchmark_pinocchio);
criterion_main!(benches);
