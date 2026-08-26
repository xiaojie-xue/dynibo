use std::{
    ffi::CString,
    hint::black_box,
    path::{Path, PathBuf},
    ptr::NonNull,
};

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use dynibo::{BaseState, FloatingRobot, Frame, IndexedLoad, LinkId, Robot, Twist};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BenchmarkRootMode {
    Fixed,
    Floating,
}

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
    fn new_for_frame(urdf_path: &Path, frame_name: &str, base_mode: BenchmarkRootMode) -> Self {
        let path = CString::new(urdf_path.to_string_lossy().as_bytes())
            .expect("URDF path must not contain a NUL byte");
        let frame_name = CString::new(frame_name).expect("frame name must not contain a NUL byte");
        // SAFETY: both arguments are valid, NUL-terminated strings for the duration of the call.
        let context = unsafe {
            match base_mode {
                BenchmarkRootMode::Fixed => {
                    dynibo_pinocchio_create_for_frame(path.as_ptr(), frame_name.as_ptr())
                }
                BenchmarkRootMode::Floating => {
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

enum BenchmarkRobot {
    Fixed(Robot),
    Floating(FloatingRobot),
}

impl BenchmarkRobot {
    fn from_urdf(path: &Path, mode: BenchmarkRootMode) -> dynibo::Result<Self> {
        match mode {
            BenchmarkRootMode::Fixed => Robot::from_urdf(path).map(Self::Fixed),
            BenchmarkRootMode::Floating => FloatingRobot::from_urdf(path).map(Self::Floating),
        }
    }

    fn link_id(&self, name: &str) -> dynibo::Result<LinkId> {
        match self {
            Self::Fixed(robot) => robot.link_id(name),
            Self::Floating(robot) => robot.link_id(name),
        }
    }
    fn joint_count(&self) -> usize {
        match self {
            Self::Fixed(robot) => robot.joint_count(),
            Self::Floating(robot) => robot.joint_count(),
        }
    }
    fn generalized_count(&self) -> usize {
        match self {
            Self::Fixed(robot) => robot.generalized_count(),
            Self::Floating(robot) => robot.generalized_count(),
        }
    }
    fn forward_kinematics(
        &mut self,
        base: &BaseState,
        q: &[f64],
        target: LinkId,
    ) -> dynibo::Result<Frame> {
        match self {
            Self::Fixed(robot) => robot.forward_kinematics(q, target),
            Self::Floating(robot) => robot.forward_kinematics(base, q, target),
        }
    }
    fn jacobian(
        &mut self,
        base: &BaseState,
        q: &[f64],
        target: LinkId,
        output: &mut [f64],
    ) -> dynibo::Result<()> {
        match self {
            Self::Fixed(robot) => robot.jacobian(q, target, output),
            Self::Floating(robot) => robot.jacobian(base, q, target, output),
        }
    }
    fn forward_acceleration_kinematics(
        &mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        target: LinkId,
    ) -> dynibo::Result<Twist> {
        match self {
            Self::Fixed(robot) => robot.forward_acceleration_kinematics(q, qd, qdd, target),
            Self::Floating(robot) => {
                robot.forward_acceleration_kinematics(base, q, qd, qdd, target)
            }
        }
    }
    fn gravity(
        &mut self,
        base: &BaseState,
        q: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> dynibo::Result<()> {
        match self {
            Self::Fixed(robot) => robot.gravity(q, loads, output),
            Self::Floating(robot) => robot.gravity(base, q, loads, output),
        }
    }
    fn inverse_dynamics(
        &mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        loads: &[IndexedLoad],
        output: &mut [f64],
    ) -> dynibo::Result<()> {
        match self {
            Self::Fixed(robot) => robot.inverse_dynamics(q, qd, qdd, loads, output),
            Self::Floating(robot) => robot.inverse_dynamics(base, q, qd, qdd, loads, output),
        }
    }
    fn jacobian_derivative(
        &mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        target: LinkId,
        output: &mut [f64],
    ) -> dynibo::Result<()> {
        match self {
            Self::Fixed(robot) => robot.jacobian_derivative(q, qd, target, output),
            Self::Floating(robot) => robot.jacobian_derivative(base, q, qd, target, output),
        }
    }
    fn mass_matrix(
        &mut self,
        base: &BaseState,
        q: &[f64],
        output: &mut [f64],
    ) -> dynibo::Result<()> {
        match self {
            Self::Fixed(robot) => robot.mass_matrix(q, output),
            Self::Floating(robot) => robot.mass_matrix(base, q, output),
        }
    }
    fn velocity_product_forces(
        &mut self,
        base: &BaseState,
        q: &[f64],
        qd: &[f64],
        output: &mut [f64],
    ) -> dynibo::Result<()> {
        match self {
            Self::Fixed(robot) => robot.velocity_product_forces(q, qd, output),
            Self::Floating(robot) => robot.velocity_product_forces(base, q, qd, output),
        }
    }
}

struct BenchmarkCase<const N: usize> {
    robot: BenchmarkRobot,
    base: BaseState,
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
    fn new(
        relative_urdf_path: impl AsRef<Path>,
        target_link: &str,
        base_mode: BenchmarkRootMode,
    ) -> Self {
        let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative_urdf_path.as_ref());
        let mut robot = BenchmarkRobot::from_urdf(&urdf_path, base_mode)
            .expect("Dynibo must load the benchmark URDF");
        let target = robot
            .link_id(target_link)
            .expect("target link must exist in the benchmark URDF");
        let pinocchio = PinocchioContext::new_for_frame(&urdf_path, target_link, base_mode);
        let base = match base_mode {
            BenchmarkRootMode::Fixed | BenchmarkRootMode::Floating => {
                BaseState::stationary(Frame::identity()).unwrap()
            }
        };
        if let BenchmarkRobot::Fixed(fixed) = &mut robot {
            fixed.set_base_frame(*base.frame()).unwrap();
        }

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
            base,
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

fn benchmark_case<const N: usize>(c: &mut Criterion, model: &str, case: &mut BenchmarkCase<N>) {
    let generalized_count = case.robot.generalized_count();

    let mut fk = c.benchmark_group(format!("forward_kinematics/{model}"));
    fk.throughput(Throughput::Elements(1));
    fk.bench_function("dynibo", |b| {
        b.iter(|| {
            black_box(
                case.robot
                    .forward_kinematics(&case.base, black_box(&case.q), case.target)
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
    let mut output = vec![0.0; 6 * generalized_count];
    jacobian.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .jacobian(
                    &case.base,
                    black_box(&case.q),
                    case.target,
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
    acceleration.bench_function("dynibo", |b| {
        b.iter(|| {
            black_box(
                case.robot
                    .forward_acceleration_kinematics(
                        &case.base,
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

    let mut gravity = c.benchmark_group(format!("gravity/{model}"));
    gravity.throughput(Throughput::Elements(1));
    let mut output = vec![0.0; generalized_count];
    gravity.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .gravity(
                    &case.base,
                    black_box(&case.q),
                    black_box(&[]),
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
    let mut output = vec![0.0; generalized_count];
    rnea.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .inverse_dynamics(
                    &case.base,
                    black_box(&case.q),
                    black_box(&case.qd),
                    black_box(&case.qdd),
                    black_box(&[]),
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
    let mut output = vec![0.0; 6 * generalized_count];
    jacobian_derivative.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .jacobian_derivative(
                    &case.base,
                    black_box(&case.q),
                    black_box(&case.qd),
                    case.target,
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
    let mut output = vec![0.0; generalized_count * generalized_count];
    mass.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .mass_matrix(&case.base, black_box(&case.q), black_box(&mut output))
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
    let mut output = vec![0.0; generalized_count];
    velocity_product.bench_function("dynibo", |b| {
        b.iter(|| {
            case.robot
                .velocity_product_forces(
                    &case.base,
                    black_box(&case.q),
                    black_box(&case.qd),
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
    let mut fixed_serial = BenchmarkCase::<40>::new(
        "tests/data/test_arm_40.urdf",
        "test_link_40",
        BenchmarkRootMode::Fixed,
    );
    let mut floating_serial = BenchmarkCase::<40>::new(
        "tests/data/test_arm_40.urdf",
        "test_link_40",
        BenchmarkRootMode::Floating,
    );
    let mut fixed_tree = BenchmarkCase::<7>::new(
        "tests/data/test_tree_7.urdf",
        "right_tool",
        BenchmarkRootMode::Fixed,
    );
    let mut floating_tree = BenchmarkCase::<7>::new(
        "tests/data/test_tree_7.urdf",
        "right_tool",
        BenchmarkRootMode::Floating,
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

    benchmark_case(c, "fixed_serial_40joint", &mut fixed_serial);
    benchmark_case(c, "floating_serial_40joint", &mut floating_serial);
    benchmark_case(c, "fixed_tree_7joint_2leaf", &mut fixed_tree);
    benchmark_case(c, "floating_tree_7joint_2leaf", &mut floating_tree);
}

criterion_group!(benches, benchmark_pinocchio);
criterion_main!(benches);
