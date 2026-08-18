use std::{
    alloc::{GlobalAlloc, Layout, System},
    hint::black_box,
    path::PathBuf,
    sync::{
        Mutex,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
};

use dynibo::{BaseMode, Frame, InverseKinematicsOptions, Robot};

struct CountingAllocator;

static COUNTING: AtomicBool = AtomicBool::new(false);
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static TEST_LOCK: Mutex<()> = Mutex::new(());

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        // SAFETY: Delegates the unchanged layout to the system allocator.
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        // SAFETY: Delegates the pointer and layout supplied by the caller.
        unsafe { System.dealloc(pointer, layout) }
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        // SAFETY: Delegates the caller-supplied allocation to the system allocator.
        unsafe { System.realloc(pointer, layout, new_size) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        // SAFETY: Delegates the unchanged layout to the system allocator.
        unsafe { System.alloc_zeroed(layout) }
    }
}

#[test]
fn floating_calculations_do_not_allocate_after_workspace_creation() {
    let _guard = TEST_LOCK.lock().unwrap();
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/floating_arm.urdf");
    let robot = Robot::from_urdf_with_base(path, BaseMode::Floating).unwrap();
    let target = robot.link_id("tool").unwrap();
    let q = [0.2, 0.1];
    let qd = [-0.3, 0.4];
    let qdd = [0.5, -0.2];
    let mut workspace = robot.workspace();
    let mut jacobian = [0.0; 48];
    let mut derivative = [0.0; 48];
    let mut matrix = [0.0; 64];
    let mut output = [0.0; 8];

    ALLOCATIONS.store(0, Ordering::Relaxed);
    COUNTING.store(true, Ordering::SeqCst);
    for _ in 0..10 {
        black_box(
            robot
                .forward_kinematics(&q, target, &mut workspace)
                .unwrap(),
        );
        robot
            .jacobian(&q, target, &mut workspace, &mut jacobian)
            .unwrap();
        robot
            .jacobian_derivative(&q, &qd, target, &mut workspace, &mut derivative)
            .unwrap();
        black_box(
            robot
                .forward_velocity_kinematics(&q, &qd, target, &Frame::identity(), &mut workspace)
                .unwrap(),
        );
        black_box(
            robot
                .forward_acceleration_kinematics(&q, &qd, &qdd, target, &mut workspace)
                .unwrap(),
        );
        robot.mass_matrix(&q, &mut workspace, &mut matrix).unwrap();
        robot
            .coriolis_matrix(&q, &qd, &mut workspace, &mut matrix)
            .unwrap();
        robot.gravity(&q, &[], &mut workspace, &mut output).unwrap();
        robot
            .inverse_dynamics(&q, &qd, &qdd, &[], &mut workspace, &mut output)
            .unwrap();
        black_box((&jacobian, &derivative, &matrix, &output));
    }
    COUNTING.store(false, Ordering::SeqCst);
    assert_eq!(ALLOCATIONS.load(Ordering::Relaxed), 0);
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

#[test]
fn dynamic_calculations_do_not_allocate_after_workspace_creation() {
    let _guard = TEST_LOCK.lock().unwrap();
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf");
    let robot = Robot::from_urdf(path).unwrap();
    let target_id = robot.link_id("test_link_4").unwrap();
    let q = [0.1, 0.8, -0.6, 0.3];
    let qd = [-0.2, 0.4, -0.1, 0.5];
    let qdd = [0.3, -0.2, 0.4, -0.1];
    let initial = [0.0; 4];
    let mut workspace = robot.workspace();
    let mut jacobian = [0.0; 24];
    let mut jacobian_derivative = [0.0; 24];
    let mut mass = [0.0; 16];
    let mut coriolis = [0.0; 16];
    let mut output = [0.0; 4];
    let desired = robot
        .forward_kinematics(&q, target_id, &mut workspace)
        .unwrap();

    robot
        .jacobian(&q, target_id, &mut workspace, &mut jacobian)
        .unwrap();
    robot
        .inverse_kinematics(
            &initial,
            target_id,
            &desired,
            InverseKinematicsOptions::default(),
            &mut workspace,
            &mut output,
        )
        .unwrap();

    ALLOCATIONS.store(0, Ordering::Relaxed);
    COUNTING.store(true, Ordering::SeqCst);
    for _ in 0..10 {
        black_box(
            robot
                .forward_kinematics(&q, target_id, &mut workspace)
                .unwrap(),
        );
        robot
            .jacobian(&q, target_id, &mut workspace, &mut jacobian)
            .unwrap();
        robot.mass_matrix(&q, &mut workspace, &mut mass).unwrap();
        robot
            .coriolis_matrix(&q, &qd, &mut workspace, &mut coriolis)
            .unwrap();
        robot
            .jacobian_derivative(&q, &qd, target_id, &mut workspace, &mut jacobian_derivative)
            .unwrap();
        black_box(
            robot
                .forward_velocity_kinematics(&q, &qd, target_id, &Frame::identity(), &mut workspace)
                .unwrap(),
        );
        black_box(
            robot
                .forward_acceleration_kinematics(&q, &qd, &qdd, target_id, &mut workspace)
                .unwrap(),
        );
        robot.gravity(&q, &[], &mut workspace, &mut output).unwrap();
        robot
            .inverse_dynamics(&q, &qd, &qdd, &[], &mut workspace, &mut output)
            .unwrap();
        robot
            .inverse_kinematics(
                &initial,
                target_id,
                &desired,
                InverseKinematicsOptions::default(),
                &mut workspace,
                &mut output,
            )
            .unwrap();
        black_box((&jacobian, &output));
    }
    COUNTING.store(false, Ordering::SeqCst);

    assert_eq!(ALLOCATIONS.load(Ordering::Relaxed), 0);
}
