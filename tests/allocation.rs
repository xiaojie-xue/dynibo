use std::{
    alloc::{GlobalAlloc, Layout, System},
    cell::Cell,
    hint::black_box,
    path::PathBuf,
};

use dynibo::{FloatingRobot, Frame, InverseKinematicsOptions, Robot};

struct CountingAllocator;

thread_local! {
    static COUNTING: Cell<bool> = const { Cell::new(false) };
    static ALLOCATIONS: Cell<usize> = const { Cell::new(0) };
}

fn record_allocation() {
    COUNTING.with(|counting| {
        if counting.get() {
            ALLOCATIONS.with(|allocations| allocations.set(allocations.get() + 1));
        }
    });
}

fn reset_allocation_count() {
    ALLOCATIONS.with(|allocations| allocations.set(0));
}

fn set_counting(enabled: bool) {
    COUNTING.with(|counting| counting.set(enabled));
}

fn allocation_count() -> usize {
    ALLOCATIONS.with(Cell::get)
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record_allocation();
        // SAFETY: Delegates the unchanged layout to the system allocator.
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        // SAFETY: Delegates the pointer and layout supplied by the caller.
        unsafe { System.dealloc(pointer, layout) }
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        record_allocation();
        // SAFETY: Delegates the caller-supplied allocation to the system allocator.
        unsafe { System.realloc(pointer, layout, new_size) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        record_allocation();
        // SAFETY: Delegates the unchanged layout to the system allocator.
        unsafe { System.alloc_zeroed(layout) }
    }
}

#[test]
fn floating_calculations_do_not_allocate_after_robot_creation() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/floating_arm.urdf");
    let mut robot = FloatingRobot::from_urdf(path).unwrap();
    let target = robot.link_id("tool").unwrap();
    let q = [0.2, 0.1];
    let qd = [-0.3, 0.4];
    let qdd = [0.5, -0.2];
    let base = dynibo::BaseState::new(
        Frame::identity(),
        dynibo::Twist::zeros(),
        dynibo::Twist::zeros(),
    )
    .unwrap();
    let mut jacobian = [0.0; 48];
    let mut derivative = [0.0; 48];
    let mut matrix = [0.0; 64];
    let mut output = [0.0; 8];
    let mut forward_output = [0.0; 8];

    reset_allocation_count();
    set_counting(true);
    for _ in 0..10 {
        black_box(robot.forward_kinematics(&base, &q, target).unwrap());
        robot.jacobian(&base, &q, target, &mut jacobian).unwrap();
        robot
            .jacobian_derivative(&base, &q, &qd, target, &mut derivative)
            .unwrap();
        black_box(
            robot
                .forward_velocity_kinematics(&base, &q, &qd, target, &Frame::identity())
                .unwrap(),
        );
        black_box(
            robot
                .forward_acceleration_kinematics(&base, &q, &qd, &qdd, target)
                .unwrap(),
        );
        robot.mass_matrix(&base, &q, &mut matrix).unwrap();
        robot
            .velocity_product_forces(&base, &q, &qd, &mut output)
            .unwrap();
        robot.gravity(&base, &q, &[], &mut output).unwrap();
        robot
            .inverse_dynamics(&base, &q, &qd, &qdd, &[], &mut output)
            .unwrap();
        robot
            .forward_dynamics(&base, &q, &qd, &output, &[], &mut forward_output)
            .unwrap();
        black_box((&jacobian, &derivative, &matrix, &output));
    }
    set_counting(false);
    assert_eq!(allocation_count(), 0);
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

#[test]
fn dynamic_calculations_do_not_allocate_after_robot_creation() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_arm.urdf");
    let mut robot = Robot::from_urdf(path).unwrap();
    let target_id = robot.link_id("test_link_4").unwrap();
    let q = [0.1, 0.8, -0.6, 0.3];
    let qd = [-0.2, 0.4, -0.1, 0.5];
    let qdd = [0.3, -0.2, 0.4, -0.1];
    let initial = [0.0; 4];
    let mut jacobian = [0.0; 24];
    let mut jacobian_derivative = [0.0; 24];
    let mut mass = [0.0; 16];
    let mut velocity_product = [0.0; 4];
    let mut output = [0.0; 4];
    let mut forward_output = [0.0; 4];
    let desired = robot.forward_kinematics(&q, target_id).unwrap();

    robot.jacobian(&q, target_id, &mut jacobian).unwrap();
    robot
        .inverse_kinematics(
            &initial,
            target_id,
            &desired,
            InverseKinematicsOptions::default(),
            &mut output,
        )
        .unwrap();

    reset_allocation_count();
    set_counting(true);
    for _ in 0..10 {
        black_box(robot.forward_kinematics(&q, target_id).unwrap());
        robot.jacobian(&q, target_id, &mut jacobian).unwrap();
        robot.mass_matrix(&q, &mut mass).unwrap();
        robot
            .velocity_product_forces(&q, &qd, &mut velocity_product)
            .unwrap();
        robot
            .jacobian_derivative(&q, &qd, target_id, &mut jacobian_derivative)
            .unwrap();
        black_box(
            robot
                .forward_velocity_kinematics(&q, &qd, target_id, &Frame::identity())
                .unwrap(),
        );
        black_box(
            robot
                .forward_acceleration_kinematics(&q, &qd, &qdd, target_id)
                .unwrap(),
        );
        robot.gravity(&q, &[], &mut output).unwrap();
        robot
            .inverse_dynamics(&q, &qd, &qdd, &[], &mut output)
            .unwrap();
        robot
            .forward_dynamics(&q, &qd, &output, &[], &mut forward_output)
            .unwrap();
        robot
            .inverse_kinematics(
                &initial,
                target_id,
                &desired,
                InverseKinematicsOptions::default(),
                &mut output,
            )
            .unwrap();
        black_box((&jacobian, &output));
    }
    set_counting(false);

    assert_eq!(allocation_count(), 0);
}
