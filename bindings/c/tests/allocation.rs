use std::{
    alloc::{GlobalAlloc, Layout, System},
    cell::Cell,
    ffi::CString,
    path::PathBuf,
    ptr,
};

use dynibo_c::{
    DYNIBO_BASE_FLOATING, DyniboLoad, DyniboPose, DyniboRobot, DyniboStatus, DyniboTwist,
    DyniboWorkspace, dynibo_forward_dynamics, dynibo_forward_velocity_kinematics, dynibo_gravity,
    dynibo_inverse_dynamics, dynibo_robot_destroy, dynibo_robot_from_urdf,
    dynibo_robot_from_urdf_with_base, dynibo_robot_generalized_count, dynibo_robot_joint_count,
    dynibo_robot_link_id, dynibo_robot_set_floating_base_state, dynibo_workspace_create,
    dynibo_workspace_destroy,
};

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
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) }
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        record_allocation();
        unsafe { System.realloc(pointer, layout, new_size) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        record_allocation();
        unsafe { System.alloc_zeroed(layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

#[test]
fn fixed_base_abi_hot_paths_do_not_allocate() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/data/test_arm.urdf");
    let path = CString::new(path.to_string_lossy().as_bytes()).unwrap();
    let mut robot: *mut DyniboRobot = ptr::null_mut();
    let mut workspace: *mut DyniboWorkspace = ptr::null_mut();
    unsafe {
        assert_eq!(
            dynibo_robot_from_urdf(path.as_ptr(), &mut robot),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_workspace_create(robot, &mut workspace),
            DyniboStatus::Ok
        );
        let n = dynibo_robot_joint_count(robot);
        let mut target = 0;
        assert_eq!(
            dynibo_robot_link_id(robot, c"test_link_4".as_ptr(), &mut target),
            DyniboStatus::Ok
        );
        let q = [0.2, 1.0, -0.7, 0.4];
        let qd = [-0.3, 0.5, -0.2, 0.8];
        let qdd = [0.7, -0.4, 0.1, 0.3];
        let tool = DyniboPose::default();
        let load = DyniboLoad {
            link_id: target,
            torque: [0.1, -0.2, 0.3],
            force: [1.0, 0.5, -0.25],
        };
        let mut twist = DyniboTwist::default();
        let mut output = [0.0; 4];

        reset_allocation_count();
        set_counting(true);
        for _ in 0..10 {
            assert_eq!(
                dynibo_forward_velocity_kinematics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    n,
                    target,
                    &tool,
                    &mut twist,
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_gravity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    n,
                    &load,
                    1,
                    output.as_mut_ptr(),
                    output.len(),
                ),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_inverse_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    qdd.as_ptr(),
                    n,
                    &load,
                    1,
                    output.as_mut_ptr(),
                    output.len(),
                ),
                DyniboStatus::Ok
            );
            let generalized_forces = output;
            assert_eq!(
                dynibo_forward_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    n,
                    generalized_forces.as_ptr(),
                    generalized_forces.len(),
                    &load,
                    1,
                    output.as_mut_ptr(),
                    output.len(),
                ),
                DyniboStatus::Ok
            );
        }
        set_counting(false);
        assert_eq!(allocation_count(), 0);

        dynibo_workspace_destroy(workspace);
        dynibo_robot_destroy(robot);
    }
}

#[test]
fn floating_base_abi_forward_dynamics_does_not_allocate() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/data/floating_arm.urdf");
    let path = CString::new(path.to_string_lossy().as_bytes()).unwrap();
    let mut robot: *mut DyniboRobot = ptr::null_mut();
    let mut workspace: *mut DyniboWorkspace = ptr::null_mut();
    unsafe {
        assert_eq!(
            dynibo_robot_from_urdf_with_base(path.as_ptr(), DYNIBO_BASE_FLOATING, &mut robot,),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_workspace_create(robot, &mut workspace),
            DyniboStatus::Ok
        );
        let frame = DyniboPose {
            translation: [0.2, -0.3, 0.4],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        };
        let velocity = DyniboTwist {
            angular: [0.21, -0.17, 0.13],
            linear: [-0.3, 0.2, 0.1],
        };
        let acceleration = DyniboTwist {
            angular: [-0.11, 0.14, 0.09],
            linear: [0.35, -0.22, 0.18],
        };
        assert_eq!(
            dynibo_robot_set_floating_base_state(robot, &frame, velocity, acceleration),
            DyniboStatus::Ok
        );
        let mut target = 0;
        assert_eq!(
            dynibo_robot_link_id(robot, c"tool".as_ptr(), &mut target),
            DyniboStatus::Ok
        );
        let n = dynibo_robot_joint_count(robot);
        let generalized_count = dynibo_robot_generalized_count(robot);
        assert_eq!((n, generalized_count), (2, 8));
        let q = [0.31, -0.27];
        let qd = [-0.24, 0.35];
        let qdd = [0.42, -0.28];
        let load = DyniboLoad {
            link_id: target,
            torque: [-0.13, 0.21, 0.08],
            force: [0.5, -0.4, 0.3],
        };
        let mut generalized_forces = [0.0; 8];
        let mut recovered = [0.0; 8];

        assert_eq!(
            dynibo_inverse_dynamics(
                robot,
                workspace,
                q.as_ptr(),
                qd.as_ptr(),
                qdd.as_ptr(),
                n,
                &load,
                1,
                generalized_forces.as_mut_ptr(),
                generalized_forces.len(),
            ),
            DyniboStatus::Ok
        );

        reset_allocation_count();
        set_counting(true);
        for _ in 0..10 {
            assert_eq!(
                dynibo_forward_dynamics(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    n,
                    generalized_forces.as_ptr(),
                    generalized_forces.len(),
                    &load,
                    1,
                    recovered.as_mut_ptr(),
                    recovered.len(),
                ),
                DyniboStatus::Ok
            );
        }
        set_counting(false);
        assert_eq!(allocation_count(), 0);

        dynibo_workspace_destroy(workspace);
        dynibo_robot_destroy(robot);
    }
}
