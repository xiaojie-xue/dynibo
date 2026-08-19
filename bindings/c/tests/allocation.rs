use std::{
    alloc::{GlobalAlloc, Layout, System},
    ffi::CString,
    path::PathBuf,
    ptr,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

use dynibo_c::{
    DyniboLoad, DyniboPose, DyniboRobot, DyniboStatus, DyniboTwist, DyniboWorkspace,
    dynibo_forward_velocity, dynibo_gravity, dynibo_inverse_dynamics, dynibo_robot_destroy,
    dynibo_robot_joint_count, dynibo_robot_link_id, dynibo_robot_load_urdf,
    dynibo_workspace_create, dynibo_workspace_destroy,
};

struct CountingAllocator;

static COUNTING: AtomicBool = AtomicBool::new(false);
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) }
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.realloc(pointer, layout, new_size) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
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
            dynibo_robot_load_urdf(path.as_ptr(), &mut robot),
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
        let pose = DyniboPose::default();
        let zero = DyniboTwist::default();
        let load = DyniboLoad {
            link_id: target,
            torque: [0.1, -0.2, 0.3],
            force: [1.0, 0.5, -0.25],
        };
        let mut twist = DyniboTwist::default();
        let mut output = [0.0; 4];

        ALLOCATIONS.store(0, Ordering::Relaxed);
        COUNTING.store(true, Ordering::SeqCst);
        for _ in 0..10 {
            assert_eq!(
                dynibo_forward_velocity(
                    robot,
                    workspace,
                    q.as_ptr(),
                    qd.as_ptr(),
                    n,
                    target,
                    &pose,
                    &pose,
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
                    &pose,
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
                    &pose,
                    zero,
                    zero,
                    &load,
                    1,
                    output.as_mut_ptr(),
                    output.len(),
                ),
                DyniboStatus::Ok
            );
        }
        COUNTING.store(false, Ordering::SeqCst);
        assert_eq!(ALLOCATIONS.load(Ordering::Relaxed), 0);

        dynibo_workspace_destroy(workspace);
        dynibo_robot_destroy(robot);
    }
}
