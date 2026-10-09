use super::*;

#[test]
fn typed_workspaces_reject_foreign_models_and_errors_are_thread_local() {
    // SAFETY: All opaque handles below are allocated by this ABI.
    unsafe {
        let (robot, workspace, target) = fixed_handles();
        let (foreign, foreign_workspace, _) = fixed_handles();
        let q = [0.0; 4];
        let mut pose = DyniboPose::default();
        assert_eq!(
            dynibo_forward_kinematics(
                robot,
                foreign_workspace,
                q.as_ptr(),
                q.len(),
                target,
                &mut pose
            ),
            DyniboStatus::InvalidArgument
        );
        assert!(
            !CStr::from_ptr(dynibo_last_error_message())
                .to_bytes()
                .is_empty()
        );
        assert_eq!(
            dynibo_forward_kinematics(robot, workspace, q.as_ptr(), q.len(), target, &mut pose),
            DyniboStatus::Ok
        );
        assert!(
            CStr::from_ptr(dynibo_last_error_message())
                .to_bytes()
                .is_empty()
        );
        dynibo_workspace_destroy(workspace);
        dynibo_robot_destroy(robot);
        dynibo_workspace_destroy(foreign_workspace);
        dynibo_robot_destroy(foreign);
    }

    let barrier = std::sync::Arc::new(std::sync::Barrier::new(2));
    let first_barrier = std::sync::Arc::clone(&barrier);
    let first = std::thread::spawn(move || {
        let mut output = ptr::null_mut();
        assert_eq!(
            unsafe { dynibo_robot_from_urdf(ptr::null(), &mut output) },
            DyniboStatus::InvalidArgument
        );
        first_barrier.wait();
        unsafe { CStr::from_ptr(dynibo_last_error_message()) }
            .to_string_lossy()
            .into_owned()
    });
    let second_barrier = std::sync::Arc::clone(&barrier);
    let second = std::thread::spawn(move || {
        assert_eq!(
            unsafe { dynibo_robot_set_base_frame(ptr::null_mut(), ptr::null()) },
            DyniboStatus::InvalidArgument
        );
        second_barrier.wait();
        unsafe { CStr::from_ptr(dynibo_last_error_message()) }
            .to_string_lossy()
            .into_owned()
    });
    assert!(first.join().unwrap().contains("path must not be null"));
    assert!(second.join().unwrap().contains("robot must not be null"));

    assert_eq!(
        call(|| -> CResult<()> { panic!("test panic") }),
        DyniboStatus::Panic
    );
    assert!(
        unsafe { CStr::from_ptr(dynibo_last_error_message()) }
            .to_string_lossy()
            .contains("panic caught")
    );
}
