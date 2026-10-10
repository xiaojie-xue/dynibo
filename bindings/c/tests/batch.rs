use dynibo_c::*;
use std::{ffi::CString, path::PathBuf, ptr};

#[test]
fn batch_abi_matches_single_queries_and_rejects_invalid_buffers() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/data/test_arm.urdf");
    let path = CString::new(path.to_str().unwrap()).unwrap();
    unsafe {
        let mut fixed = ptr::null_mut();
        let mut fw = ptr::null_mut();
        let mut floating = ptr::null_mut();
        let mut bw = ptr::null_mut();
        assert_eq!(
            dynibo_robot_from_urdf(path.as_ptr(), &mut fixed),
            DyniboStatus::Ok
        );
        assert_eq!(dynibo_workspace_create(fixed, &mut fw), DyniboStatus::Ok);
        assert_eq!(
            dynibo_floating_robot_from_urdf(path.as_ptr(), &mut floating),
            DyniboStatus::Ok
        );
        assert_eq!(
            dynibo_floating_workspace_create(floating, &mut bw),
            DyniboStatus::Ok
        );
        let q = vec![0.1; dynibo_robot_joint_count(fixed)];
        let count = dynibo_robot_link_count(fixed);
        let base = DyniboBaseState {
            frame: DyniboPose {
                translation: [0.2, -0.3, 0.4],
                ..Default::default()
            },
            ..Default::default()
        };
        assert_eq!(
            dynibo_robot_set_base_frame(fixed, &base.frame),
            DyniboStatus::Ok
        );
        for is_floating in [false, true] {
            let mut output = vec![DyniboPose::default(); count];
            let call = |q: *const f64, n, out: *mut DyniboPose, len| {
                if is_floating {
                    dynibo_floating_forward_kinematics_all(floating, bw, &base, q, n, out, len)
                } else {
                    dynibo_forward_kinematics_all(fixed, fw, q, n, out, len)
                }
            };
            assert_eq!(
                call(q.as_ptr(), q.len(), output.as_mut_ptr(), count),
                DyniboStatus::Ok
            );
            for (i, pose) in output.iter().enumerate() {
                let mut single = DyniboPose::default();
                let status = if is_floating {
                    dynibo_floating_forward_kinematics(
                        floating,
                        bw,
                        &base,
                        q.as_ptr(),
                        q.len(),
                        i,
                        &mut single,
                    )
                } else {
                    dynibo_forward_kinematics(fixed, fw, q.as_ptr(), q.len(), i, &mut single)
                };
                assert_eq!(status, DyniboStatus::Ok);
                assert_eq!(pose.translation, single.translation);
                assert_eq!(pose.rotation_xyzw, single.rotation_xyzw);
            }
            let before = output.iter().map(|p| p.translation).collect::<Vec<_>>();
            assert_eq!(
                call(q.as_ptr(), q.len(), output.as_mut_ptr(), count - 1),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                call(q.as_ptr(), q.len() - 1, output.as_mut_ptr(), count),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                call(q.as_ptr(), q.len(), ptr::null_mut(), count),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                call(q.as_ptr(), usize::MAX, output.as_mut_ptr(), count),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                call(output.as_ptr().cast(), q.len(), output.as_mut_ptr(), count),
                DyniboStatus::InvalidArgument
            );
            assert_eq!(
                output.iter().map(|p| p.translation).collect::<Vec<_>>(),
                before
            );
            assert_eq!(
                call(q.as_ptr(), q.len(), output.as_mut_ptr(), count),
                DyniboStatus::Ok
            );
        }
        dynibo_workspace_destroy(fw);
        dynibo_robot_destroy(fixed);
        dynibo_floating_workspace_destroy(bw);
        dynibo_floating_robot_destroy(floating);
    }
}
