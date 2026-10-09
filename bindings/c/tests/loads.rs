use std::{ffi::CString, path::PathBuf, ptr};

use dynibo_c::{
    DyniboLoad, DyniboRobot, DyniboStatus, DyniboWorkspace, dynibo_gravity, dynibo_robot_destroy,
    dynibo_robot_from_urdf, dynibo_robot_joint_count, dynibo_robot_link_id,
    dynibo_workspace_create, dynibo_workspace_destroy,
};

struct Fixture {
    robot: *mut DyniboRobot,
    workspace: *mut DyniboWorkspace,
}

impl Fixture {
    fn tree() -> Self {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("tests/data/test_tree_7.urdf");
        let path = CString::new(path.to_string_lossy().as_bytes()).unwrap();
        let mut robot = ptr::null_mut();
        let mut workspace = ptr::null_mut();
        unsafe {
            assert_eq!(
                dynibo_robot_from_urdf(path.as_ptr(), &mut robot),
                DyniboStatus::Ok
            );
            assert_eq!(
                dynibo_workspace_create(robot, &mut workspace),
                DyniboStatus::Ok
            );
        }
        Self { robot, workspace }
    }

    fn link_id(&self, name: &std::ffi::CStr) -> usize {
        let mut output = 0;
        unsafe {
            assert_eq!(
                dynibo_robot_link_id(self.robot, name.as_ptr(), &mut output),
                DyniboStatus::Ok
            );
        }
        output
    }

    fn gravity(&mut self, q: &[f64], loads: &[DyniboLoad]) -> Vec<f64> {
        let mut output = vec![0.0; unsafe { dynibo_robot_joint_count(self.robot) }];
        let load_pointer = if loads.is_empty() {
            ptr::null()
        } else {
            loads.as_ptr()
        };
        unsafe {
            assert_eq!(
                dynibo_gravity(
                    self.robot,
                    self.workspace,
                    q.as_ptr(),
                    q.len(),
                    load_pointer,
                    loads.len(),
                    output.as_mut_ptr(),
                    output.len(),
                ),
                DyniboStatus::Ok
            );
        }
        output
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        unsafe {
            dynibo_workspace_destroy(self.workspace);
            dynibo_robot_destroy(self.robot);
        }
    }
}

fn assert_close(actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len());
    for (index, (&actual, &expected)) in actual.iter().zip(expected).enumerate() {
        assert!(
            (actual - expected).abs() <= 2.0e-12,
            "mismatch at {index}: actual={actual}, expected={expected}"
        );
    }
}

#[test]
fn load_workspace_aggregates_duplicates_clears_state_and_isolates_links() {
    let mut fixture = Fixture::tree();
    let left = fixture.link_id(c"left_tool");
    let right = fixture.link_id(c"right_tool");
    let q = [0.2, -0.1, 0.35, -0.4, 0.15, 0.3, -0.25];
    let left_a = DyniboLoad {
        link_id: left,
        torque: [0.1, -0.2, 0.3],
        force: [1.0, 0.5, -0.25],
    };
    let left_b = DyniboLoad {
        link_id: left,
        torque: [-0.4, 0.15, 0.2],
        force: [0.3, -0.7, 0.6],
    };
    let left_combined = DyniboLoad {
        link_id: left,
        torque: [-0.3, -0.05, 0.5],
        force: [1.3, -0.2, 0.35],
    };
    let right_load = DyniboLoad {
        link_id: right,
        torque: [0.25, 0.35, -0.1],
        force: [-0.6, 0.2, 0.8],
    };

    let baseline = fixture.gravity(&q, &[]);
    let duplicate = fixture.gravity(&q, &[left_a, left_b]);
    let combined = fixture.gravity(&q, &[left_combined]);
    assert_close(&duplicate, &combined);

    let cleared = fixture.gravity(&q, &[]);
    assert_close(&cleared, &baseline);

    let left_only = fixture.gravity(&q, &[left_a]);
    let right_only = fixture.gravity(&q, &[right_load]);
    let both = fixture.gravity(&q, &[left_a, right_load]);
    let expected: Vec<f64> = baseline
        .iter()
        .zip(&left_only)
        .zip(&right_only)
        .map(|((baseline, left), right)| left + right - baseline)
        .collect();
    assert_close(&both, &expected);
}

#[test]
fn invalid_loads_and_overflow_preserve_output_and_reset_aggregation() {
    let mut fixture = Fixture::tree();
    let id = fixture.link_id(c"left_tool");
    let n = unsafe { dynibo_robot_joint_count(fixture.robot) };
    let q = vec![0.0; n];
    let expected = fixture.gravity(&q, &[]);
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::MAX] {
        let load = DyniboLoad {
            link_id: id,
            torque: [value; 3],
            force: [0.0; 3],
        };
        let loads = [
            DyniboLoad {
                link_id: id,
                ..Default::default()
            },
            load,
            load,
        ];
        let mut out = vec![123.0; n];
        let status = unsafe {
            dynibo_gravity(
                fixture.robot,
                fixture.workspace,
                q.as_ptr(),
                n,
                loads.as_ptr(),
                loads.len(),
                out.as_mut_ptr(),
                n,
            )
        };
        assert_eq!(
            status,
            if value.is_finite() {
                DyniboStatus::SolverError
            } else {
                DyniboStatus::InvalidArgument
            }
        );
        assert!(out.iter().all(|v| *v == 123.0));
        assert_close(&fixture.gravity(&q, &[]), &expected);
        let valid = DyniboLoad {
            link_id: id,
            torque: [0.1; 3],
            force: [0.2; 3],
        };
        let single = fixture.gravity(&q, &[valid]);
        let duplicate = fixture.gravity(&q, &[valid, valid]);
        let doubled: Vec<_> = single
            .iter()
            .zip(&expected)
            .map(|(v, b)| 2.0 * v - b)
            .collect();
        assert_close(&duplicate, &doubled);
    }
}

#[test]
fn compact_load_workspace_handles_more_inputs_than_links() {
    let mut fixture = Fixture::tree();
    let left = fixture.link_id(c"left_tool");
    let q = [0.2, -0.1, 0.35, -0.4, 0.15, 0.3, -0.25];
    let repeated = DyniboLoad {
        link_id: left,
        torque: [0.125, -0.25, 0.5],
        force: [1.0, -0.5, 0.25],
    };
    let combined = DyniboLoad {
        link_id: left,
        torque: [4.0, -8.0, 16.0],
        force: [32.0, -16.0, 8.0],
    };

    let duplicate_inputs = vec![repeated; 32];
    let duplicate_output = fixture.gravity(&q, &duplicate_inputs);
    let combined_output = fixture.gravity(&q, &[combined]);
    assert_close(&duplicate_output, &combined_output);

    let baseline = fixture.gravity(&q, &[]);
    let cleared = fixture.gravity(&q, &[]);
    assert_close(&cleared, &baseline);
}
