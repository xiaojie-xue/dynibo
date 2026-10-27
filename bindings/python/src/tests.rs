use super::{calculation::*, errors::*};
use dynibo::Robot;
use pyo3::prelude::*;
use std::{
    panic::{AssertUnwindSafe, catch_unwind, panic_any},
    path::Path,
    sync::Mutex,
};

type PanicCase = (fn() -> PyResult<()>, &'static str);

fn assert_panic_error(py: Python<'_>, error: PyErr, message: &str) {
    assert!(error.is_instance_of::<PanicError>(py));
    assert_eq!(error.value(py).str().unwrap().to_str().unwrap(), message);
}

#[test]
fn panic_payloads_preserve_messages_and_use_a_fallback_for_other_types() {
    Python::initialize();
    Python::attach(|py| {
        let cases: [PanicCase; 3] = [
            (|| panic_any(String::from("owned panic")), "owned panic"),
            (|| panic!("borrowed panic"), "borrowed panic"),
            (|| panic_any(42_u32), "native dynibo panic"),
        ];
        for (calculate, message) in cases {
            assert_panic_error(py, catch_panic(calculate).unwrap_err(), message);
        }
    });
}

#[test]
fn caught_calculation_panics_leave_the_robot_usable_and_closable() {
    Python::initialize();
    Python::attach(|py| {
        for through_robot in [false, true] {
            let robot = Robot::from_urdf(
                Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/data/test_arm.urdf"),
            )
            .unwrap();
            let target = robot.link_id("test_link_4").unwrap();
            let q = [0.2, -0.3, 0.4, -0.1];
            let mut fresh = robot.fork();
            let expected = fresh.forward_kinematics(&q, target).unwrap();
            let inner = Mutex::new(Some(Calculation {
                robot,
                poses: Vec::new(),
            }));
            let result: PyResult<()> = if through_robot {
                with_robot(&inner, py, |robot| {
                    robot.forward_kinematics(&q, target).unwrap();
                    panic!("calculation panic")
                })
            } else {
                with_calculation(&inner, py, |calculation| {
                    calculation.robot.forward_kinematics(&q, target).unwrap();
                    panic!("calculation panic")
                })
            };
            assert_panic_error(py, result.unwrap_err(), "calculation panic");
            assert!(!inner.is_poisoned());
            let actual = with_robot(&inner, py, |robot| {
                Ok(robot.forward_kinematics(&q, target).unwrap())
            })
            .unwrap();
            assert_eq!(actual, expected);
            close_calculation(&inner, py).unwrap();
            close_calculation(&inner, py).unwrap();
            let error = lock_calculation(&inner, py).err().unwrap();
            assert!(error.is_instance_of::<pyo3::exceptions::PyRuntimeError>(py));
            assert_eq!(
                error.value(py).str().unwrap().to_str().unwrap(),
                "robot is closed"
            );
        }
    });
}

#[test]
fn poisoned_locks_report_python_errors_for_calculation_and_close() {
    let inner = Mutex::new(Some(Calculation {
        robot: (),
        poses: Vec::new(),
    }));
    assert!(
        catch_unwind(AssertUnwindSafe(|| {
            let _guard = inner.lock().unwrap();
            panic!("poison workspace");
        }))
        .is_err()
    );
    assert!(inner.is_poisoned());
    Python::initialize();
    Python::attach(|py| {
        for error in [
            with_robot(&inner, py, |_| -> PyResult<()> {
                panic!("must not execute")
            })
            .unwrap_err(),
            with_calculation(&inner, py, |_| -> PyResult<()> {
                panic!("must not execute")
            })
            .unwrap_err(),
            close_calculation(&inner, py).unwrap_err(),
        ] {
            assert_panic_error(py, error, "robot workspace lock is poisoned");
        }
    });
}
