use super::{Case, OPERATIONS, assert_close};
use std::{
    ffi::{CString, c_char, c_void},
    hint::black_box,
    path::PathBuf,
    ptr::NonNull,
};

unsafe extern "C" {
    fn dynibo_pinocchio_create_for_frame(path: *const c_char, frame: *const c_char) -> *mut c_void;
    fn dynibo_pinocchio_create_floating_for_frame(
        path: *const c_char,
        frame: *const c_char,
    ) -> *mut c_void;
    fn dynibo_pinocchio_destroy(context: *mut c_void);
    fn dynibo_pinocchio_dof(context: *const c_void) -> usize;
    fn dynibo_pinocchio_configuration_size(context: *const c_void) -> usize;
    fn dynibo_pinocchio_neutral_configuration(context: *const c_void, q: *mut f64);
    fn dynibo_pinocchio_joint_configuration_index(
        context: *const c_void,
        name: *const c_char,
    ) -> usize;
    fn dynibo_pinocchio_joint_velocity_index(context: *const c_void, name: *const c_char) -> usize;
    fn dynibo_pinocchio_link_jacobian_values(context: *mut c_void, q: *const f64, out: *mut f64);
    fn dynibo_pinocchio_rnea_values(
        context: *mut c_void,
        q: *const f64,
        v: *const f64,
        a: *const f64,
        out: *mut f64,
    );
    fn dynibo_pinocchio_aba_values(
        context: *mut c_void,
        q: *const f64,
        v: *const f64,
        tau: *const f64,
        out: *mut f64,
    );
}

pub struct Pinocchio {
    context: NonNull<c_void>,
    q: Vec<f64>,
    v: Vec<f64>,
    a: Vec<f64>,
    tau: Vec<f64>,
    // Maps Dynibo angular-first world coordinates to Pinocchio local linear-first.
    // The benchmark base orientation is identity, so only permutation is needed.
    columns: Vec<usize>,
}

impl Drop for Pinocchio {
    fn drop(&mut self) {
        // SAFETY: this context is owned by self and destroyed exactly once.
        unsafe {
            dynibo_pinocchio_destroy(self.context.as_ptr());
        }
    }
}

impl Pinocchio {
    pub fn new(case: &Case) -> Self {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(case.model.path);
        let path = CString::new(path.to_str().unwrap()).unwrap();
        let frame = CString::new(case.model.target).unwrap();
        // SAFETY: both strings are NUL-terminated and valid during the calls.
        let context = NonNull::new(unsafe {
            if case.model.floating {
                dynibo_pinocchio_create_floating_for_frame(path.as_ptr(), frame.as_ptr())
            } else {
                dynibo_pinocchio_create_for_frame(path.as_ptr(), frame.as_ptr())
            }
        })
        .expect("Pinocchio failed to load benchmark model");
        // SAFETY: context has just been created and is nonnull.
        let (nq, nv) = unsafe {
            (
                dynibo_pinocchio_configuration_size(context.as_ptr()),
                dynibo_pinocchio_dof(context.as_ptr()),
            )
        };
        assert_eq!(nv, case.g());
        let mut result = Self {
            context,
            q: vec![0.0; nq],
            v: vec![0.0; nv],
            a: vec![0.0; nv],
            tau: vec![0.0; nv],
            columns: Vec::new(),
        };
        // SAFETY: q has the model's configuration size.
        unsafe {
            dynibo_pinocchio_neutral_configuration(context.as_ptr(), result.q.as_mut_ptr());
        }
        if case.model.floating {
            result.q[..3].copy_from_slice(case.base.frame().translation.vector.as_slice());
            // Identity base orientation, nonzero base velocity and acceleration.
            let velocity = case.base.velocity();
            let acceleration = case.base.acceleration();
            result.v[..3].copy_from_slice(velocity.linear.as_slice());
            result.v[3..6].copy_from_slice(velocity.angular.as_slice());
            let spatial_linear = acceleration.linear - velocity.angular.cross(&velocity.linear);
            result.a[..3].copy_from_slice(spatial_linear.as_slice());
            result.a[3..6].copy_from_slice(acceleration.angular.as_slice());
            result.columns.extend_from_slice(&[3, 4, 5, 0, 1, 2]);
        }
        // G1 is branched: never assume that two libraries enumerate joints identically.
        for (i, name) in case.names.iter().enumerate() {
            let name = CString::new(name.as_str()).unwrap();
            // SAFETY: the context and the NUL-terminated name are valid.
            let (qi, vi) = unsafe {
                (
                    dynibo_pinocchio_joint_configuration_index(context.as_ptr(), name.as_ptr()),
                    dynibo_pinocchio_joint_velocity_index(context.as_ptr(), name.as_ptr()),
                )
            };
            assert!(qi < nq && vi < nv, "joint missing from Pinocchio");
            result.q[qi] = case.q[i];
            result.v[vi] = case.qd[i];
            result.a[vi] = case.qdd[i];
            result.columns.push(vi);
        }
        for (i, &column) in result.columns.iter().enumerate() {
            result.tau[column] = case.forces[i];
        }
        result
    }

    pub fn calculate(&mut self, operation: &str, output: &mut [f64]) {
        assert_eq!(
            output.len(),
            self.v.len() * if operation == "jacobian" { 6 } else { 1 }
        );
        // SAFETY: all vectors have the validated model sizes, and context is owned.
        // The bridge writes full outputs into caller-owned buffers, without reductions.
        unsafe {
            let context = self.context.as_ptr();
            let q = black_box(self.q.as_ptr());
            let out = black_box(output.as_mut_ptr());
            match operation {
                "jacobian" => dynibo_pinocchio_link_jacobian_values(context, q, out),
                "rnea" => dynibo_pinocchio_rnea_values(
                    context,
                    q,
                    black_box(self.v.as_ptr()),
                    black_box(self.a.as_ptr()),
                    out,
                ),
                "aba" => dynibo_pinocchio_aba_values(
                    context,
                    q,
                    black_box(self.v.as_ptr()),
                    black_box(self.tau.as_ptr()),
                    out,
                ),
                _ => unreachable!(),
            }
        }
    }

    pub fn validate(&mut self, case: &mut Case) {
        for operation in OPERATIONS {
            let size = case.g() * if operation == "jacobian" { 6 } else { 1 };
            let mut actual = vec![0.0; size];
            let mut reference = vec![0.0; size];
            let mut converted = vec![0.0; size];
            case.calculate(operation, &mut actual);
            self.calculate(operation, &mut reference);
            if operation == "jacobian" {
                for (column, &pin_column) in self.columns.iter().enumerate() {
                    for row in 0..6 {
                        converted[6 * column + row] = reference[6 * pin_column + (row + 3) % 6];
                    }
                }
            } else {
                for (i, &column) in self.columns.iter().enumerate() {
                    converted[i] = reference[column];
                }
                if operation == "aba" && case.model.floating {
                    // Pinocchio returns spatial acceleration; Dynibo returns classical.
                    let velocity = case.base.velocity();
                    let correction = velocity.angular.cross(&velocity.linear);
                    for i in 0..3 {
                        converted[3 + i] += correction[i];
                    }
                }
            }
            let error = assert_close(&actual, &converted, operation);
            eprintln!(
                "validated {}/{operation}: max_abs_error={error:.3e}",
                case.model.name
            );
        }
    }
}
