use super::context::TestContext;

#[derive(Clone, Copy, Debug)]
pub struct Tolerance {
    pub absolute: f64,
    pub relative: f64,
}

impl Tolerance {
    pub const fn new(absolute: f64, relative: f64) -> Self {
        Self { absolute, relative }
    }

    fn allowed(self, actual: f64, expected: f64) -> f64 {
        self.absolute + self.relative * actual.abs().max(expected.abs())
    }
}

pub const STRICT: Tolerance = Tolerance::new(2.0e-12, 2.0e-12);
pub const DYNAMICS: Tolerance = Tolerance::new(3.0e-9, 1.0e-9);
pub const KINEMATICS: Tolerance = Tolerance::new(3.0e-9, 1.0e-9);

#[track_caller]
pub fn assert_scalar_close(
    actual: f64,
    expected: f64,
    tolerance: Tolerance,
    context: &TestContext,
) {
    let error = (actual - expected).abs();
    let allowed = tolerance.allowed(actual, expected);
    assert!(
        actual.is_finite() && expected.is_finite() && error <= allowed,
        "numeric mismatch: {context} actual={actual:.17e} expected={expected:.17e} \
         abs_error={error:.17e} allowed={allowed:.17e} atol={:.3e} rtol={:.3e}",
        tolerance.absolute,
        tolerance.relative,
    );
}

#[track_caller]
pub fn assert_slice_close(
    actual: &[f64],
    expected: &[f64],
    tolerance: Tolerance,
    context: &TestContext,
) {
    assert_eq!(
        actual.len(),
        expected.len(),
        "numeric length mismatch: {context}"
    );

    let mut worst: Option<(usize, f64, f64, f64, f64)> = None;
    for (index, (&actual, &expected)) in actual.iter().zip(expected).enumerate() {
        let error = (actual - expected).abs();
        let allowed = tolerance.allowed(actual, expected);
        let normalized = if allowed > 0.0 {
            error / allowed
        } else {
            error
        };
        if worst.is_none_or(|(_, _, _, _, worst_normalized)| normalized > worst_normalized) {
            worst = Some((index, actual, expected, error, normalized));
        }
    }

    if let Some((index, actual, expected, error, _)) = worst {
        let allowed = tolerance.allowed(actual, expected);
        assert!(
            actual.is_finite() && expected.is_finite() && error <= allowed,
            "numeric slice mismatch: {context} index={index} actual={actual:.17e} \
             expected={expected:.17e} abs_error={error:.17e} allowed={allowed:.17e} \
             atol={:.3e} rtol={:.3e}",
            tolerance.absolute,
            tolerance.relative,
        );
    }
}
