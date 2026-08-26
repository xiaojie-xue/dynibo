use std::fmt;

use dynibo::BaseMode;

#[derive(Clone, Debug)]
pub struct TestContext {
    pub operation: String,
    pub fixture: String,
    pub seed: Option<u64>,
    pub sample: usize,
    pub base_mode: BaseMode,
    pub target: Option<String>,
    pub load_case: Option<String>,
    pub step: Option<usize>,
}

impl TestContext {
    pub fn new(operation: impl Into<String>, fixture: impl Into<String>) -> Self {
        Self {
            operation: operation.into(),
            fixture: fixture.into(),
            seed: None,
            sample: 0,
            base_mode: BaseMode::Fixed,
            target: None,
            load_case: None,
            step: None,
        }
    }

    pub fn seed(mut self, seed: u64) -> Self {
        self.seed = Some(seed);
        self
    }

    pub fn sample(mut self, sample: usize) -> Self {
        self.sample = sample;
        self
    }

    pub fn base_mode(mut self, base_mode: BaseMode) -> Self {
        self.base_mode = base_mode;
        self
    }

    pub fn target(mut self, target: impl Into<String>) -> Self {
        self.target = Some(target.into());
        self
    }

    pub fn load_case(mut self, load_case: impl Into<String>) -> Self {
        self.load_case = Some(load_case.into());
        self
    }

    pub fn step(mut self, step: usize) -> Self {
        self.step = Some(step);
        self
    }
}

impl fmt::Display for TestContext {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "operation={} fixture={} sample={} base={:?}",
            self.operation, self.fixture, self.sample, self.base_mode
        )?;
        if let Some(seed) = self.seed {
            write!(formatter, " seed={seed}")?;
        }
        if let Some(target) = &self.target {
            write!(formatter, " target={target}")?;
        }
        if let Some(load_case) = &self.load_case {
            write!(formatter, " loads={load_case}")?;
        }
        if let Some(step) = self.step {
            write!(formatter, " step={step}")?;
        }
        Ok(())
    }
}
