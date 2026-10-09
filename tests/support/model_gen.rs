//! Reproducible model fixtures: descriptions, generation, corpus selection, and URDF output.
mod corpus;
mod fixtures;
mod generate;
mod spec;
mod urdf;

#[allow(
    unused_imports,
    reason = "integration-test targets use different parts of this shared facade"
)]
pub use {
    corpus::{
        corpus_model_cases, corpus_model_seeds, randomized_model_cases, selected_model_cases,
    },
    fixtures::{GeneratedModel, generate_case, generate_model},
    generate::{StableRng, generate_spec},
    spec::*,
    urdf::serialize_urdf,
};

/// Bump this whenever a deliberate incompatibility changes the seed-to-model mapping.
pub const GENERATOR_VERSION: u32 = 2;
