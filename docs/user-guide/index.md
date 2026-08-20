# User Guide

This guide describes the model and numerical conventions shared by every
dynibo binding. Read it independently of the language-specific API reference:
the mathematics, ordering, units, and lifetime rules are the same in Rust,
Python, C++, and C.

## Recommended path

1. [Robot Model and URDF](robot-model-and-urdf.md) explains what dynibo loads.
2. [Joint and Generalized Coordinates](joint-and-generalized-coordinates.md)
   defines input and output dimensions.
3. [Frames and Spatial Vectors](frames-and-spatial-vectors.md) defines poses,
   twists, wrenches, and matrix layout.
4. [Fixed and Floating Bases](fixed-and-floating-bases.md) explains base state.
5. [Workspaces and Allocation](workspaces-and-allocation.md) covers reuse and
   concurrency.
6. [Kinematics](kinematics.md) and [Dynamics](dynamics.md) describe calculations.

Read [External Loads](external-loads.md) before applying forces, and [Errors and
Thread Safety](errors-and-thread-safety.md) before integrating dynibo into a
long-running or concurrent application.

Language spelling and ownership differences are summarized in [API
Mapping](../languages/api-mapping.md).
