# Errors and Thread Safety

Bindings expose the same broad error categories through language-appropriate
mechanisms.

## Error mapping

| Condition | Rust | Python | C++ | C |
|---|---|---|---|---|
| Invalid argument, handle, or length | `Error` with `InvalidInput` category | `ValueError` | `dynibo::Error` | `DYNIBO_STATUS_INVALID_ARGUMENT` |
| URDF/model failure | `Error` with `Model` category | `ModelError` | `dynibo::Error` | `DYNIBO_STATUS_MODEL_ERROR` |
| IK numerical failure/non-convergence | `Error` with `Solver` category | `SolverError` | `dynibo::Error` | `DYNIBO_STATUS_SOLVER_ERROR` |
| Panic caught at ABI boundary | Not applicable to native Rust calls | `PanicError` | `dynibo::Error` | `DYNIBO_STATUS_PANIC` |

Do not branch on human-readable error text. Rust exposes `ErrorCategory`, C has
stable status values, Python has exception types, and C++ `Error::status()`
retains the C status.

## C error messages

`dynibo_last_error_message()` returns a thread-local string. It remains valid
until the next fallible dynibo call on that thread; a successful call clears it.
Copy the string when it must outlive the next call.

## Thread-safety rules

- Immutable model queries may be read when no thread is changing a fixed
  `Robot` base frame.
- Each Rust `Robot` or `FloatingRobot` owns one mutable workspace; calculation
  methods require mutable access. Use `fork()` to obtain an independent
  instance for each concurrent calculation.
- C callers create one typed workspace per concurrent calculation:
  `DyniboWorkspace` for fixed and `DyniboFloatingWorkspace` for floating.
- Python serializes methods on one `Robot` or `FloatingRobot`; separate
  instances enable parallel native calls.
- The C++ wrapper has no internal lock; use one `dynibo::Robot` or
  `dynibo::FloatingRobot` per worker.
- Never destroy or move an object while another thread is using its handles.

## Recovery

Argument and solver errors do not invalidate a robot or workspace. Correct the
input and call again. A caught ABI panic is reported instead of unwinding across
the foreign-language boundary, but it indicates an unexpected internal failure;
record the message and dynibo version before deciding whether to continue.
