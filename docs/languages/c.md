# C Guide

The C interface is dynibo's stable ABI. Include `<dynibo/dynibo.h>` and link
against the installed `dynibo::dynibo` CMake target or `dynibo_c` library.

## Naming

C has no namespaces, so every exported function uses the `dynibo_` prefix and
constants use `DYNIBO_`. Opaque object types and value structs use the `Dynibo`
prefix. See [API Mapping](api-mapping.md) for mappings to the other
bindings.

## Ownership

Robot and workspace handles are opaque. A workspace belongs to the robot that
created it and must not be used with another robot. Release both explicitly:

```c
DyniboRobot *robot = NULL;
DyniboWorkspace *workspace = NULL;

/* Create and use the handles... */

dynibo_workspace_destroy(workspace);
dynibo_robot_destroy(robot);
```

Destroy functions accept null. Input and output arrays remain owned by the
caller. Unless a function says otherwise, pointers must be non-null and output
buffers must not overlap inputs.

## Error handling

Fallible functions return `DyniboStatus`. After a failure,
`dynibo_last_error_message()` returns a thread-local message valid until the
next fallible dynibo call on the same thread:

```c
static int check(DyniboStatus status) {
    if (status == DYNIBO_STATUS_OK)
        return 1;
    fprintf(stderr, "dynibo: %s\n", dynibo_last_error_message());
    return 0;
}
```

Copy the message if it must be retained. A successful fallible call clears it.

## Buffers and workspaces

The API validates input and output lengths. Use `dynibo_robot_joint_count()` for
joint-state arrays and `dynibo_robot_generalized_count()` for generalized
outputs. Matrix storage and floating-base ordering are defined in
[Joint and Generalized Coordinates](../user-guide/joint-and-generalized-coordinates.md).

A workspace is mutable. Use a separate workspace for every simultaneous
calculation. Fixed `DyniboRobot` stores its frame; `DyniboFloatingRobot` never
stores base state and instead receives `DyniboBaseState` in every calculation.

## Floating bases

Floating robot and workspace handles are distinct C types:

```c
DyniboFloatingRobot *robot = NULL;
DyniboFloatingWorkspace *workspace = NULL;
check(dynibo_floating_robot_from_urdf("robot.urdf", &robot));
check(dynibo_floating_workspace_create(robot, &workspace));

DyniboBaseState base = {0};
base.frame.rotation_xyzw[3] = 1.0;
size_t target = 0;
check(dynibo_floating_robot_link_id(robot, "tool", &target));
check(dynibo_floating_forward_kinematics(
    robot, workspace, &base, q, joint_count, target, &pose));

dynibo_floating_workspace_destroy(workspace);
dynibo_floating_robot_destroy(robot);
```

Floating `generalized_count` is `joint_count + 6`; generalized outputs begin
with world-frame angular then linear base components.

## ABI and version checks

The header defines `DYNIBO_VERSION_MAJOR`, `DYNIBO_VERSION_MINOR`, and
`DYNIBO_VERSION_PATCH`. `dynibo_version()` reports the linked native library at
runtime. Ship the header and library from the same dynibo release; the runtime
string is useful for diagnostics and for detecting deployment mistakes. The C
ABI does not negotiate incompatible structure layouts at runtime.

## pkg-config and dynamic libraries

In a non-CMake build, use the installed pkg-config metadata:

```bash
cc main.c $(pkg-config --cflags --libs dynibo)
```

The shared library is named `libdynibo_c.so` on Linux,
`libdynibo_c.dylib` on macOS, and `dynibo_c.dll` on Windows. Normal platform
loader rules apply: install the library in a standard search location, embed an
appropriate runtime search path, or deploy it beside the application where the
platform supports that convention.

[Open the C API reference](../c-api/dynibo_8h.md){ .md-button }
