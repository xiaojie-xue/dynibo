#ifndef DYNIBO_DYNIBO_H
#define DYNIBO_DYNIBO_H

/**
 * @file dynibo.h
 * @brief Stable C API for fixed- and floating-base dynibo models.
 *
 * A fixed `DyniboRobot` and a floating `DyniboFloatingRobot` are different,
 * incompatible types. Their workspaces are likewise incompatible. A workspace
 * is mutable, belongs only to the model that created it, and must not be used
 * concurrently. Create one workspace per simultaneous calculation.
 *
 * Let `J` be a robot's joint count and `G` its generalized count. Fixed-base
 * robots have `G == J`; floating-base robots have `G == J + 6`. `q`, `qd`, and
 * `qdd` always contain exactly `J` non-fixed URDF joint values. Floating
 * generalized vectors prepend world-frame angular then linear base values.
 * Matrices are column-major. Jacobians are `6 x G`; mass matrices are `G x G`.
 * Spatial values use angular-before-linear ordering.
 *
 * Unless explicitly documented otherwise, all pointer arguments are non-null.
 * Input arrays have their documented exact length, and numeric output buffers
 * must not overlap any numeric input buffer. A failed call never creates a
 * borrowed output. Invalid arguments return `DYNIBO_STATUS_INVALID_ARGUMENT`.
 */

#include <stddef.h>
#include "version.h"

#if defined(_WIN32) && defined(DYNIBO_SHARED)
#  if defined(DYNIBO_BUILDING_LIBRARY)
#    define DYNIBO_API __declspec(dllexport)
#  else
#    define DYNIBO_API __declspec(dllimport)
#  endif
#else
#  define DYNIBO_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DyniboRobot DyniboRobot;
typedef struct DyniboWorkspace DyniboWorkspace;
typedef struct DyniboFloatingRobot DyniboFloatingRobot;
typedef struct DyniboFloatingWorkspace DyniboFloatingWorkspace;

/** Status returned by fallible ABI functions. Details are in the thread-local error string. */
typedef enum DyniboStatus {
    DYNIBO_STATUS_OK = 0,
    DYNIBO_STATUS_INVALID_ARGUMENT = 1,
    DYNIBO_STATUS_MODEL_ERROR = 2,
    DYNIBO_STATUS_PANIC = 3,
    DYNIBO_STATUS_SOLVER_ERROR = 4
} DyniboStatus;
#define DYNIBO_STATUS_ERROR DYNIBO_STATUS_MODEL_ERROR

/** Translation in metres plus a unit `(x, y, z, w)` quaternion. */
typedef struct DyniboPose { double translation[3]; double rotation_xyzw[4]; } DyniboPose;
/** World-frame angular-first spatial velocity or acceleration. */
typedef struct DyniboTwist { double angular[3]; double linear[3]; } DyniboTwist;
/** Explicit floating-root pose, velocity, and acceleration for one calculation. */
typedef struct DyniboBaseState { DyniboPose frame; DyniboTwist velocity; DyniboTwist acceleration; } DyniboBaseState;
/** Resisting wrench at a link origin, expressed in that link frame. */
typedef struct DyniboLoad { size_t link_id; double torque[3]; double force[3]; } DyniboLoad;
/** Damped-least-squares inverse-kinematics configuration. */
typedef struct DyniboIkOptions { size_t max_iterations; double translation_tolerance; double rotation_tolerance; double damping; double max_step_norm; } DyniboIkOptions;

/** Returns the linked ABI version; the static string must not be freed. */
DYNIBO_API const char *dynibo_version(void);
/** Returns the calling thread's latest fallible-call error; valid until its next fallible call. */
DYNIBO_API const char *dynibo_last_error_message(void);
DYNIBO_API DyniboIkOptions dynibo_ik_options_default(void);

/** @name Fixed-base model lifetime and metadata */
/**@{*/
/** Loads a fixed-base URDF and writes an owned handle to `output` (null on failure). */
DYNIBO_API DyniboStatus dynibo_robot_from_urdf(const char *path, DyniboRobot **output);
/** Destroys a fixed robot; null is accepted. */
DYNIBO_API void dynibo_robot_destroy(DyniboRobot *robot);
/** Returns a borrowed name, or null for a null handle. */
DYNIBO_API const char *dynibo_robot_name(const DyniboRobot *robot);
DYNIBO_API size_t dynibo_robot_joint_count(const DyniboRobot *robot);
DYNIBO_API size_t dynibo_robot_generalized_count(const DyniboRobot *robot);
DYNIBO_API size_t dynibo_robot_link_count(const DyniboRobot *robot);
/** Resolves `name` to a link ID valid only for `robot`. */
DYNIBO_API DyniboStatus dynibo_robot_link_id(const DyniboRobot *robot, const char *name, size_t *output);
/** Stores the fixed root world frame used by later calculations. */
DYNIBO_API DyniboStatus dynibo_robot_set_base_frame(DyniboRobot *robot, const DyniboPose *frame);
/** Allocates a workspace belonging to `robot`. */
DYNIBO_API DyniboStatus dynibo_workspace_create(const DyniboRobot *robot, DyniboWorkspace **output);
/** Destroys a fixed workspace; null is accepted. */
DYNIBO_API void dynibo_workspace_destroy(DyniboWorkspace *workspace);
/**@}*/

/** @name Floating-base model lifetime and metadata */
/**@{*/
/** Loads a floating-base URDF. The returned handle never stores a base state. */
DYNIBO_API DyniboStatus dynibo_floating_robot_from_urdf(const char *path, DyniboFloatingRobot **output);
DYNIBO_API void dynibo_floating_robot_destroy(DyniboFloatingRobot *robot);
DYNIBO_API const char *dynibo_floating_robot_name(const DyniboFloatingRobot *robot);
DYNIBO_API size_t dynibo_floating_robot_joint_count(const DyniboFloatingRobot *robot);
DYNIBO_API size_t dynibo_floating_robot_generalized_count(const DyniboFloatingRobot *robot);
DYNIBO_API size_t dynibo_floating_robot_link_count(const DyniboFloatingRobot *robot);
DYNIBO_API DyniboStatus dynibo_floating_robot_link_id(const DyniboFloatingRobot *robot, const char *name, size_t *output);
DYNIBO_API DyniboStatus dynibo_floating_workspace_create(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace **output);
DYNIBO_API void dynibo_floating_workspace_destroy(DyniboFloatingWorkspace *workspace);
/**@}*/

/**
 * @name Calculations
 * Fixed functions take `(robot, workspace, ...)`; floating functions take
 * `(robot, workspace, base, ...)`. `base` is validated for a finite pose,
 * non-zero quaternion, and finite motion. `target` and every load ID must be
 * obtained from the matching robot. `loads` may be null only when `load_count`
 * is zero. `tool` is a finite target-relative pose.
 * Position-only functions require `q_len == J`; state functions require
 * `state_len == J`. Jacobian and derivative output lengths are `6 * G`, mass
 * matrix output lengths are `G * G`, and dynamics/force output lengths are
 * `G`. Fixed-base IK takes and writes exactly `J` values. Pose and twist
 * output pointers each designate one complete value.
 */
/**@{*/
DYNIBO_API DyniboStatus dynibo_forward_kinematics(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, size_t q_len, size_t target, DyniboPose *output);
DYNIBO_API DyniboStatus dynibo_forward_velocity_kinematics(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, const double *qd, size_t state_len, size_t target, const DyniboPose *tool, DyniboTwist *output);
DYNIBO_API DyniboStatus dynibo_forward_acceleration_kinematics(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, const double *qd, const double *qdd, size_t state_len, size_t target, DyniboTwist *output);
/** Writes a world-frame, target-origin, column-major `6 x G` Jacobian. */
DYNIBO_API DyniboStatus dynibo_jacobian(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, size_t q_len, size_t target, double *output, size_t output_len);
/** Writes a column-major `6 x G` Jacobian derivative. */
DYNIBO_API DyniboStatus dynibo_jacobian_derivative(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, const double *qd, size_t state_len, size_t target, double *output, size_t output_len);
/** Writes a column-major `G x G` mass matrix. */
DYNIBO_API DyniboStatus dynibo_mass_matrix(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, size_t q_len, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_velocity_product_forces(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, const double *qd, size_t state_len, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_gravity(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, size_t q_len, const DyniboLoad *loads, size_t load_count, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_inverse_dynamics(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, const double *qd, const double *qdd, size_t state_len, const DyniboLoad *loads, size_t load_count, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_forward_dynamics(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *q, const double *qd, size_t state_len, const double *generalized_forces, size_t generalized_force_len, const DyniboLoad *loads, size_t load_count, double *output, size_t output_len);
/** Fixed-base only; writes `J` joint values. */
DYNIBO_API DyniboStatus dynibo_inverse_kinematics(const DyniboRobot *robot, DyniboWorkspace *workspace, const double *initial_q, size_t q_len, size_t target, const DyniboPose *desired, DyniboIkOptions options, double *output, size_t output_len);

DYNIBO_API DyniboStatus dynibo_floating_forward_kinematics(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, size_t q_len, size_t target, DyniboPose *output);
DYNIBO_API DyniboStatus dynibo_floating_forward_velocity_kinematics(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, const double *qd, size_t state_len, size_t target, const DyniboPose *tool, DyniboTwist *output);
DYNIBO_API DyniboStatus dynibo_floating_forward_acceleration_kinematics(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, const double *qd, const double *qdd, size_t state_len, size_t target, DyniboTwist *output);
DYNIBO_API DyniboStatus dynibo_floating_jacobian(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, size_t q_len, size_t target, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_floating_jacobian_derivative(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, const double *qd, size_t state_len, size_t target, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_floating_mass_matrix(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, size_t q_len, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_floating_velocity_product_forces(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, const double *qd, size_t state_len, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_floating_gravity(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, size_t q_len, const DyniboLoad *loads, size_t load_count, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_floating_inverse_dynamics(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, const double *qd, const double *qdd, size_t state_len, const DyniboLoad *loads, size_t load_count, double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_floating_forward_dynamics(const DyniboFloatingRobot *robot, DyniboFloatingWorkspace *workspace, const DyniboBaseState *base, const double *q, const double *qd, size_t state_len, const double *generalized_forces, size_t generalized_force_len, const DyniboLoad *loads, size_t load_count, double *output, size_t output_len);
/**@}*/

#ifdef __cplusplus
}
#endif
#endif
