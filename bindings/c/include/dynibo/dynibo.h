#ifndef DYNIBO_DYNIBO_H
#define DYNIBO_DYNIBO_H

#include <stddef.h>

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

#define DYNIBO_VERSION_MAJOR 0
#define DYNIBO_VERSION_MINOR 1
#define DYNIBO_VERSION_PATCH 0

typedef struct DyniboRobot DyniboRobot;
typedef struct DyniboWorkspace DyniboWorkspace;

typedef enum DyniboStatus {
    DYNIBO_STATUS_OK = 0,
    DYNIBO_STATUS_INVALID_ARGUMENT = 1,
    DYNIBO_STATUS_MODEL_ERROR = 2,
    DYNIBO_STATUS_PANIC = 3,
    DYNIBO_STATUS_SOLVER_ERROR = 4
} DyniboStatus;

/* Compatibility alias retained for clients of dynibo 0.1.0. */
#define DYNIBO_STATUS_ERROR DYNIBO_STATUS_MODEL_ERROR

typedef struct DyniboPose {
    double translation[3];
    /* Quaternion order is x, y, z, w. */
    double rotation_xyzw[4];
} DyniboPose;

typedef struct DyniboTwist {
    double angular[3];
    double linear[3];
} DyniboTwist;

typedef struct DyniboLoad {
    size_t link_id;
    double torque[3];
    double force[3];
} DyniboLoad;

typedef struct DyniboIkOptions {
    size_t max_iterations;
    double translation_tolerance;
    double rotation_tolerance;
    double damping;
    double max_step_norm;
} DyniboIkOptions;

DYNIBO_API const char *dynibo_version(void);
DYNIBO_API const char *dynibo_last_error_message(void);
DYNIBO_API DyniboIkOptions dynibo_ik_options_default(void);

DYNIBO_API DyniboStatus dynibo_robot_load_urdf(const char *path, DyniboRobot **output);
DYNIBO_API void dynibo_robot_destroy(DyniboRobot *robot);
DYNIBO_API const char *dynibo_robot_name(const DyniboRobot *robot);
DYNIBO_API size_t dynibo_robot_joint_count(const DyniboRobot *robot);
DYNIBO_API size_t dynibo_robot_link_count(const DyniboRobot *robot);
DYNIBO_API DyniboStatus dynibo_robot_link_id(
    const DyniboRobot *robot, const char *name, size_t *output);

DYNIBO_API DyniboStatus dynibo_workspace_create(
    const DyniboRobot *robot, DyniboWorkspace **output);
DYNIBO_API void dynibo_workspace_destroy(DyniboWorkspace *workspace);

DYNIBO_API DyniboStatus dynibo_forward_kinematics(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, size_t q_len, size_t target, DyniboPose *output);
DYNIBO_API DyniboStatus dynibo_jacobian(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, size_t q_len, size_t target,
    double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_inverse_kinematics(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *initial_q, size_t q_len, size_t target,
    const DyniboPose *desired, DyniboIkOptions options,
    double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_forward_velocity(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, const double *qd, size_t state_len, size_t target,
    const DyniboPose *base, const DyniboPose *tool, DyniboTwist *output);
DYNIBO_API DyniboStatus dynibo_forward_acceleration(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, const double *qd, const double *qdd,
    size_t state_len, size_t target, DyniboTwist *output);
DYNIBO_API DyniboStatus dynibo_gravity(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, size_t q_len, const DyniboPose *base,
    const DyniboLoad *loads, size_t load_count,
    double *output, size_t output_len);
DYNIBO_API DyniboStatus dynibo_inverse_dynamics(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, const double *qd, const double *qdd, size_t state_len,
    const DyniboPose *base, DyniboTwist base_velocity,
    DyniboTwist base_acceleration, const DyniboLoad *loads, size_t load_count,
    double *output, size_t output_len);

#ifdef __cplusplus
}
#endif

#endif
