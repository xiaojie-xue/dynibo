#ifndef DYNO_DYNO_H
#define DYNO_DYNO_H

#include <stddef.h>

#if defined(_WIN32) && defined(DYNO_SHARED)
#  if defined(DYNO_BUILDING_LIBRARY)
#    define DYNO_API __declspec(dllexport)
#  else
#    define DYNO_API __declspec(dllimport)
#  endif
#else
#  define DYNO_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DYNO_VERSION_MAJOR 0
#define DYNO_VERSION_MINOR 1
#define DYNO_VERSION_PATCH 0

typedef struct DynoRobot DynoRobot;
typedef struct DynoWorkspace DynoWorkspace;

typedef enum DynoStatus {
    DYNO_STATUS_OK = 0,
    DYNO_STATUS_INVALID_ARGUMENT = 1,
    DYNO_STATUS_MODEL_ERROR = 2,
    DYNO_STATUS_PANIC = 3,
    DYNO_STATUS_SOLVER_ERROR = 4
} DynoStatus;

/* Compatibility alias retained for clients of dyno 0.1.0. */
#define DYNO_STATUS_ERROR DYNO_STATUS_MODEL_ERROR

typedef struct DynoPose {
    double translation[3];
    /* Quaternion order is x, y, z, w. */
    double rotation_xyzw[4];
} DynoPose;

typedef struct DynoTwist {
    double angular[3];
    double linear[3];
} DynoTwist;

typedef struct DynoLoad {
    size_t link_id;
    double torque[3];
    double force[3];
} DynoLoad;

typedef struct DynoIkOptions {
    size_t max_iterations;
    double translation_tolerance;
    double rotation_tolerance;
    double damping;
    double max_step_norm;
} DynoIkOptions;

DYNO_API const char *dyno_version(void);
DYNO_API const char *dyno_last_error_message(void);
DYNO_API DynoIkOptions dyno_ik_options_default(void);

DYNO_API DynoStatus dyno_robot_load_urdf(const char *path, DynoRobot **output);
DYNO_API void dyno_robot_destroy(DynoRobot *robot);
DYNO_API const char *dyno_robot_name(const DynoRobot *robot);
DYNO_API size_t dyno_robot_joint_count(const DynoRobot *robot);
DYNO_API size_t dyno_robot_link_count(const DynoRobot *robot);
DYNO_API DynoStatus dyno_robot_link_id(
    const DynoRobot *robot, const char *name, size_t *output);

DYNO_API DynoStatus dyno_workspace_create(
    const DynoRobot *robot, DynoWorkspace **output);
DYNO_API void dyno_workspace_destroy(DynoWorkspace *workspace);

DYNO_API DynoStatus dyno_forward_kinematics(
    const DynoRobot *robot, DynoWorkspace *workspace,
    const double *q, size_t q_len, size_t target, DynoPose *output);
DYNO_API DynoStatus dyno_jacobian(
    const DynoRobot *robot, DynoWorkspace *workspace,
    const double *q, size_t q_len, size_t target,
    double *output, size_t output_len);
DYNO_API DynoStatus dyno_inverse_kinematics(
    const DynoRobot *robot, DynoWorkspace *workspace,
    const double *initial_q, size_t q_len, size_t target,
    const DynoPose *desired, DynoIkOptions options,
    double *output, size_t output_len);
DYNO_API DynoStatus dyno_forward_velocity(
    const DynoRobot *robot, DynoWorkspace *workspace,
    const double *q, const double *qd, size_t state_len, size_t target,
    const DynoPose *base, const DynoPose *tool, DynoTwist *output);
DYNO_API DynoStatus dyno_forward_acceleration(
    const DynoRobot *robot, DynoWorkspace *workspace,
    const double *q, const double *qd, const double *qdd,
    size_t state_len, size_t target, DynoTwist *output);
DYNO_API DynoStatus dyno_gravity(
    const DynoRobot *robot, DynoWorkspace *workspace,
    const double *q, size_t q_len, const DynoPose *base,
    const DynoLoad *loads, size_t load_count,
    double *output, size_t output_len);
DYNO_API DynoStatus dyno_inverse_dynamics(
    const DynoRobot *robot, DynoWorkspace *workspace,
    const double *q, const double *qd, const double *qdd, size_t state_len,
    const DynoPose *base, DynoTwist base_velocity,
    DynoTwist base_acceleration, const DynoLoad *loads, size_t load_count,
    double *output, size_t output_len);

#ifdef __cplusplus
}
#endif

#endif
