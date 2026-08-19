#ifndef DYNIBO_DYNIBO_H
#define DYNIBO_DYNIBO_H

/**
 * @file dynibo.h
 * @brief Stable C API for dynibo robot kinematics and dynamics.
 *
 * Dynibo uses opaque robot and workspace handles. Unless stated otherwise,
 * pointer arguments must be non-null, input arrays must contain exactly the
 * documented number of elements, and output arrays must not overlap inputs.
 *
 * Let `J = dynibo_robot_joint_count(robot)` and
 * `G = dynibo_robot_generalized_count(robot)`. Joint-state inputs (`q`, `qd`,
 * and `qdd`) contain `J` elements in non-fixed URDF joint order. For a fixed
 * base `G == J`; for a floating base `G == J + 6`.
 *
 * A workspace is reusable but mutable. Use a separate workspace for each
 * simultaneous calculation and only with the robot from which it was created.
 */

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

/** Compile-time major version. */
#define DYNIBO_VERSION_MAJOR 0
/** Compile-time minor version. */
#define DYNIBO_VERSION_MINOR 2
/** Compile-time patch version. */
#define DYNIBO_VERSION_PATCH 0

/** @brief Opaque robot model handle. */
typedef struct DyniboRobot DyniboRobot;
/** @brief Opaque reusable calculation workspace. */
typedef struct DyniboWorkspace DyniboWorkspace;

/** @brief Status returned by fallible C API functions. */
typedef enum DyniboStatus {
    DYNIBO_STATUS_OK = 0,               /**< The operation succeeded. */
    DYNIBO_STATUS_INVALID_ARGUMENT = 1, /**< An argument or handle was invalid. */
    DYNIBO_STATUS_MODEL_ERROR = 2,      /**< A robot model could not be loaded. */
    DYNIBO_STATUS_PANIC = 3,            /**< A panic was caught at the ABI boundary. */
    DYNIBO_STATUS_SOLVER_ERROR = 4      /**< A numerical solver failed. */
} DyniboStatus;

/** @brief Validated integer selecting the root-link connection mode. */
typedef int DyniboBaseMode;
enum {
    DYNIBO_BASE_FIXED = 0,    /**< The root link is fixed to the world. */
    DYNIBO_BASE_FLOATING = 1  /**< The root link has six velocity coordinates. */
};

/** Compatibility alias retained for clients of dynibo 0.1.0. */
#define DYNIBO_STATUS_ERROR DYNIBO_STATUS_MODEL_ERROR

/** @brief Translation plus a unit quaternion. */
typedef struct DyniboPose {
    double translation[3];   /**< Translation in metres. */
    double rotation_xyzw[4]; /**< Unit quaternion ordered x, y, z, w. */
} DyniboPose;

/** @brief Angular-first spatial motion vector. */
typedef struct DyniboTwist {
    double angular[3]; /**< Angular component. */
    double linear[3];  /**< Linear component. */
} DyniboTwist;

/** @brief External wrench applied at a link origin in that link's frame. */
typedef struct DyniboLoad {
    size_t link_id;  /**< ID returned by dynibo_robot_link_id(). */
    double torque[3]; /**< Torque component. */
    double force[3];  /**< Force component. */
} DyniboLoad;

/** @brief Damped-least-squares inverse-kinematics options. */
typedef struct DyniboIkOptions {
    size_t max_iterations;        /**< Maximum number of solver updates. */
    double translation_tolerance; /**< Translation tolerance in metres. */
    double rotation_tolerance;    /**< Rotation-vector tolerance in radians. */
    double damping;               /**< Damped-least-squares damping factor. */
    double max_step_norm;         /**< Maximum norm of one joint update. */
} DyniboIkOptions;

/**
 * @brief Returns the linked dynibo ABI version.
 * @return A static null-terminated string that must not be freed.
 */
DYNIBO_API const char *dynibo_version(void);

/**
 * @brief Returns the last error message for the calling thread.
 *
 * The returned pointer remains valid until the next fallible dynibo call on
 * the same thread. A successful fallible call clears the message. Copy the
 * string if it must be retained.
 *
 * @return A thread-local null-terminated string that must not be freed.
 */
DYNIBO_API const char *dynibo_last_error_message(void);

/**
 * @brief Returns the default inverse-kinematics options.
 * @return A complete options value suitable for dynibo_inverse_kinematics().
 */
DYNIBO_API DyniboIkOptions dynibo_ik_options_default(void);

/**
 * @brief Loads a fixed-base robot from a URDF file.
 * @param[in] path Null-terminated UTF-8 path to the URDF file.
 * @param[out] output Receives the newly allocated robot handle. It is set to
 * null when loading fails.
 * @return DYNIBO_STATUS_OK on success, DYNIBO_STATUS_INVALID_ARGUMENT for an
 * invalid argument, or DYNIBO_STATUS_MODEL_ERROR when the model cannot be
 * loaded or represented.
 * @see dynibo_robot_destroy
 */
DYNIBO_API DyniboStatus dynibo_robot_from_urdf(const char *path, DyniboRobot **output);

/**
 * @brief Loads a robot from a URDF file with an explicit base mode.
 * @param[in] path Null-terminated UTF-8 path to the URDF file.
 * @param[in] base_mode Fixed or floating root-link mode.
 * @param[out] output Receives the newly allocated robot handle. It is set to
 * null when loading fails.
 * @return DYNIBO_STATUS_OK on success, DYNIBO_STATUS_INVALID_ARGUMENT for an
 * invalid argument, or DYNIBO_STATUS_MODEL_ERROR when the model cannot be
 * loaded or represented.
 * @see dynibo_robot_destroy
 */
DYNIBO_API DyniboStatus dynibo_robot_from_urdf_with_base(
    const char *path, DyniboBaseMode base_mode, DyniboRobot **output);

/**
 * @brief Destroys a robot handle.
 * @param[in] robot Handle returned by a robot-loading function, or null.
 * @note Passing null is allowed. The handle must not be used after this call.
 */
DYNIBO_API void dynibo_robot_destroy(DyniboRobot *robot);

/**
 * @brief Returns the URDF robot name.
 * @param[in] robot Robot handle.
 * @return A null-terminated string valid until the robot is destroyed, or
 * null when @p robot is null. The string must not be freed.
 */
DYNIBO_API const char *dynibo_robot_name(const DyniboRobot *robot);

/**
 * @brief Returns the number of non-fixed joints.
 * @param[in] robot Robot handle.
 * @return The joint count, or zero when @p robot is null.
 */
DYNIBO_API size_t dynibo_robot_joint_count(const DyniboRobot *robot);

/**
 * @brief Returns the generalized output dimension.
 *
 * For a floating base, generalized vectors begin with six world-frame base
 * entries in angular-then-linear or torque-then-force order, followed by
 * non-fixed joints in URDF order.
 *
 * @param[in] robot Robot handle.
 * @return The generalized count, or zero when @p robot is null.
 */
DYNIBO_API size_t dynibo_robot_generalized_count(const DyniboRobot *robot);

/**
 * @brief Returns the number of links, including the root link.
 * @param[in] robot Robot handle.
 * @return The link count, or zero when @p robot is null.
 */
DYNIBO_API size_t dynibo_robot_link_count(const DyniboRobot *robot);

/**
 * @brief Replaces the root-link pose used by every calculation.
 * @param[in,out] robot Robot handle.
 * @param[in] frame Root-link pose in the world frame.
 * @return DYNIBO_STATUS_OK on success or DYNIBO_STATUS_INVALID_ARGUMENT for a
 * null pointer or invalid pose.
 * @note This operation is valid for both fixed-base and floating-base robots.
 */
DYNIBO_API DyniboStatus dynibo_robot_set_base_frame(
    DyniboRobot *robot, const DyniboPose *frame);

/**
 * @brief Replaces the complete state of a floating base.
 * @param[in,out] robot Floating-base robot handle.
 * @param[in] frame Root-link pose in the world frame.
 * @param[in] velocity Angular-first root velocity expressed in the world frame
 * at the root origin.
 * @param[in] acceleration Angular-first root acceleration expressed in the
 * world frame at the root origin.
 * @return DYNIBO_STATUS_OK on success or DYNIBO_STATUS_INVALID_ARGUMENT for a
 * null pointer, invalid pose, non-finite value, or fixed-base robot.
 */
DYNIBO_API DyniboStatus dynibo_robot_set_floating_base_state(
    DyniboRobot *robot, const DyniboPose *frame,
    DyniboTwist velocity, DyniboTwist acceleration);

/**
 * @brief Resolves a link name to a model-scoped link ID.
 * @param[in] robot Robot handle.
 * @param[in] name Null-terminated UTF-8 link name.
 * @param[out] output Receives the link ID.
 * @return DYNIBO_STATUS_OK on success or DYNIBO_STATUS_INVALID_ARGUMENT for a
 * null pointer, invalid UTF-8 name, or unknown link.
 * @note A link ID is valid only with the robot that produced it.
 */
DYNIBO_API DyniboStatus dynibo_robot_link_id(
    const DyniboRobot *robot, const char *name, size_t *output);

/**
 * @brief Allocates reusable calculation storage for a robot.
 * @param[in] robot Robot handle.
 * @param[out] output Receives the newly allocated workspace.
 * @return DYNIBO_STATUS_OK on success or DYNIBO_STATUS_INVALID_ARGUMENT for a
 * null pointer.
 * @note A workspace must be used only with the robot that created it. Use a
 * separate workspace for each simultaneous calculation.
 * @see dynibo_workspace_destroy
 */
DYNIBO_API DyniboStatus dynibo_workspace_create(
    const DyniboRobot *robot, DyniboWorkspace **output);

/**
 * @brief Destroys a workspace.
 * @param[in] workspace Workspace handle, or null.
 * @note Passing null is allowed. The handle must not be used after this call.
 */
DYNIBO_API void dynibo_workspace_destroy(DyniboWorkspace *workspace);

/**
 * @brief Computes a target-link pose in the world frame.
 *
 * For the joints on the root-to-target path,
 *
 * \f$
 * {}^W T_{\mathrm{target}}(q) = {}^W T_{\mathrm{base}}
 * \prod_{i \in \mathrm{path}} {}^{i-1}T_i(q_i).
 * \f$
 *
 * @param[in] robot Robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] q Joint-position array containing exactly `J` elements.
 * @param[in] q_len Number of elements in @p q; must equal `J`.
 * @param[in] target Link ID returned by dynibo_robot_link_id().
 * @param[out] output Receives the target-link pose.
 * @return DYNIBO_STATUS_OK on success or an error status for an invalid
 * pointer, length, link ID, workspace, or numerical input.
 */
DYNIBO_API DyniboStatus dynibo_forward_kinematics(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, size_t q_len, size_t target, DyniboPose *output);

/**
 * @brief Computes a geometric Jacobian.
 *
 * The result is a column-major `6 x G` matrix expressed in the world frame at
 * the target-link origin. Rows are angular then linear; columns follow the
 * generalized-vector ordering documented by
 * dynibo_robot_generalized_count().
 *
 * \f$
 * {}^W V_{\mathrm{target}} = J(q)\nu, \qquad
 * J(q) = \begin{bmatrix} J_\omega(q) \\ J_v(q) \end{bmatrix}.
 * \f$
 *
 * @param[in] robot Robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] q Joint-position array containing exactly `J` elements.
 * @param[in] q_len Number of elements in @p q; must equal `J`.
 * @param[in] target Link ID returned by dynibo_robot_link_id().
 * @param[out] output Caller-owned buffer receiving the Jacobian.
 * @param[in] output_len Number of elements in @p output; must equal `6 * G`.
 * @return DYNIBO_STATUS_OK on success or an error status for invalid input.
 */
DYNIBO_API DyniboStatus dynibo_jacobian(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, size_t q_len, size_t target,
    double *output, size_t output_len);

/**
 * @brief Computes the time derivative of a geometric Jacobian.
 *
 * The result uses the same column-major `6 x G` layout, frame, origin, and
 * ordering as dynibo_jacobian().
 *
 * \f$
 * {}^W A_{\mathrm{target}} = J(q)\dot\nu
 * + \dot J(q,\nu)\nu.
 * \f$
 *
 * @param[in] robot Robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] q Joint-position array containing exactly `J` elements.
 * @param[in] qd Joint-velocity array containing exactly `J` elements.
 * @param[in] state_len Number of elements in both state arrays; must equal `J`.
 * @param[in] target Link ID returned by dynibo_robot_link_id().
 * @param[out] output Caller-owned buffer receiving the Jacobian derivative.
 * @param[in] output_len Number of elements in @p output; must equal `6 * G`.
 * @return DYNIBO_STATUS_OK on success or an error status for invalid input.
 * @note The state arrays must not overlap @p output.
 */
DYNIBO_API DyniboStatus dynibo_jacobian_derivative(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, const double *qd, size_t state_len, size_t target,
    double *output, size_t output_len);

/**
 * @brief Solves fixed-base inverse kinematics for one target link.
 *
 * Each iteration applies the damped-least-squares update
 *
 * \f$
 * \Delta q = J^T\left(JJ^T + \lambda^2 I\right)^{-1}e,
 * \qquad q_{k+1} = q_k + \Delta q,
 * \f$
 *
 * where \f$e\f$ combines target translation and rotation-vector errors.
 *
 * @param[in] robot Fixed-base robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] initial_q Initial joint-position array containing `J` elements.
 * @param[in] q_len Number of elements in @p initial_q; must equal `J`.
 * @param[in] target Link ID returned by dynibo_robot_link_id().
 * @param[in] desired Desired target-link pose in the world frame.
 * @param[in] options Solver options, normally initialized with
 * dynibo_ik_options_default().
 * @param[out] output Caller-owned solution buffer containing `J` elements.
 * @param[in] output_len Number of elements in @p output; must equal `J`.
 * @return DYNIBO_STATUS_OK on convergence, DYNIBO_STATUS_SOLVER_ERROR for
 * numerical failure or non-convergence, or another error status for invalid
 * input.
 * @note Floating-base inverse kinematics is not supported.
 */
DYNIBO_API DyniboStatus dynibo_inverse_kinematics(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *initial_q, size_t q_len, size_t target,
    const DyniboPose *desired, DyniboIkOptions options,
    double *output, size_t output_len);

/**
 * @brief Computes spatial velocity at a point fixed to a target link.
 *
 * The angular-first result is expressed in the world frame at the selected
 * tool point. The root pose and motion come from the state stored in @p robot.
 *
 * \f$
 * {}^W V_{\mathrm{tool}} = J_{\mathrm{tool}}(q)\nu.
 * \f$
 *
 * @param[in] robot Robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] q Joint-position array containing `J` elements.
 * @param[in] qd Joint-velocity array containing `J` elements.
 * @param[in] state_len Number of elements in both state arrays; must equal `J`.
 * @param[in] target Link ID returned by dynibo_robot_link_id().
 * @param[in] tool Tool pose relative to the target-link frame.
 * @param[out] output Receives the world-expressed tool-point velocity.
 * @return DYNIBO_STATUS_OK on success or an error status for invalid input.
 */
DYNIBO_API DyniboStatus dynibo_forward_velocity_kinematics(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, const double *qd, size_t state_len, size_t target,
    const DyniboPose *tool, DyniboTwist *output);

/**
 * @brief Computes spatial acceleration at a target-link origin.
 *
 * \f$
 * {}^W A_{\mathrm{target}} = J(q)\dot\nu
 * + \dot J(q,\nu)\nu.
 * \f$
 *
 * @param[in] robot Robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] q Joint-position array containing `J` elements.
 * @param[in] qd Joint-velocity array containing `J` elements.
 * @param[in] qdd Joint-acceleration array containing `J` elements.
 * @param[in] state_len Number of elements in each state array; must equal `J`.
 * @param[in] target Link ID returned by dynibo_robot_link_id().
 * @param[out] output Receives the angular-first, world-expressed acceleration.
 * @return DYNIBO_STATUS_OK on success or an error status for invalid input.
 */
DYNIBO_API DyniboStatus dynibo_forward_acceleration_kinematics(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, const double *qd, const double *qdd,
    size_t state_len, size_t target, DyniboTwist *output);

/**
 * @brief Computes the generalized mass matrix.
 *
 * The result is a column-major `G x G` matrix whose rows and columns follow
 * the generalized-vector ordering documented by
 * dynibo_robot_generalized_count().
 *
 * It is the inertia term in the manipulator equation
 *
 * \f$
 * \tau = M(q)\dot\nu + C(q,\nu)\nu + g(q).
 * \f$
 *
 * @param[in] robot Robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] q Joint-position array containing `J` elements.
 * @param[in] q_len Number of elements in @p q; must equal `J`.
 * @param[out] output Caller-owned buffer receiving the matrix.
 * @param[in] output_len Number of elements in @p output; must equal `G * G`.
 * @return DYNIBO_STATUS_OK on success or an error status for invalid input.
 * @note @p q must not overlap @p output.
 */
DYNIBO_API DyniboStatus dynibo_mass_matrix(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, size_t q_len, double *output, size_t output_len);

/**
 * @brief Computes Coriolis and centrifugal generalized forces.
 *
 * Gravity, prescribed base acceleration, and external loads are excluded. A
 * floating-base output begins with the world-frame root wrench in
 * torque-then-force order, followed by scalar joint forces.
 *
 * \f$
 * \tau_v = C(q,\nu)\nu.
 * \f$
 *
 * @param[in] robot Robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] q Joint-position array containing `J` elements.
 * @param[in] qd Joint-velocity array containing `J` elements.
 * @param[in] state_len Number of elements in both state arrays; must equal `J`.
 * @param[out] output Caller-owned generalized-force buffer.
 * @param[in] output_len Number of elements in @p output; must equal `G`.
 * @return DYNIBO_STATUS_OK on success or an error status for invalid input.
 * @note The state arrays must not overlap @p output.
 */
DYNIBO_API DyniboStatus dynibo_velocity_product_forces(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, const double *qd, size_t state_len,
    double *output, size_t output_len);

/**
 * @brief Computes gravity and external-load generalized forces.
 *
 * The root pose comes from the state stored in @p robot. A floating-base output
 * begins with the world-frame root wrench in torque-then-force order, followed
 * by scalar joint forces.
 *
 * With no external loads, the returned vector is
 *
 * \f$
 * g(q) = \tau(q,0,0).
 * \f$
 *
 * @param[in] robot Robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] q Joint-position array containing `J` elements.
 * @param[in] q_len Number of elements in @p q; must equal `J`.
 * @param[in] loads External loads, or null when @p load_count is zero.
 * @param[in] load_count Number of entries in @p loads.
 * @param[out] output Caller-owned generalized-force buffer.
 * @param[in] output_len Number of elements in @p output; must equal `G`.
 * @return DYNIBO_STATUS_OK on success or an error status for invalid input.
 */
DYNIBO_API DyniboStatus dynibo_gravity(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, size_t q_len, const DyniboLoad *loads, size_t load_count,
    double *output, size_t output_len);

/**
 * @brief Computes Newton-Euler inverse dynamics.
 *
 * The root pose and motion come from the state stored in @p robot. A
 * floating-base output begins with the world-frame root wrench in
 * torque-then-force order, followed by scalar joint forces.
 *
 * With a stationary base and no external loads, the returned forces satisfy
 *
 * \f$
 * \tau = M(q)\dot\nu + C(q,\nu)\nu + g(q).
 * \f$
 *
 * @param[in] robot Robot handle.
 * @param[in,out] workspace Workspace created for @p robot.
 * @param[in] q Joint-position array containing `J` elements.
 * @param[in] qd Joint-velocity array containing `J` elements.
 * @param[in] qdd Joint-acceleration array containing `J` elements.
 * @param[in] state_len Number of elements in each state array; must equal `J`.
 * @param[in] loads External loads, or null when @p load_count is zero.
 * @param[in] load_count Number of entries in @p loads.
 * @param[out] output Caller-owned generalized-force buffer.
 * @param[in] output_len Number of elements in @p output; must equal `G`.
 * @return DYNIBO_STATUS_OK on success or an error status for invalid input.
 */
DYNIBO_API DyniboStatus dynibo_inverse_dynamics(
    const DyniboRobot *robot, DyniboWorkspace *workspace,
    const double *q, const double *qd, const double *qdd, size_t state_len,
    const DyniboLoad *loads, size_t load_count,
    double *output, size_t output_len);

#ifdef __cplusplus
}
#endif

#endif
