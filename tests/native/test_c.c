#include <dynibo/dynibo.h>

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expression)                                                       \
    do {                                                                        \
        if (!(expression)) {                                                    \
            fprintf(stderr, "check failed at %s:%d: %s\n",                    \
                    __FILE__, __LINE__, #expression);                           \
            return 1;                                                           \
        }                                                                       \
    } while (0)

static void check(DyniboStatus status) {
    if (status != DYNIBO_STATUS_OK) {
        fprintf(stderr, "dynibo error: %s\n", dynibo_last_error_message());
        abort();
    }
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    CHECK(strcmp(dynibo_version(), "0.1.0") == 0);
    CHECK(dynibo_robot_name(NULL) == NULL);
    CHECK(dynibo_robot_joint_count(NULL) == 0);
    CHECK(dynibo_robot_link_count(NULL) == 0);
    dynibo_robot_destroy(NULL);
    dynibo_workspace_destroy(NULL);

    const DyniboIkOptions defaults = dynibo_ik_options_default();
    CHECK(defaults.max_iterations > 0);
    CHECK(defaults.translation_tolerance > 0.0);
    CHECK(defaults.rotation_tolerance > 0.0);
    CHECK(defaults.damping > 0.0);
    CHECK(defaults.max_step_norm > 0.0);

    DyniboRobot *robot = NULL;
    DyniboWorkspace *workspace = NULL;
    check(dynibo_robot_load_urdf(argv[1], &robot));
    check(dynibo_workspace_create(robot, &workspace));
    CHECK(strcmp(dynibo_robot_name(robot), "test_arm") == 0);
    CHECK(dynibo_robot_joint_count(robot) == 4);
    CHECK(dynibo_robot_link_count(robot) == 5);

    size_t target = 0;
    check(dynibo_robot_link_id(robot, "test_link_4", &target));
    const size_t n = dynibo_robot_joint_count(robot);
    double *q = (double *)calloc(n, sizeof(double));
    double *jacobian = (double *)calloc(6 * n, sizeof(double));
    double *jacobian_derivative = (double *)calloc(6 * n, sizeof(double));
    double *square = (double *)calloc(n * n, sizeof(double));
    double *output = (double *)calloc(n, sizeof(double));
    CHECK(q != NULL && jacobian != NULL && jacobian_derivative != NULL);
    CHECK(square != NULL && output != NULL);

    DyniboPose pose;
    check(dynibo_forward_kinematics(robot, workspace, q, n, target, &pose));
    CHECK(fabs(pose.translation[0] - 0.62) < 1.0e-12);
    CHECK(fabs(pose.translation[1]) < 1.0e-12);
    CHECK(fabs(pose.translation[2] - 0.108) < 1.0e-12);
    check(dynibo_jacobian(robot, workspace, q, n, target, jacobian, 6 * n));

    const DyniboPose identity = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    const DyniboTwist zero_twist = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    DyniboTwist twist;
    check(dynibo_forward_velocity(
        robot, workspace, q, q, n, target, &identity, &identity, &twist));
    check(dynibo_forward_acceleration(
        robot, workspace, q, q, q, n, target, &twist));
    check(dynibo_gravity(
        robot, workspace, q, n, &identity, NULL, 0, output, n));
    check(dynibo_inverse_dynamics(
        robot, workspace, q, q, q, n, &identity, zero_twist, zero_twist,
        NULL, 0, output, n));

    DyniboIkOptions options = defaults;
    check(dynibo_inverse_kinematics(
        robot, workspace, q, n, target, &pose, options, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index]) < 1.0e-12);
    }

    const double reference_q[4] = {0.2, 1.0, -0.7, 0.4};
    const double reference_qd[4] = {-0.3, 0.5, -0.2, 0.8};
    const double reference_qdd[4] = {0.7, -0.4, 0.1, 0.3};
    const double expected_gravity[4] = {
        1.7763568394002505e-15, 39.629058959145354,
        17.60815765611755, 0.053134179784508524};
    const double expected_dynamics[4] = {
        1.7649236924309104, 38.319908179086525,
        17.136450444507805, 0.05169960944426318};
    check(dynibo_gravity(robot, workspace, reference_q, n, &identity,
                       NULL, 0, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index] - expected_gravity[index]) < 2.0e-10);
    }
    check(dynibo_inverse_dynamics(
        robot, workspace, reference_q, reference_qd, reference_qdd, n,
        &identity, zero_twist, zero_twist, NULL, 0, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index] - expected_dynamics[index]) < 2.0e-10);
    }

    const double zero_qdd[4] = {0.0, 0.0, 0.0, 0.0};
    check(dynibo_mass_matrix(robot, workspace, reference_q, n, square, n * n));
    for (size_t row = 0; row < n; ++row) {
        for (size_t col = 0; col < n; ++col) {
            CHECK(fabs(square[col * n + row] - square[row * n + col]) < 1.0e-12);
        }
    }
    check(dynibo_coriolis_matrix(
        robot, workspace, reference_q, reference_qd, n, square, n * n));
    double gravity_vec[4];
    double bias_vec[4];
    check(dynibo_gravity(
        robot, workspace, reference_q, n, &identity, NULL, 0, gravity_vec, n));
    check(dynibo_inverse_dynamics(
        robot, workspace, reference_q, reference_qd, zero_qdd, n,
        &identity, zero_twist, zero_twist, NULL, 0, bias_vec, n));
    for (size_t row = 0; row < n; ++row) {
        double reconstructed = gravity_vec[row];
        for (size_t col = 0; col < n; ++col) {
            reconstructed += square[col * n + row] * reference_qd[col];
        }
        CHECK(fabs(reconstructed - bias_vec[row]) < 1.0e-10);
    }
    check(dynibo_jacobian_derivative(
        robot, workspace, reference_q, reference_qd, n, target,
        jacobian_derivative, 6 * n));
    DyniboTwist origin_acceleration;
    check(dynibo_forward_acceleration(
        robot, workspace, reference_q, reference_qd, zero_qdd, n, target,
        &origin_acceleration));
    for (size_t row = 0; row < 6; ++row) {
        const double expected = row < 3
            ? origin_acceleration.angular[row]
            : origin_acceleration.linear[row - 3];
        double contracted = 0.0;
        for (size_t col = 0; col < n; ++col) {
            contracted += jacobian_derivative[col * 6 + row] * reference_qd[col];
        }
        CHECK(fabs(contracted - expected) < 1.0e-10);
    }

    CHECK(dynibo_jacobian(
        robot, workspace, q, n - 1, target, jacobian, 6 * n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(strlen(dynibo_last_error_message()) > 0);
    CHECK(dynibo_robot_link_id(robot, "missing", &target)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_robot_link_id(robot, "test_link_4", &target) == DYNIBO_STATUS_OK);
    CHECK(strlen(dynibo_last_error_message()) == 0);

    CHECK(dynibo_forward_kinematics(
        NULL, workspace, q, n, target, &pose) == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_forward_kinematics(
        robot, NULL, q, n, target, &pose) == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_forward_kinematics(
        robot, workspace, NULL, n, target, &pose) == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_forward_kinematics(
        robot, workspace, q, n, target, NULL) == DYNIBO_STATUS_INVALID_ARGUMENT);

    DyniboPose invalid_pose = identity;
    memset(invalid_pose.rotation_xyzw, 0, sizeof(invalid_pose.rotation_xyzw));
    CHECK(dynibo_inverse_kinematics(
        robot, workspace, q, n, target, &invalid_pose, options, output, n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_gravity(
        robot, workspace, q, n, &identity, NULL, 1, output, n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    const DyniboLoad invalid_load = {SIZE_MAX, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    CHECK(dynibo_gravity(
        robot, workspace, q, n, &identity, &invalid_load, 1, output, n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_mass_matrix(
        robot, workspace, reference_q, n, square, n * n - 1)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_coriolis_matrix(
        robot, workspace, reference_q, reference_qd, n - 1, square, n * n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_jacobian_derivative(
        robot, workspace, reference_q, reference_qd, n, target,
        jacobian_derivative, 6 * n - 1)
        == DYNIBO_STATUS_INVALID_ARGUMENT);

    free(output);
    free(square);
    free(jacobian_derivative);
    free(jacobian);
    free(q);
    dynibo_workspace_destroy(workspace);
    dynibo_robot_destroy(robot);
    return 0;
}
