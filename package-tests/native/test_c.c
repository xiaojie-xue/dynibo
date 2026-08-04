#include <dyno/dyno.h>

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

static void check(DynoStatus status) {
    if (status != DYNO_STATUS_OK) {
        fprintf(stderr, "dyno error: %s\n", dyno_last_error_message());
        abort();
    }
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    CHECK(strcmp(dyno_version(), "0.1.0") == 0);
    CHECK(dyno_robot_name(NULL) == NULL);
    CHECK(dyno_robot_joint_count(NULL) == 0);
    CHECK(dyno_robot_link_count(NULL) == 0);
    dyno_robot_destroy(NULL);
    dyno_workspace_destroy(NULL);

    const DynoIkOptions defaults = dyno_ik_options_default();
    CHECK(defaults.max_iterations > 0);
    CHECK(defaults.translation_tolerance > 0.0);
    CHECK(defaults.rotation_tolerance > 0.0);
    CHECK(defaults.damping > 0.0);
    CHECK(defaults.max_step_norm > 0.0);

    DynoRobot *robot = NULL;
    DynoWorkspace *workspace = NULL;
    check(dyno_robot_load_urdf(argv[1], &robot));
    check(dyno_workspace_create(robot, &workspace));
    CHECK(strcmp(dyno_robot_name(robot), "test_arm") == 0);
    CHECK(dyno_robot_joint_count(robot) == 4);
    CHECK(dyno_robot_link_count(robot) == 5);

    size_t target = 0;
    check(dyno_robot_link_id(robot, "test_link_4", &target));
    const size_t n = dyno_robot_joint_count(robot);
    double *q = (double *)calloc(n, sizeof(double));
    double *jacobian = (double *)calloc(6 * n, sizeof(double));
    double *output = (double *)calloc(n, sizeof(double));
    CHECK(q != NULL && jacobian != NULL && output != NULL);

    DynoPose pose;
    check(dyno_forward_kinematics(robot, workspace, q, n, target, &pose));
    CHECK(fabs(pose.translation[0] - 0.62) < 1.0e-12);
    CHECK(fabs(pose.translation[1]) < 1.0e-12);
    CHECK(fabs(pose.translation[2] - 0.108) < 1.0e-12);
    check(dyno_jacobian(robot, workspace, q, n, target, jacobian, 6 * n));

    const DynoPose identity = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    const DynoTwist zero_twist = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    DynoTwist twist;
    check(dyno_forward_velocity(
        robot, workspace, q, q, n, target, &identity, &identity, &twist));
    check(dyno_forward_acceleration(
        robot, workspace, q, q, q, n, target, &twist));
    check(dyno_gravity(
        robot, workspace, q, n, &identity, NULL, 0, output, n));
    check(dyno_inverse_dynamics(
        robot, workspace, q, q, q, n, &identity, zero_twist, zero_twist,
        NULL, 0, output, n));

    DynoIkOptions options = defaults;
    check(dyno_inverse_kinematics(
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
    check(dyno_gravity(robot, workspace, reference_q, n, &identity,
                       NULL, 0, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index] - expected_gravity[index]) < 2.0e-10);
    }
    check(dyno_inverse_dynamics(
        robot, workspace, reference_q, reference_qd, reference_qdd, n,
        &identity, zero_twist, zero_twist, NULL, 0, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index] - expected_dynamics[index]) < 2.0e-10);
    }

    CHECK(dyno_jacobian(
        robot, workspace, q, n - 1, target, jacobian, 6 * n) != DYNO_STATUS_OK);
    CHECK(strlen(dyno_last_error_message()) > 0);
    CHECK(dyno_robot_link_id(robot, "missing", &target) == DYNO_STATUS_ERROR);
    CHECK(dyno_robot_link_id(robot, "test_link_4", &target) == DYNO_STATUS_OK);
    CHECK(strlen(dyno_last_error_message()) == 0);

    CHECK(dyno_forward_kinematics(
        NULL, workspace, q, n, target, &pose) == DYNO_STATUS_INVALID_ARGUMENT);
    CHECK(dyno_forward_kinematics(
        robot, NULL, q, n, target, &pose) == DYNO_STATUS_INVALID_ARGUMENT);
    CHECK(dyno_forward_kinematics(
        robot, workspace, NULL, n, target, &pose) == DYNO_STATUS_INVALID_ARGUMENT);
    CHECK(dyno_forward_kinematics(
        robot, workspace, q, n, target, NULL) == DYNO_STATUS_INVALID_ARGUMENT);

    DynoPose invalid_pose = identity;
    memset(invalid_pose.rotation_xyzw, 0, sizeof(invalid_pose.rotation_xyzw));
    CHECK(dyno_inverse_kinematics(
        robot, workspace, q, n, target, &invalid_pose, options, output, n)
        == DYNO_STATUS_INVALID_ARGUMENT);
    CHECK(dyno_gravity(
        robot, workspace, q, n, &identity, NULL, 1, output, n)
        == DYNO_STATUS_INVALID_ARGUMENT);
    const DynoLoad invalid_load = {SIZE_MAX, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    CHECK(dyno_gravity(
        robot, workspace, q, n, &identity, &invalid_load, 1, output, n)
        == DYNO_STATUS_INVALID_ARGUMENT);

    free(output);
    free(jacobian);
    free(q);
    dyno_workspace_destroy(workspace);
    dyno_robot_destroy(robot);
    return 0;
}
