#include <dyno/dyno.h>

#include <math.h>
#include <stdio.h>
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

    DynoIkOptions options = dyno_ik_options_default();
    check(dyno_inverse_kinematics(
        robot, workspace, q, n, target, &pose, options, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index]) < 1.0e-12);
    }

    CHECK(dyno_jacobian(
        robot, workspace, q, n - 1, target, jacobian, 6 * n) != DYNO_STATUS_OK);
    CHECK(strlen(dyno_last_error_message()) > 0);
    CHECK(dyno_robot_link_id(robot, "missing", &target) == DYNO_STATUS_ERROR);

    free(output);
    free(jacobian);
    free(q);
    dyno_workspace_destroy(workspace);
    dyno_robot_destroy(robot);
    return 0;
}
