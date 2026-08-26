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

static int read_reference(
    const char *path, const char *key, double *output, size_t count) {
    FILE *file = fopen(path, "r");
    if (file == NULL) return 0;
    char line[2048];
    while (fgets(line, sizeof(line), file) != NULL) {
        if (line[0] == '#') continue;
        char *token = strtok(line, "\t\r\n");
        if (token == NULL || strcmp(token, key) != 0) continue;
        for (size_t index = 0; index < count; ++index) {
            token = strtok(NULL, "\t\r\n");
            if (token == NULL) {
                fclose(file);
                return 0;
            }
            char *end = NULL;
            output[index] = strtod(token, &end);
            if (end == token || *end != '\0') {
                fclose(file);
                return 0;
            }
        }
        fclose(file);
        return strtok(NULL, "\t\r\n") == NULL;
    }
    fclose(file);
    return 0;
}

int main(int argc, char **argv) {
    CHECK(argc == 3);
    CHECK(strcmp(dynibo_version(), "0.3.0") == 0);
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

    DyniboRobot *invalid_mode_robot = (DyniboRobot *)(uintptr_t)1;
    CHECK(dynibo_robot_from_urdf_with_base(
        argv[1], 99, &invalid_mode_robot) == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(invalid_mode_robot == NULL);

    DyniboRobot *robot = NULL;
    DyniboWorkspace *workspace = NULL;
    check(dynibo_robot_from_urdf(argv[1], &robot));
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
    DyniboPose shifted_base = identity;
    shifted_base.translation[0] = 0.25;
    check(dynibo_robot_set_base_frame(robot, &shifted_base));
    check(dynibo_forward_kinematics(robot, workspace, q, n, target, &pose));
    CHECK(fabs(pose.translation[0] - 0.87) < 1.0e-12);
    check(dynibo_robot_set_base_frame(robot, &identity));
    check(dynibo_forward_kinematics(robot, workspace, q, n, target, &pose));
    DyniboTwist twist;
    check(dynibo_forward_velocity_kinematics(
        robot, workspace, q, q, n, target, &identity, &twist));
    check(dynibo_forward_acceleration_kinematics(
        robot, workspace, q, q, q, n, target, &twist));
    check(dynibo_gravity(
        robot, workspace, q, n, NULL, 0, output, n));
    check(dynibo_inverse_dynamics(
        robot, workspace, q, q, q, n, NULL, 0, output, n));

    DyniboIkOptions options = defaults;
    check(dynibo_inverse_kinematics(
        robot, workspace, q, n, target, &pose, options, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index]) < 1.0e-12);
    }

    double reference_q[4];
    double reference_qd[4];
    double reference_qdd[4];
    double expected_gravity[4];
    double expected_dynamics[4];
    double expected_translation[3];
    CHECK(read_reference(argv[2], "q", reference_q, 4));
    CHECK(read_reference(argv[2], "qd", reference_qd, 4));
    CHECK(read_reference(argv[2], "qdd", reference_qdd, 4));
    CHECK(read_reference(argv[2], "fk_translation", expected_translation, 3));
    CHECK(read_reference(argv[2], "gravity", expected_gravity, 4));
    CHECK(read_reference(argv[2], "rnea", expected_dynamics, 4));
    check(dynibo_forward_kinematics(
        robot, workspace, reference_q, n, target, &pose));
    for (size_t index = 0; index < 3; ++index) {
        CHECK(fabs(pose.translation[index] - expected_translation[index]) < 2.0e-12);
    }
    check(dynibo_gravity(
        robot, workspace, reference_q, n, NULL, 0, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index] - expected_gravity[index]) < 2.0e-10);
    }
    check(dynibo_inverse_dynamics(
        robot, workspace, reference_q, reference_qd, reference_qdd, n,
        NULL, 0, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index] - expected_dynamics[index]) < 2.0e-10);
    }
    check(dynibo_forward_dynamics(
        robot, workspace, reference_q, reference_qd, n,
        expected_dynamics, n, NULL, 0, output, n));
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(output[index] - reference_qdd[index]) < 2.0e-10);
    }

    const double zero_qdd[4] = {0.0, 0.0, 0.0, 0.0};
    check(dynibo_mass_matrix(robot, workspace, reference_q, n, square, n * n));
    for (size_t row = 0; row < n; ++row) {
        for (size_t col = 0; col < n; ++col) {
            CHECK(fabs(square[col * n + row] - square[row * n + col]) < 1.0e-12);
        }
    }
    double velocity_product[4];
    check(dynibo_velocity_product_forces(
        robot, workspace, reference_q, reference_qd, n, velocity_product, n));
    double gravity_vec[4];
    double bias_vec[4];
    check(dynibo_gravity(
        robot, workspace, reference_q, n, NULL, 0, gravity_vec, n));
    check(dynibo_inverse_dynamics(
        robot, workspace, reference_q, reference_qd, zero_qdd, n,
        NULL, 0, bias_vec, n));
    for (size_t row = 0; row < n; ++row) {
        const double reconstructed = gravity_vec[row] + velocity_product[row];
        CHECK(fabs(reconstructed - bias_vec[row]) < 1.0e-10);
    }
    check(dynibo_jacobian_derivative(
        robot, workspace, reference_q, reference_qd, n, target,
        jacobian_derivative, 6 * n));
    DyniboTwist origin_acceleration;
    check(dynibo_forward_acceleration_kinematics(
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
        robot, workspace, q, n, NULL, 1, output, n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    const DyniboLoad invalid_load = {SIZE_MAX, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    CHECK(dynibo_gravity(
        robot, workspace, q, n, &invalid_load, 1, output, n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_mass_matrix(
        robot, workspace, reference_q, n, square, n * n - 1)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_velocity_product_forces(
        robot, workspace, reference_q, reference_qd, n - 1, output, n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_forward_dynamics(
        robot, workspace, reference_q, reference_qd, n,
        expected_dynamics, n - 1, NULL, 0, output, n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    CHECK(dynibo_jacobian_derivative(
        robot, workspace, reference_q, reference_qd, n, target,
        jacobian_derivative, 6 * n - 1)
        == DYNIBO_STATUS_INVALID_ARGUMENT);
    double overlapping_mass[16] = {0.2, 1.0, -0.7, 0.4};
    CHECK(dynibo_mass_matrix(
        robot, workspace, overlapping_mass, n, overlapping_mass, n * n)
        == DYNIBO_STATUS_INVALID_ARGUMENT);

    DyniboRobot *floating = NULL;
    DyniboWorkspace *floating_workspace = NULL;
    check(dynibo_robot_from_urdf_with_base(
        argv[1], DYNIBO_BASE_FLOATING, &floating));
    check(dynibo_workspace_create(floating, &floating_workspace));
    size_t floating_target = 0;
    check(dynibo_robot_link_id(floating, "test_link_4", &floating_target));
    CHECK(dynibo_robot_generalized_count(floating) == 10);
    double base_translation[3];
    double base_rotation[4];
    double base_velocity[6];
    double base_acceleration[6];
    CHECK(read_reference(argv[2], "floating_base_translation", base_translation, 3));
    CHECK(read_reference(argv[2], "floating_base_rotation_xyzw", base_rotation, 4));
    CHECK(read_reference(argv[2], "floating_base_velocity", base_velocity, 6));
    CHECK(read_reference(argv[2], "floating_base_acceleration", base_acceleration, 6));
    DyniboPose floating_base;
    memcpy(floating_base.translation, base_translation, sizeof(base_translation));
    memcpy(floating_base.rotation_xyzw, base_rotation, sizeof(base_rotation));
    DyniboTwist floating_velocity;
    DyniboTwist floating_acceleration;
    memcpy(floating_velocity.angular, base_velocity, 3 * sizeof(double));
    memcpy(floating_velocity.linear, base_velocity + 3, 3 * sizeof(double));
    memcpy(floating_acceleration.angular, base_acceleration, 3 * sizeof(double));
    memcpy(floating_acceleration.linear, base_acceleration + 3, 3 * sizeof(double));
    check(dynibo_robot_set_floating_base_state(
        floating, &floating_base, floating_velocity, floating_acceleration));
    double floating_output[10];
    double floating_reference[10];
    check(dynibo_forward_kinematics(
        floating, floating_workspace, reference_q, n, floating_target, &pose));
    CHECK(read_reference(argv[2], "floating_fk_translation", expected_translation, 3));
    for (size_t index = 0; index < 3; ++index) {
        CHECK(fabs(pose.translation[index] - expected_translation[index]) < 2.0e-12);
    }
    check(dynibo_gravity(
        floating, floating_workspace, reference_q, n, NULL, 0,
        floating_output, 10));
    CHECK(read_reference(argv[2], "floating_gravity", floating_reference, 10));
    for (size_t index = 0; index < 10; ++index) {
        CHECK(fabs(floating_output[index] - floating_reference[index]) < 2.0e-10);
    }
    check(dynibo_inverse_dynamics(
        floating, floating_workspace, reference_q, reference_qd, reference_qdd, n,
        NULL, 0, floating_output, 10));
    CHECK(read_reference(argv[2], "floating_rnea", floating_reference, 10));
    for (size_t index = 0; index < 10; ++index) {
        CHECK(fabs(floating_output[index] - floating_reference[index]) < 2.0e-10);
    }
    check(dynibo_forward_dynamics(
        floating, floating_workspace, reference_q, reference_qd, n,
        floating_reference, 10, NULL, 0, floating_output, 10));
    for (size_t index = 0; index < 6; ++index) {
        CHECK(fabs(floating_output[index] - base_acceleration[index]) < 2.0e-9);
    }
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(floating_output[6 + index] - reference_qdd[index]) < 2.0e-9);
    }
    double floating_load_values[6];
    CHECK(read_reference(argv[2], "floating_load", floating_load_values, 6));
    DyniboLoad floating_load;
    floating_load.link_id = floating_target;
    memcpy(floating_load.torque, floating_load_values, 3 * sizeof(double));
    memcpy(floating_load.force, floating_load_values + 3, 3 * sizeof(double));
    check(dynibo_inverse_dynamics(
        floating, floating_workspace, reference_q, reference_qd, reference_qdd, n,
        &floating_load, 1, floating_output, 10));
    CHECK(read_reference(argv[2], "floating_rnea_loaded", floating_reference, 10));
    for (size_t index = 0; index < 10; ++index) {
        CHECK(fabs(floating_output[index] - floating_reference[index]) < 2.0e-10);
    }
    check(dynibo_forward_dynamics(
        floating, floating_workspace, reference_q, reference_qd, n,
        floating_reference, 10, &floating_load, 1, floating_output, 10));
    for (size_t index = 0; index < 6; ++index) {
        CHECK(fabs(floating_output[index] - base_acceleration[index]) < 2.0e-9);
    }
    for (size_t index = 0; index < n; ++index) {
        CHECK(fabs(floating_output[6 + index] - reference_qdd[index]) < 2.0e-9);
    }
    dynibo_workspace_destroy(floating_workspace);
    dynibo_robot_destroy(floating);

    free(output);
    free(square);
    free(jacobian_derivative);
    free(jacobian);
    free(q);
    dynibo_workspace_destroy(workspace);
    dynibo_robot_destroy(robot);
    return 0;
}
