#include <dynibo/dynibo.h>

#include <stdio.h>
#include <stdlib.h>

static void check(DyniboStatus status) {
    if (status != DYNIBO_STATUS_OK) {
        fprintf(stderr, "dynibo: %s\n", dynibo_last_error_message());
        exit(EXIT_FAILURE);
    }
}

static double *allocate(size_t count) {
    double *values = (double *)calloc(count, sizeof(double));
    if (values == NULL && count != 0) {
        fprintf(stderr, "could not allocate %zu doubles\n", count);
        exit(EXIT_FAILURE);
    }
    return values;
}

static void print_vector(const char *label, const double *values, size_t count) {
    size_t index;
    printf("%s: [", label);
    for (index = 0; index < count; ++index) {
        printf("%s%.5f", index == 0 ? "" : ", ", values[index]);
    }
    puts("]");
}

static void print_matrix(
    const char *label, const double *values, size_t rows, size_t columns) {
    size_t row;
    size_t column;
    printf("%s (%zu x %zu):\n", label, rows, columns);
    for (row = 0; row < rows; ++row) {
        printf("  ");
        for (column = 0; column < columns; ++column) {
            /* All dynibo matrix outputs are column-major. */
            printf("% .5f%s", values[row + column * rows],
                   column + 1 == columns ? "" : " ");
        }
        putchar('\n');
    }
}

static void print_twist(const char *label, DyniboTwist value) {
    double values[6] = {
        value.angular[0], value.angular[1], value.angular[2],
        value.linear[0], value.linear[1], value.linear[2]
    };
    print_vector(label, values, 6);
}

int main(int argc, char **argv) {
    static const double sample_q[7] = {0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7};
    static const double sample_qd[7] = {0.10, -0.20, 0.15, 0.05, -0.10, 0.20, -0.05};
    static const double sample_qdd[7] = {0.20, 0.10, -0.10, 0.05, 0.10, -0.05, 0.15};
    static const double ik_initial_q[7] = {0.05, -0.2, -0.05, -1.6, 0.05, 1.4, 0.6};
    const DyniboPose identity = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    DyniboRobot *robot = NULL;
    DyniboWorkspace *workspace = NULL;
    DyniboPose pose;
    DyniboTwist velocity;
    DyniboTwist acceleration;
    DyniboLoad load = {0};
    DyniboIkOptions ik_options;
    double *q;
    double *qd;
    double *qdd;
    double *jacobian;
    double *jacobian_derivative;
    double *ik_solution;
    double *mass_matrix;
    double *velocity_forces;
    double *gravity;
    double *joint_forces;
    size_t target = 0;
    size_t n;
    size_t g;
    size_t index;

    if (argc != 3) {
        fprintf(stderr, "usage: %s ROBOT.urdf TOOL_LINK\n", argv[0]);
        return EXIT_FAILURE;
    }

    check(dynibo_robot_from_urdf(argv[1], &robot));
    check(dynibo_workspace_create(robot, &workspace));
    check(dynibo_robot_link_id(robot, argv[2], &target));
    n = dynibo_robot_joint_count(robot);
    g = dynibo_robot_generalized_count(robot);
    if (n != 7) {
        fprintf(stderr, "this Franka example expects 7 non-fixed joints, got %zu\n", n);
        dynibo_workspace_destroy(workspace);
        dynibo_robot_destroy(robot);
        return EXIT_FAILURE;
    }

    q = allocate(n);
    qd = allocate(n);
    qdd = allocate(n);
    jacobian = allocate(6 * g);
    jacobian_derivative = allocate(6 * g);
    ik_solution = allocate(n);
    mass_matrix = allocate(g * g);
    velocity_forces = allocate(g);
    gravity = allocate(g);
    joint_forces = allocate(g);
    for (index = 0; index < n; ++index) {
        q[index] = sample_q[index];
        qd[index] = sample_qd[index];
        qdd[index] = sample_qdd[index];
    }

    /* forward_kinematics -- target-link pose in the world frame. */
    check(dynibo_forward_kinematics(robot, workspace, q, n, target, &pose));

    /* jacobian and jacobian_derivative -- flat column-major matrices. */
    check(dynibo_jacobian(robot, workspace, q, n, target, jacobian, 6 * g));
    check(dynibo_jacobian_derivative(
        robot, workspace, q, qd, n, target, jacobian_derivative, 6 * g));

    check(dynibo_forward_velocity_kinematics(
        robot, workspace, q, qd, n, target, &identity, &velocity));
    check(dynibo_forward_acceleration_kinematics(
        robot, workspace, q, qd, qdd, n, target, &acceleration));

    /* inverse_kinematics -- recover a known, reachable pose. */
    ik_options = dynibo_ik_options_default();
    check(dynibo_inverse_kinematics(
        robot, workspace, ik_initial_q, n, target, &pose, ik_options,
        ik_solution, n));

    /* Joint-space dynamics. */
    check(dynibo_mass_matrix(robot, workspace, q, n, mass_matrix, g * g));
    check(dynibo_velocity_product_forces(
        robot, workspace, q, qd, n, velocity_forces, g));

    /* Link-local downward force at the flange origin. */
    load.link_id = target;
    load.force[2] = -5.0;
    check(dynibo_gravity(
        robot, workspace, q, n, &load, 1, gravity, g));
    check(dynibo_inverse_dynamics(
        robot, workspace, q, qd, qdd, n, &load, 1, joint_forces, g));

    printf("loaded %s: %zu links, %zu non-fixed joints\n",
           dynibo_robot_name(robot), dynibo_robot_link_count(robot), n);
    print_vector("forward_kinematics translation [m]", pose.translation, 3);
    print_vector("forward_kinematics quaternion (x, y, z, w)", pose.rotation_xyzw, 4);
    print_matrix("jacobian", jacobian, 6, g);
    print_matrix("jacobian_derivative", jacobian_derivative, 6, g);
    print_twist("forward_velocity_kinematics", velocity);
    print_twist("forward_acceleration_kinematics", acceleration);
    print_vector("inverse_kinematics", ik_solution, n);
    print_matrix("mass_matrix", mass_matrix, g, g);
    print_vector("velocity_product_forces", velocity_forces, g);
    print_vector("gravity (including external load)", gravity, g);
    print_vector("inverse_dynamics (including external load)", joint_forces, g);

    free(joint_forces);
    free(gravity);
    free(velocity_forces);
    free(mass_matrix);
    free(ik_solution);
    free(jacobian_derivative);
    free(jacobian);
    free(qdd);
    free(qd);
    free(q);
    dynibo_workspace_destroy(workspace);
    dynibo_robot_destroy(robot);
    return EXIT_SUCCESS;
}
