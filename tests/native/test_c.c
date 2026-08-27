#include <dynibo/dynibo.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "failed: %s (%s)\n", #x, dynibo_last_error_message()); return 1; } } while (0)
#define OK(x) CHECK((x) == DYNIBO_STATUS_OK)
#define INVALID(x) CHECK((x) == DYNIBO_STATUS_INVALID_ARGUMENT)

int main(int argc, char **argv) {
    CHECK(argc >= 2);
    CHECK(strcmp(dynibo_version(), "0.4.0") == 0);
    CHECK(dynibo_robot_name(NULL) == NULL);
    CHECK(dynibo_robot_joint_count(NULL) == 0);
    CHECK(dynibo_floating_robot_joint_count(NULL) == 0);

    DyniboRobot *fixed = NULL, *foreign = NULL;
    DyniboWorkspace *workspace = NULL, *foreign_workspace = NULL;
    OK(dynibo_robot_from_urdf(argv[1], &fixed));
    OK(dynibo_workspace_create(fixed, &workspace));
    OK(dynibo_robot_from_urdf(argv[1], &foreign));
    OK(dynibo_workspace_create(foreign, &foreign_workspace));
    size_t target = 0, n = dynibo_robot_joint_count(fixed);
    OK(dynibo_robot_link_id(fixed, "test_link_4", &target));
    CHECK(dynibo_robot_generalized_count(fixed) == n);
    double q[4] = {0}, qd[4] = {0}, qdd[4] = {0}, forces[4] = {0}, matrix[16] = {0}, jacobian[24] = {0};
    DyniboPose identity = {{0,0,0},{0,0,0,1}}, pose = identity;
    DyniboTwist motion = {{0,0,0},{0,0,0}};
    DyniboLoad load = {target, {0.1,0,0}, {0,1,0}};
    OK(dynibo_forward_kinematics(fixed, workspace, q, n, target, &pose));
    OK(dynibo_forward_velocity_kinematics(fixed, workspace, q, qd, n, target, &identity, &motion));
    OK(dynibo_forward_acceleration_kinematics(fixed, workspace, q, qd, qdd, n, target, &motion));
    OK(dynibo_jacobian(fixed, workspace, q, n, target, jacobian, 6*n));
    OK(dynibo_jacobian_derivative(fixed, workspace, q, qd, n, target, jacobian, 6*n));
    OK(dynibo_mass_matrix(fixed, workspace, q, n, matrix, n*n));
    OK(dynibo_velocity_product_forces(fixed, workspace, q, qd, n, forces, n));
    OK(dynibo_gravity(fixed, workspace, q, n, &load, 1, forces, n));
    OK(dynibo_inverse_dynamics(fixed, workspace, q, qd, qdd, n, &load, 1, forces, n));
    OK(dynibo_forward_dynamics(fixed, workspace, q, qd, n, forces, n, &load, 1, qdd, n));
    OK(dynibo_inverse_kinematics(fixed, workspace, q, n, target, &pose, dynibo_ik_options_default(), forces, n));
    INVALID(dynibo_mass_matrix(fixed, workspace, q, n, q, n*n));
    INVALID(dynibo_jacobian(fixed, workspace, q, n-1, target, jacobian, 6*n));
    INVALID(dynibo_forward_kinematics(fixed, foreign_workspace, q, n, target, &pose));
    INVALID(dynibo_gravity(fixed, workspace, q, n, NULL, 1, forces, n));
    INVALID(dynibo_forward_kinematics(NULL, workspace, q, n, target, &pose));
    CHECK(strlen(dynibo_last_error_message()) > 0);

    DyniboFloatingRobot *floating = NULL, *floating_foreign = NULL;
    DyniboFloatingWorkspace *floating_workspace = NULL, *floating_foreign_workspace = NULL;
    OK(dynibo_floating_robot_from_urdf(argv[1], &floating));
    OK(dynibo_floating_workspace_create(floating, &floating_workspace));
    OK(dynibo_floating_robot_from_urdf(argv[1], &floating_foreign));
    OK(dynibo_floating_workspace_create(floating_foreign, &floating_foreign_workspace));
    n = dynibo_floating_robot_joint_count(floating);
    OK(dynibo_floating_robot_link_id(floating, "test_link_4", &target));
    CHECK(dynibo_floating_robot_generalized_count(floating) == n + 6);
    DyniboBaseState base = {identity, {{0,0,0},{0,0,0}}, {{0,0,0},{0,0,0}}};
    DyniboPose floating_a, floating_b, floating_recovered;
    double floating_forces[10] = {0}, floating_acceleration[10] = {0}, floating_matrix[100] = {0}, floating_jacobian[60] = {0};
    OK(dynibo_floating_forward_kinematics(floating, floating_workspace, &base, q, n, target, &pose));
    floating_a = pose;
    base.frame.translation[0] = 0.25;
    OK(dynibo_floating_forward_kinematics(floating, floating_workspace, &base, q, n, target, &floating_b));
    base.frame.translation[0] = 0.0;
    OK(dynibo_floating_forward_kinematics(floating, floating_workspace, &base, q, n, target, &floating_recovered));
    CHECK(fabs(floating_b.translation[0] - floating_a.translation[0] - 0.25) < 1e-12);
    CHECK(fabs(floating_recovered.translation[0] - floating_a.translation[0]) < 1e-12);
    OK(dynibo_floating_forward_velocity_kinematics(floating, floating_workspace, &base, q, qd, n, target, &identity, &motion));
    OK(dynibo_floating_forward_acceleration_kinematics(floating, floating_workspace, &base, q, qd, qdd, n, target, &motion));
    OK(dynibo_floating_jacobian(floating, floating_workspace, &base, q, n, target, floating_jacobian, 6*(n+6)));
    OK(dynibo_floating_jacobian_derivative(floating, floating_workspace, &base, q, qd, n, target, floating_jacobian, 6*(n+6)));
    OK(dynibo_floating_mass_matrix(floating, floating_workspace, &base, q, n, floating_matrix, (n+6)*(n+6)));
    OK(dynibo_floating_velocity_product_forces(floating, floating_workspace, &base, q, qd, n, floating_forces, n+6));
    OK(dynibo_floating_gravity(floating, floating_workspace, &base, q, n, &load, 1, floating_forces, n+6));
    OK(dynibo_floating_inverse_dynamics(floating, floating_workspace, &base, q, qd, qdd, n, &load, 1, floating_forces, n+6));
    OK(dynibo_floating_forward_dynamics(floating, floating_workspace, &base, q, qd, n, floating_forces, n+6, &load, 1, floating_acceleration, n+6));
    INVALID(dynibo_floating_mass_matrix(floating, floating_workspace, &base, q, n, q, (n+6)*(n+6)));
    INVALID(dynibo_floating_forward_kinematics(floating, floating_foreign_workspace, &base, q, n, target, &pose));
    INVALID(dynibo_floating_forward_kinematics(floating, floating_workspace, NULL, q, n, target, &pose));
    base.velocity.angular[0] = NAN;
    INVALID(dynibo_floating_forward_kinematics(floating, floating_workspace, &base, q, n, target, &pose));

    dynibo_workspace_destroy(foreign_workspace); dynibo_robot_destroy(foreign);
    dynibo_workspace_destroy(workspace); dynibo_robot_destroy(fixed);
    dynibo_floating_workspace_destroy(floating_foreign_workspace); dynibo_floating_robot_destroy(floating_foreign);
    dynibo_floating_workspace_destroy(floating_workspace); dynibo_floating_robot_destroy(floating);
    return 0;
}
