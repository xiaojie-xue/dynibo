#include <dynibo/dynibo.h>

#include <stdio.h>
#include <stdlib.h>

static void check(DyniboStatus status) {
    if (status != DYNIBO_STATUS_OK) {
        fprintf(stderr, "dynibo: %s\n", dynibo_last_error_message());
        exit(1);
    }
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s ROBOT.urdf TOOL_LINK\n", argv[0]);
        return 2;
    }
    DyniboRobot *robot = NULL;
    DyniboWorkspace *workspace = NULL;
    check(dynibo_robot_load_urdf(argv[1], &robot));
    check(dynibo_workspace_create(robot, &workspace));

    const size_t n = dynibo_robot_joint_count(robot);
    double *q = (double *)calloc(n, sizeof(double));
    size_t target = 0;
    DyniboPose pose;
    check(dynibo_robot_link_id(robot, argv[2], &target));
    check(dynibo_forward_kinematics(robot, workspace, q, n, target, &pose));
    printf("%s: translation = [%.6f, %.6f, %.6f]\n",
           dynibo_robot_name(robot), pose.translation[0],
           pose.translation[1], pose.translation[2]);

    free(q);
    dynibo_workspace_destroy(workspace);
    dynibo_robot_destroy(robot);
    return 0;
}
