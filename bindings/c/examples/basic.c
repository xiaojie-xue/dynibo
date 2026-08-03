#include <dyno/dyno.h>

#include <stdio.h>
#include <stdlib.h>

static void check(DynoStatus status) {
    if (status != DYNO_STATUS_OK) {
        fprintf(stderr, "dyno: %s\n", dyno_last_error_message());
        exit(1);
    }
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s ROBOT.urdf TOOL_LINK\n", argv[0]);
        return 2;
    }
    DynoRobot *robot = NULL;
    DynoWorkspace *workspace = NULL;
    check(dyno_robot_load_urdf(argv[1], &robot));
    check(dyno_workspace_create(robot, &workspace));

    const size_t n = dyno_robot_joint_count(robot);
    double *q = (double *)calloc(n, sizeof(double));
    size_t target = 0;
    DynoPose pose;
    check(dyno_robot_link_id(robot, argv[2], &target));
    check(dyno_forward_kinematics(robot, workspace, q, n, target, &pose));
    printf("%s: translation = [%.6f, %.6f, %.6f]\n",
           dyno_robot_name(robot), pose.translation[0],
           pose.translation[1], pose.translation[2]);

    free(q);
    dyno_workspace_destroy(workspace);
    dyno_robot_destroy(robot);
    return 0;
}
