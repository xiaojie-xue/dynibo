# Quick Start

All interfaces follow the same workflow:

1. Load a robot from URDF.
2. Resolve a target link by name.
3. Create a joint-position vector in non-fixed URDF joint order.
4. Run a calculation.

=== "Rust"

    ```rust
    use dynibo::Robot;

    fn main() -> dynibo::Result<()> {
        let mut robot = Robot::from_urdf("robot.urdf")?;
        let target = robot.link_id("tool")?;
        let q = vec![0.0; robot.joint_count()];

        let pose = robot.forward_kinematics(&q, target)?;
        println!("translation: {}", pose.translation.vector.transpose());
        Ok(())
    }
    ```

=== "Python"

    ```python
    from dynibo import Robot

    with Robot.from_urdf("robot.urdf") as robot:
        target = robot.link_id("tool")
        q = [0.0] * robot.joint_count

        pose = robot.forward_kinematics(q, target)
        print("translation:", pose.translation)
    ```

=== "C++"

    ```cpp
    #include <dynibo/dynibo.hpp>

    #include <iostream>
    #include <vector>

    int main() {
        try {
            dynibo::Robot robot("robot.urdf");
            const auto target = robot.link_id("tool");
            const std::vector<double> q(robot.joint_count(), 0.0);

            const auto pose = robot.forward_kinematics(q, target);
            std::cout << "x: " << pose.translation[0] << '\n';
        } catch (const dynibo::Error& error) {
            std::cerr << "dynibo: " << error.what() << '\n';
            return 1;
        }
    }
    ```

=== "C"

    ```c
    #include <dynibo/dynibo.h>

    #include <stdio.h>
    #include <stdlib.h>

    int main(void) {
        DyniboRobot *robot = NULL;
        DyniboWorkspace *workspace = NULL;
        size_t target = 0;
        DyniboPose pose;

        if (dynibo_robot_from_urdf("robot.urdf", &robot) != DYNIBO_STATUS_OK ||
            dynibo_workspace_create(robot, &workspace) != DYNIBO_STATUS_OK ||
            dynibo_robot_link_id(robot, "tool", &target) != DYNIBO_STATUS_OK) {
            fprintf(stderr, "dynibo: %s\n", dynibo_last_error_message());
            dynibo_workspace_destroy(workspace);
            dynibo_robot_destroy(robot);
            return 1;
        }

        const size_t n = dynibo_robot_joint_count(robot);
        double *q = calloc(n, sizeof(*q));
        const DyniboStatus status = dynibo_forward_kinematics(
            robot, workspace, q, n, target, &pose);

        if (status == DYNIBO_STATUS_OK)
            printf("x: %f\n", pose.translation[0]);
        else
            fprintf(stderr, "dynibo: %s\n", dynibo_last_error_message());

        free(q);
        dynibo_workspace_destroy(workspace);
        dynibo_robot_destroy(robot);
        return status == DYNIBO_STATUS_OK ? 0 : 1;
    }
    ```

The examples differ in resource and error handling, not in numerical meaning.
Continue with [API Mapping](../languages/api-mapping.md) to understand that mapping,
then read [Frames and Spatial Vectors](../user-guide/frames-and-spatial-vectors.md) before
using matrix or spatial-vector results.
