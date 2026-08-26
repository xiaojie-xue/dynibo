# 入门示例

所有语言接口都遵循相同的工作流程：

1. 从 URDF 加载机器人。
2. 根据名称查找目标 link。
3. 按 URDF 中非固定关节的顺序创建关节位置向量。
4. 执行计算。

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

这些示例的区别在资源管理和错误处理方式，而不是数值含义。接下来可以阅读
[API 对照](../languages/api-mapping.md)了解接口对应关系，并在使用矩阵或空间向量结果前
阅读[参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)。
