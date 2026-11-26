# C 指南

C 接口是 dynibo 的稳定 ABI。包含 `<dynibo/dynibo.h>`，并链接安装后的
`dynibo::dynibo` CMake target 或 `dynibo_c` 动态库。

## 命名

C 语言没有命名空间，因此所有导出函数使用 `dynibo_` 前缀，常量使用 `DYNIBO_`，
不透明对象类型和值结构体使用 `Dynibo` 前缀。与其他语言接口的对应关系参见
[API 对照](api-mapping.md)。

## 所有权

机器人句柄与 workspace 句柄均为不透明类型。Workspace 只能与创建它的机器人句柄一起
使用，不能跨模型混用。两个句柄都需要显式释放：

```c
DyniboRobot *robot = NULL;
DyniboWorkspace *workspace = NULL;

/* 创建并使用句柄…… */

dynibo_workspace_destroy(workspace);
dynibo_robot_destroy(robot);
```

销毁函数接受空指针（`NULL`）。输入和输出数组始终由调用方持有。除非函数文档另有说明，
指针参数不得为空，且输出缓冲区不得与任何输入重叠。

## 错误处理

可能失败的函数返回 `DyniboStatus`。失败后，`dynibo_last_error_message()` 返回线程局部的
错误消息；该字符串在同一线程再次调用可能失败的 dynibo 函数前保持有效：

```c
static int check(DyniboStatus status) {
    if (status == DYNIBO_STATUS_OK)
        return 1;
    fprintf(stderr, "dynibo: %s\n", dynibo_last_error_message());
    return 0;
}
```

如果需要长期保存消息，请先复制字符串。下一次成功调用会清空该消息。

## 缓冲区与 workspace

API 会严格校验输入和输出长度。关节状态数组长度使用 `dynibo_robot_joint_count()`，
广义输出长度使用 `dynibo_robot_generalized_count()`。矩阵存储和浮动基座顺序定义在
[关节与广义坐标](../user-guide/joint-and-generalized-coordinates.md)中。

Workspace 是可变的，每个并行计算都需要独立的 workspace。固定基座世界位姿请通过
`dynibo_workspace_set_base_frame()` 按 workspace 配置（`dynibo_robot_set_base_frame()`
已弃用），重力加速度通过 `dynibo_workspace_set_gravity()` /
`dynibo_floating_workspace_set_gravity()` 按 workspace 配置。`DyniboFloatingRobot`
不保存基座状态，而是在每次计算时传入 `DyniboBaseState`。

## 浮动基座

浮动基座机器人与 workspace 使用独立的 C 类型：

```c
DyniboFloatingRobot *robot = NULL;
DyniboFloatingWorkspace *workspace = NULL;
check(dynibo_floating_robot_from_urdf("robot.urdf", &robot));
check(dynibo_floating_workspace_create(robot, &workspace));

DyniboBaseState base = {0};
base.frame.rotation_xyzw[3] = 1.0;
size_t target = 0;
check(dynibo_floating_robot_link_id(robot, "tool", &target));
check(dynibo_floating_forward_kinematics(
    robot, workspace, &base, q, joint_count, target, &pose));

dynibo_floating_workspace_destroy(workspace);
dynibo_floating_robot_destroy(robot);
```

浮动基座的 `generalized_count` 为 `joint_count + 6`；广义输出依次为世界坐标系下的
角分量、线分量，以及各关节分量。

## ABI 与版本检查

头文件定义了 `DYNIBO_VERSION_MAJOR`、`DYNIBO_VERSION_MINOR` 和
`DYNIBO_VERSION_PATCH`，`dynibo_version()` 在运行时返回实际链接的原生库版本。
部署时应确保头文件和动态库来自同一个 dynibo 版本；运行时版本字符串便于排查部署不一致
问题。C ABI 不会在运行时协商不兼容的结构体布局。

## pkg-config 与动态库

非 CMake 构建可以使用已安装的 pkg-config 元数据：

```bash
cc main.c $(pkg-config --cflags --libs dynibo)
```

动态库文件名在 Linux 上为 `libdynibo_c.so`，macOS 上为 `libdynibo_c.dylib`，Windows 上为
`dynibo_c.dll`。动态库遵循各平台的常规加载规则：安装到标准搜索目录、配置合适的运行时
搜索路径，或在平台支持时与可执行程序一同部署。

[打开 C API 参考](../c-api/dynibo_8h.md){ .md-button }
