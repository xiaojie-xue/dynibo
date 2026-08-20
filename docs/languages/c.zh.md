# C 指南

C 接口是 dynibo 的稳定 ABI。包含 `<dynibo/dynibo.h>`，并链接安装后的
`dynibo::dynibo` CMake target 或 `dynibo_c` 动态库。

## 命名

C 没有 namespace，因此所有导出函数使用 `dynibo_` 前缀，常量使用 `DYNIBO_`，
opaque object 类型和值结构体使用 `Dynibo` 前缀。与其他语言接口的对应关系参见
[API 对照](api-mapping.md)。

## 所有权

Robot 和 workspace handle 均为 opaque。Workspace 只能与创建它的 robot 一起
使用，不能跨模型混用。两个 handle 都需要显式释放：

```c
DyniboRobot *robot = NULL;
DyniboWorkspace *workspace = NULL;

/* 创建并使用 handle…… */

dynibo_workspace_destroy(workspace);
dynibo_robot_destroy(robot);
```

Destroy 函数接受 null。输入和输出数组始终由调用方持有。除非函数文档另有说明，
指针必须非 null，并且输出 buffer 不得与输入重叠。

## 错误处理

可能失败的函数返回 `DyniboStatus`。失败后，`dynibo_last_error_message()` 返回线程
局部的消息；它在同一线程下一次调用可能失败的 dynibo 函数之前有效：

```c
static int check(DyniboStatus status) {
    if (status == DYNIBO_STATUS_OK)
        return 1;
    fprintf(stderr, "dynibo: %s\n", dynibo_last_error_message());
    return 0;
}
```

如果需要长期保存消息，应复制字符串。下一次成功调用会清空该消息。

## Buffer 与 workspace

API 会验证输入和输出长度。关节状态数组长度使用
`dynibo_robot_joint_count()`，广义输出长度使用
`dynibo_robot_generalized_count()`。矩阵存储和浮动基顺序定义在
[关节与广义坐标](../user-guide/joint-and-generalized-coordinates.md)中。

Workspace 是可变的。每个并行计算需要独立 workspace，并且不能在计算过程中修改
robot 的基座状态。

## ABI 与版本检查

头文件定义了 `DYNIBO_VERSION_MAJOR`、`DYNIBO_VERSION_MINOR` 和
`DYNIBO_VERSION_PATCH`，`dynibo_version()` 在运行时返回实际链接的原生库版本。
部署时应使用同一个 dynibo release 的头文件和动态库；运行时版本字符串适合诊断和发现
部署错误。C ABI 不会在运行时协商不兼容的结构体布局。

## pkg-config 与动态库

非 CMake 构建可以使用安装的 pkg-config 元数据：

```bash
cc main.c $(pkg-config --cflags --libs dynibo)
```

Linux 动态库名为 `libdynibo_c.so`，macOS 为 `libdynibo_c.dylib`，Windows 为
`dynibo_c.dll`。动态库遵循各平台的常规 loader 规则：安装到标准搜索位置、设置合适的
运行时搜索路径，或在平台支持时随应用一起部署。

[打开 C API 参考](../c-api/dynibo_8h.md){ .md-button }
