# 复用输出缓冲区

预先创建连续的 float64 输入、扁平输出数组和原生载荷缓冲区，在计算中重复使用。
返回值就是传入的 `out` 对象，reshape 后的雅可比可以共享原数组存储。
代码在每个角度验证缓冲区身份与数值结果。

安装已发布的包，然后保存并运行下面的完整 Python 脚本。代码会在临时目录创建 URDF，
无需克隆仓库、下载 mesh 或安装 ROS。

```bash
python -m pip install dynibo
python reuse_buffers.py
```

```python
--8<-- "examples/python/recipes/reuse_buffers.py"
```

## 结果与约定

这里复用原生计算存储和输出数组。Python 标量、值对象、函数调用与示例中的断言仍可能
分配内存，不能据此声称整个 Python 循环没有分配。使用 `robot.load_buffer()` 可以避免
每次调用都重新创建并转换 `Load` 列表。

[Python API 参考](../reference/python.md) · [参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)
