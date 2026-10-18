# foray_platform

> **L1** · owner 角色：平台

L1 平台基础 + L2 下行 HAL

零业务语义，被所有层依赖，**最稳定**。

---

## 快速开始

> 当前已落地 L1 的时间/时钟抽象与参数系统（`foray_platform_core`，**不依赖 ROS**）。
> L2 HAL 与 ROS 适配层尚未开始。

### 仅构建核心库（无需 ROS）

依赖：CMake ≥ 3.16、C++17 编译器、yaml-cpp；测试另需 GTest。

```bash
cmake -S foray_platform_core -B ~/.cache/foray-platform-build/core \
      -DCMAKE_BUILD_TYPE=Release
cmake --build ~/.cache/foray-platform-build/core -j"$(nproc)"
ctest --test-dir ~/.cache/foray-platform-build/core --output-on-failure
```

> ⚠️ 本仓位于 SMB 共享盘，构建目录必须放本地盘（共享盘不支持符号链接）。
> 这是 `foray_ws` 元仓构建约定的同一条纪律。

### 在 ROS 2 工作空间中构建

```bash
# 1. 拉取（仓齐备后改为从元仓 foray_ws 一键拉取）
git clone git@github.com:ZQU-Foray/foray_platform.git
cd foray_platform

# 2. 依赖安装
rosdep install -y -r --from-paths . --ignore-src --rosdistro humble

# 3. 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. 测试
colcon test --event-handlers console_direct+

# 5. 运行
# （待装配根就绪后补充）
```

## 依赖

| 类型 | 依赖 |
|---|---|
| **自研仓** | X2 |
| **第三方** | yaml-cpp（参数热加载）· GTest（仅测试）· Eigen3 · ROS 2 Humble · 各传感器官方 SDK（Livox / Hikvision 等） |

## 接口

**L1 基础设施**

| 模块 | 内容 |
|---|---|
| 时间与时钟 | 统一时钟源 · `use_sim_time` 行为一致（**已实现**，见 [设计](docs/time_and_clock.md)） |
| 数学/几何/滤波 | EKF · PID · 坐标变换 |
| 数据结构 | 无锁队列 · 线程池 |
| 参数系统 | 类型化 schema + 校验 + **热加载**（**已实现**，见 [设计](docs/parameter_system.md)） |
| 错误码 | 结果类型 · 生命周期 |
| 日志/录制/回放 | 基础设施（X4 的底座） |
| 状态编码器 | **纯函数**，训练与部署共用 |

**L2 下行 HAL**（依赖倒置：抽象在本仓，具体驱动放本仓之下的插件目录）

| 模块 | 内容 |
|---|---|
| 传感器 | 激光 · IMU · 相机 |
| 执行器 | 底盘 · 云台 · 发射机构 |
| 总线 | 串口 · CAN · USB |
| 设备生命周期 | `open` / `close` / `recover`，`ERROR` 态尽量自修复 |

**急停硬路径放本仓**：最终安全降级（`cmd_vel` 超时归零、发射机构断电）**不依赖中间件**，
位于最靠近执行器的执行器抽象内部，独立实时线程 + 超时定时器。

## 纪律



- **零业务语义**——不知道该车是哨兵还是步兵

- **不直接到处使用 ROS 2**——否则无法单测、无法去 ROS 化、无法换中间件

- 依赖方向恒为「**实现依赖抽象**」：抽象接口在本仓，具体驱动在本仓之下的插件目录

- 接口稳定；生产构建可关闭可观测实现，但**接口不变**

## 上下游

| 方向 | 对象 |
|---|---|
| **上游**（本仓依赖谁） | `foray_interfaces` |
| **下游**（谁依赖本仓） | `foray_localization` · `foray_vision` · `foray_auto_aim` · `foray_navigation` · `foray_decision` · `foray_robots` · `foray_ws` |

## 设计文档

- [时间与时钟](docs/time_and_clock.md)——时钟域、`use_sim_time` 语义、急停时间源
- [参数系统](docs/parameter_system.md)——schema、校验、YAML、热加载与线程模型

## 参考

- [算法结构](https://github.com/ZQU-Foray/foray_docs/blob/main/algorithm_structure.md)
- [仓库结构](https://github.com/ZQU-Foray/foray_docs/blob/main/repository_structure.md)
- [组织贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)
