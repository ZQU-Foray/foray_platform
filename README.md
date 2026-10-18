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
| **自研仓** | `foray_interfaces`（X2 接口契约层） |
| **第三方** | yaml-cpp（参数热加载）· GTest（仅测试）· Eigen3 · ROS 2 Humble · 各传感器官方 SDK（Livox / Hikvision 等） |

## 接口

> 本仓是 **C++ 库**，不是 ROS 节点。这里的「接口」指**公共 C++ API**；
> `foray_platform_core` 不直接定义 ROS 话题/服务。需要跨进程的契约由
> `foray_interfaces`（X2）定义；后续 ROS 适配层落地后再在本节补充话题/参数清单。
>
> 未冻结的接口只描述**契约方向**，不承诺具体签名。

### 已实现：L1 公共 API

| 头文件 | 命名空间 | 公共类型 / 函数 | 契约 |
|---|---|---|---|
| `foray/platform/time/time.hpp` | `foray::platform::time` | `Duration` · `Timestamp` · `Clock` · `ClockDomain` · `SystemClock` · `SteadyClock` · `ManualClock` | 纳秒精度；时间点只在同一时钟域内比较；仿真时钟未就绪时 `now()` 为零且 `is_ready() == false` |
| `foray/platform/param/param.hpp` | `foray::platform::param` | `ParamType` · `ParamValue` · `ParamSpec` · `ParamSchema` · `ParamSnapshot` · `ParamStore` · `ParamSource` · `MapParamSource` · `YamlFileParamSource` · `ParamManager` · `ParamResult` / `ParamError` | 类型化 schema + 校验 + 热加载；读路径为不可变快照，写路径在非实时线程整批原子生效 |

**时钟域与时间源**（详见 [设计](docs/time_and_clock.md)）：

| 域 | 实现 | 用途 |
|---|---|---|
| `kSystemWall` | `SystemClock` | 消息时间戳、与外部系统对时 |
| `kSteady` | `SteadyClock` | 间隔测量、实时循环、**急停/看门狗** |
| `kSimulated` | `ManualClock` | `use_sim_time=true`、回放、单测 |

### 规划中：L1 模块

> 以下为方向性接口，落地时在本节替换为具体头文件与类型。

| 模块 | 预期接口形态 | 状态 |
|---|---|---|
| 数学/几何/滤波 | 无 ROS 的纯函数与类：EKF、PID、坐标变换 | 未开始 |
| 数据结构 | `LockFreeQueue<T>`、`ThreadPool` | 未开始 |
| 错误码 | `Result<T>` / `ErrorCode`，生命周期与错误上下文 | 未开始 |
| 日志/录制/回放 | X4 底座：日志、录制、回放接口 | 未开始 |
| 状态编码器 | 纯函数，训练与部署共用 | 未开始 |

### 规划中：L2 下行 HAL

> 依赖倒置：抽象在本仓，具体驱动放本仓之下的插件目录；抽象层不得出现 ROS 类型。

| 抽象接口 | 职责 | 契约要点 | 状态 |
|---|---|---|---|
| 设备生命周期 | `open` / `close` / `recover` | `ERROR` 态尽量自修复；状态可查询 | 未开始 |
| 传感器 | 激光 · IMU · 相机 | 只暴露类型化数据读取接口，不暴露厂商 SDK 类型 | 未开始 |
| 执行器 | 底盘 · 云台 · 发射机构 | 指令 + 安全状态；内置超时归零 | 未开始 |
| 总线 | 串口 · CAN · USB | 字节流/帧读写，设备无关 | 未开始 |

**急停硬路径**：最终安全降级（`cmd_vel` 超时归零、发射机构断电）**不依赖中间件**，
位于最靠近执行器的执行器抽象内部，使用独立实时线程 + 超时定时器，
时间源固定为 `SteadyClock`（见 [时间与时钟](docs/time_and_clock.md)）。

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
