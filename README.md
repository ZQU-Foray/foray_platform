# foray_platform

> **L1** · owner 角色：平台

L1 平台基础 + L2 下行 HAL

零业务语义，被所有层依赖，**最稳定**。

---

## 快速开始

> 当前已落地 L1 的时间/时钟、参数系统与通用错误码（`foray_platform` 包，**不依赖 ROS**）。
> L2 HAL 与 ROS 适配层正在按计划落地。

### 仅构建核心库（无需 ROS）

依赖：CMake ≥ 3.16、C++17 编译器、yaml-cpp；测试另需 GTest。

```bash
cmake -S foray_platform -B ~/.cache/foray-platform-build/platform \
      -DCMAKE_BUILD_TYPE=Release
cmake --build ~/.cache/foray-platform-build/platform -j"$(nproc)"
ctest --test-dir ~/.cache/foray-platform-build/platform --output-on-failure
```

> ⚠️ 本仓位于 SMB 共享盘，构建目录必须放本地盘（共享盘不支持符号链接）。
> 这是 `foray_ws` 元仓构建约定的同一条纪律。

### 构建 Livox 驱动（可选）

驱动包默认 **不依赖 SDK**，用 stub 构建并跑 mock 测试：

```bash
cmake -S foray_platform_driver_livox -B ~/.cache/foray-platform-build/livox \
      -DCMAKE_PREFIX_PATH="$HOME/.cache/foray-platform-prefix" \
      -DFORAY_BUILD_DRIVER_LIVOX=OFF
cmake --build ~/.cache/foray-platform-build/livox -j"$(nproc)"
ctest --test-dir ~/.cache/foray-platform-build/livox --output-on-failure
```

真实硬件构建需先把 Livox SDK2 安装到私有前缀（推荐由
`foray_ws/scripts/build_livox_sdk2.sh` 完成），再打开开关：

```bash
cmake -S foray_platform_driver_livox -B ~/.cache/foray-platform-build/livox-on \
      -DCMAKE_PREFIX_PATH="$HOME/.cache/foray-platform-prefix" \
      -DFORAY_BUILD_DRIVER_LIVOX=ON \
      -DFORAY_LIVOX_SDK_PROVIDER=PREBUILT \
      -DLIVOX_SDK2_ROOT="$HOME/.cache/foray-platform-prefix/livox-sdk2" \
      -DFORAY_LIVOX_BUILD_TOOLS=ON
```

- `FORAY_LIVOX_SDK_PROVIDER=SYSTEM`：从系统路径 / `CMAKE_PREFIX_PATH` 查找；
- `PREBUILT`：只接受 `LIVOX_SDK2_ROOT` 前缀下的 SDK；
- 安装后的 target 导出 `FORAY_DRIVER_LIVOX_HAS_SDK=0/1`，下游用
  `#if FORAY_DRIVER_LIVOX_HAS_SDK` 判断能力；
- `FORAY_LIVOX_BUILD_TOOLS=ON` 构建 `foray_livox_dump`，可写 PCD 与俯视 PPM，
  也可用 `--stream` 把点云逐点流式写到 stdout，用于手动查看与管道消费。

详见 [驱动部署](docs/driver_deployment.md) 与
[手动测试](docs/driver_livox.md#12-手动测试与点云查看)。

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
| **第三方** | yaml-cpp（参数热加载）· GTest（仅测试）· Eigen3 · ROS 2 Humble · Livox SDK2（可选，驱动编译开关）· Hikvision MVS（可选，待落地） |

## 接口

> 本仓是 **C++ 库**，不是 ROS 节点。这里的「接口」指**公共 C++ API**；
> `foray_platform` 的 core target 不直接定义 ROS 话题/服务。需要跨进程的契约由
> `foray_interfaces`（X2）定义；后续 ROS 适配层落地后再在本节补充话题/参数清单。
>
> 未冻结的接口只描述**契约方向**，不承诺具体签名。

### 已实现：L1 公共 API

| 头文件 | 命名空间 | 公共类型 / 函数 | 契约 |
|---|---|---|---|
| `foray/platform/time/time.hpp` | `foray::platform::time` | `Duration` · `Timestamp` · `Clock` · `ClockDomain` · `SystemClock` · `SteadyClock` · `ManualClock` | 纳秒精度；时间点只在同一时钟域内比较；仿真时钟未就绪时 `now()` 为零且 `is_ready() == false` |
| `foray/platform/param/param.hpp` | `foray::platform::param` | `ParamType` · `ParamValue` · `ParamSpec` · `ParamSchema` · `ParamSnapshot` · `ParamStore` · `ParamSource` · `MapParamSource` · `YamlFileParamSource` · `ParamManager` · `ParamResult` / `ParamError` | 类型化 schema + 校验 + 热加载；读路径为不可变快照，写路径在非实时线程整批原子生效 |
| `foray/platform/result/result.hpp` | `foray::platform` | `ErrorCode` · `Error` · `Result<T>` · `Result<void>` | 公共 API 不抛异常；驱动原始错误码保存在 `Error::vendor_code` |
| `foray/platform/container/spsc_queue.hpp` | `foray::platform::container` | `SpscQueue<T>` | 有界、无锁、单生产者/单消费者；满时 `try_push` 返回 false 并累计 `dropped()` |
| `foray/platform/config/config.hpp` | `foray::platform::config` | `ConfigDocument` · `ConfigResolver` · `ConfigSnapshot` · `ConfigExporter` · `ConfigTool` · `MapEnvProvider` | 分层 YAML、include、`${ENV}`、`FORAY_<PARAM>` 覆盖、provenance；复用 `param` 做校验 |
| `foray/platform/hal/hal.hpp` | `foray::platform::hal` | `Device` · `Camera` · `Lidar` · `ImageFrame` · `PointCloudFrame` · `ImuSample` · `FrameLease` · `FramePool` · `CameraConfig` / `LidarConfig` | 单写单读、最新帧邮箱；SDK 边界一次拷贝，租约之后零拷贝；时间戳带 `TimestampSync` |
| `foray/platform/hal/config_binder.hpp` | `foray::platform::hal` | `make_camera_param_schema` · `make_lidar_param_schema` · `bind_camera_config` · `bind_lidar_config` | 复用 `param` schema；跨字段校验（transport/serial/trigger）；不探测设备能力 |

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
| 数据结构 | `SpscQueue<T>`（HAL 前置）、`LockFreeQueue<T>`、`ThreadPool` | 进行中 |
| 日志/录制/回放 | X4 底座：日志、录制、回放接口 | 未开始 |
| 状态编码器 | 纯函数，训练与部署共用 | 未开始 |

### L2 下行 HAL

> 依赖倒置：抽象在本仓，具体驱动放本仓之下的插件目录；抽象层不得出现 ROS 类型。
> 设计说明：[docs/hal_architecture.md](docs/hal_architecture.md)。

| 抽象接口 | 职责 | 契约要点 | 状态 |
|---|---|---|---|
| 设备生命周期 | `open` / `close` / `recover` | `ERROR` 态尽量自修复；状态可查询 | **接口已实现** |
| 传感器 | 激光 · IMU · 相机 | 只暴露类型化数据读取接口，不暴露厂商 SDK 类型 | **接口与帧池已实现**；驱动进行中 |
| 执行器 | 底盘 · 云台 · 发射机构 | 指令 + 安全状态；内置超时归零 | 未开始 |
| 总线 | 串口 · CAN · USB | 字节流/帧读写，设备无关 | 未开始 |

### 驱动包

| 包 | 接口 | 状态 |
|---|---|---|
| `foray_platform_driver_livox` | `foray::platform::driver::livox::make_lidar(const hal::LidarConfig&)` | **代码、mock 测试与实机 HIL 已完成**（Mid-360s） |
| `foray_platform_driver_hikvision` | `foray::platform::driver::hikvision::make_camera(const hal::CameraConfig&)` | 方案已定、待实施（M7/M8），见 [接入与封装方案](docs/driver_hikvision.md) |

Livox 驱动特性：

- Livox SDK2 进程级单例 + 引用计数，多雷达按 handle 路由；
- 同时支持 Mid-360（dev_type 9）与 Mid-360s（dev_type 35）；
- SDK 回调只做长度校验、时间戳与一次拷贝，解析/组帧在独立线程；
- 支持高精度/低精度/球坐标点云与 IMU；双回波 `0x11` 显式返回 `kUnsupportedFormat`；
- 非重复扫描按 `frame_duration_ms` 时间切帧，IMU 加速度由 g 换算为 m/s²；
- `time_type` → `TimestampSync` 映射，NoSync/PPS 不伪造绝对时间；
- 默认单播，组播作为可选自定义 JSON；
- 编译期 `FORAY_BUILD_DRIVER_LIVOX=ON/OFF`；OFF 时 `open()` 返回 `kSdkUnavailable`；
- `FORAY_LIVOX_BUILD_TOOLS=ON` 提供 `foray_livox_dump`
  （PCD + 俯视 PPM；`--stream` 逐点流式 stdout，统计走 stderr）与
  `foray_livox_config`（读回 → 下发雷达 IP/姿态/主机端口 → 再读回校验）。

**雷达网络自举**：作为硬件抽象层的一部分，本仓必须能一键把物理网口挂进
雷达网桥并配置主机地址，而不要求操作手现场 `sudo ip link`：

- 只读探测拓扑（`foray/platform/net/net_types.hpp`）→ 幂等规划
  （`plan_link`）→ 授权执行（`ensure_link`，失败自动回滚）；
- NetworkManager 优先（创建 `foray-<iface>-slave` profile，重启不丢），
  没有 NM 时回退 rtnetlink；
- 特权只来自 polkit：`foray-net-helper` 装到 libexec，动作
  `org.foray.netconfig.bootstrap` 只放行该程序；
- 雷达参数下发走 `foray_livox_config`，默认只读回校验，`--set-lidar-ip`
  等显式开关才改写，写后必读回。

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
- [统一配置解析栈](docs/config_stack.md)——分层、include、环境变量、来源追踪与导出
- [L2 HAL 架构](docs/hal_architecture.md)——生命周期、线程模型、帧池与零拷贝边界
- [传感器驱动封装调研](docs/hal_sensor_driver_research.md)——海康 MVS 与 Livox SDK2 的 HAL 封装方案
- [Livox 驱动](docs/driver_livox.md)——运行时、包解析、组帧、时间同步与多雷达
- [Hikvision 驱动方案](docs/driver_hikvision.md)——MVS 接入序列、`MvsApi` 隔离、取流线程、配置映射与时间戳
- [驱动部署](docs/driver_deployment.md)——SDK2 构建、网络、PPS/PTP 与运行环境
- [厂商 SDK 打包调研](docs/sdk_packaging_survey.md)——RM 战队如何打包与调用 MVS 等闭源相机 SDK（横评）
- [雷达网络自举](docs/net_bootstrap.md)——拓扑探测、NM/netlink 双后端、polkit 与回滚

## 参考

- [算法结构](https://github.com/ZQU-Foray/foray_docs/blob/main/algorithm_structure.md)
- [仓库结构](https://github.com/ZQU-Foray/foray_docs/blob/main/repository_structure.md)
- [组织贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)
