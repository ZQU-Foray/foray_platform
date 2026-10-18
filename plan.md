# 开发计划

## 任务列表

- [x] 定义时间与时钟抽象（含 `use_sim_time` 语义）
- [x] 搭参数系统骨架（schema + 校验 + 热加载）
- [x] 定义 HAL 抽象接口（传感器 / 执行器 / 总线 / 设备生命周期）
- [ ] 接一个真实驱动跑通「一个传感器 → 一个话题」
  - Livox 驱动代码与 mock 测试已完成；真实 SDK 构建 + HIL 待部署验证
- [ ] 实现急停硬路径（独立实时线程 + 超时定时器）

### 任务 1 交付物

- `foray_platform/include/foray/platform/time/`
  - `Duration` / `Timestamp` 强类型；
  - `Clock` 接口 + `SystemClock` / `SteadyClock` / `ManualClock`；
  - 三个时钟域与 `use_sim_time` 就绪/跳变语义；
- 设计说明：[docs/time_and_clock.md](docs/time_and_clock.md)；
- 单测：`foray_platform/test/time/clock_test.cpp`。

### 任务 2 交付物

- `foray_platform/include/foray/platform/param/`
  - 类型化 schema、声明校验、范围/枚举校验；
  - 不可变快照 + 原子替换的存储；
  - 多来源分层（对应 `shared.yaml` + `<robot>.yaml`）；
  - YAML 文件来源（严格未知键、按 schema 解析类型）；
  - 初始加载 / 热加载门面，整批原子生效；
- 设计说明：[docs/parameter_system.md](docs/parameter_system.md)；
- 单测：`foray_platform/test/param/`。

### M1 交付物（HAL 前置）

- 包合并：`foray_platform_core/` → `foray_platform/`；
  `foray_platform_core` 与后续 `foray_platform_hal` 为同包两个 target；
- `foray_platform/include/foray/platform/result/`
  - `ErrorCode` / `Error` / `Result<T>` / `Result<void>`；
  - 驱动原始错误码保留在 `Error::vendor_code`；
- 单测：`foray_platform/test/result/result_test.cpp`；
- 决策记录：`decision.md` 的包合并、配置栈归属、通用错误类型条目。

### M2 交付物

- `foray_platform/include/foray/platform/container/spsc_queue.hpp`
  - 有界、无锁、单生产者/单消费者；
  - 容量向上取 2 的幂；满时 `try_push` 返回 false 并累计 `dropped()`；
  - `T` 要求可默认构造、移动构造/赋值不抛异常；
- 单测：`foray_platform/test/container/spsc_queue_test.cpp`；
- TSan 并发用例通过。

### M3 交付物

- `foray_platform/include/foray/platform/config/`
  - `ConfigDocument` / `ConfigLayer`：分层与可选层；
  - `ConfigResolver`：YAML 加载、顶层 `include`、`${ENV}` 替换、
    `FORAY_<PARAM>` 覆盖、来源追踪；
  - `ConfigSnapshot`：已校验值 + provenance；
  - `ConfigExporter`：dump / Markdown schema / JSON Schema；
  - `ConfigTool`：validate/dump/schema 门面（可执行文件待 HAL schema 落地）；
- 复用 `param` 的 `ParamSchema` / `ParamValue` / `ParamStore` 做校验与宽化；
- 设计说明：[docs/config_stack.md](docs/config_stack.md)；
- 单测：`foray_platform/test/config/`；ASan/UBSan 通过。

### M4/M5 交付物

- `foray_platform/include/foray/platform/hal/`
  - `Device` / `Camera` / `Lidar` 生命周期与读取接口；
  - `ImageFrame` / `PointCloudFrame` / `ImuSample` / `TimestampSync`；
  - `CameraConfig` / `LidarConfig` / `DeviceSelector` / `TriggerConfig`；
  - `FrameLease`（move-only）+ `FramePool`（最新帧邮箱、固定槽位、无锁读写）；
- 同包新增 `foray_platform_hal` interface target，导出 `foray_platform::hal`；
- 单测：类型/默认值、发布/获取、替换丢帧、槽位耗尽、租约晚于池销毁、
  单写单读并发；ASan/UBSan/TSan 通过；
- 设计说明：[docs/hal_architecture.md](docs/hal_architecture.md)。

### M6 交付物

- `foray_platform/include/foray/platform/hal/config_binder.hpp`
  - `make_camera_param_schema()` / `make_lidar_param_schema()`；
  - `bind_camera_config()` / `bind_lidar_config()`；
  - 跨字段约束：transport 与 serial/ip、trigger 与 frame_rate、像素格式支持；
- `foray_platform_hal` 从 interface 转为普通库；
- `test/hal/fake_sensor.hpp`：FakeCamera/FakeLidar，覆盖生命周期、超时、恢复；
- 单测：ConfigBinder + Fake 传感器；ASan/UBSan 通过。

### M9/M10 交付物（Livox）

- `foray_platform_driver_livox/`（纯 CMake，不依赖 ROS）
  - 公共工厂 `driver::livox::make_lidar(const hal::LidarConfig&)`，公共头无 SDK 类型；
  - `livox_api.hpp` 函数表抽象 + stub/真实实现，`FORAY_BUILD_DRIVER_LIVOX` 开关；
  - `LivoxRuntime`：进程级 SDK 引用计数、handle 路由、sink 墓地；
  - 原始包环（`SpscQueue<PacketSlot>`）+ 解析线程 + `FramePool` 组帧；
  - 包解析：0x01 高精度 / 0x02 低精度 / 0x03 球坐标 / 0x00 IMU；
  - 时间映射：`time_type` 0/1/2/3/4/未知 → `TimestampSync`；
  - JSON 生成（0600）、本机 IP 校验、多雷达、退避恢复；
- 文档：[docs/driver_livox.md](docs/driver_livox.md)、
  [docs/driver_deployment.md](docs/driver_deployment.md)；
- 单测：27 个用例，mock 覆盖生命周期、路由、恢复、解析、组帧；
  常规 / ASan+UBSan / TSan 全绿；安装后下游 `find_package` 通过；
- **未完成**：真实 SDK2 构建与 HIL（本机尚未安装 SDK2）、双回波 `0x11`。

## 当前 Agent State

M10_CODE_DONE_SDK_PENDING

## Before Snapshot

commit hash:   d40e647
branch:        feat/l1-foundation
modified files: 新增 foray_platform_driver_livox；更新 README/tree/decision/plan/ci/docs
risk level:    L1

## 模糊点与待确认项

- 本仓接口尚未冻结，任务清单为**方向性**的，落地顺序以 `foray_docs/algorithm_structure.md` §9 演进阶段为准
- 依赖的自研仓尚未建齐，跨仓任务需等对方 `README.md` 明确接口后再启动
- **ROS 适配层尚未开始**：`use_sim_time` 的映射已在文档中冻结，但 `/clock` 订阅与 `rclcpp` 适配留到后续单元
- **`package.xml` 的 `license` 字段为 `TODO`**，需组织确认许可证后填写
- 参数系统暂不支持序列参数与变更回调；配置栈的数组/动态列表支持按需扩展
- 配置解析栈按计划在 core 内扩展（`config/`）；机器人 YAML 归 `foray_robots`
- HAL 目标的驱动与 ROS 适配尚未开始，按 M2–M12 里程碑推进

## Vector Backend Status

Backend: Markdown
Status:  ready
Environment: 仓库内 Markdown 文档（无外部向量后端）
Index: 本仓 `README.md` / `tree.md` / `decision.md` / `docs/**`
Initialization: 2026-02-19
Commit: （首次提交）

## Acceptance Criteria

- [x] 每个新增模块带独立可执行测试（平台 127 个 + Livox 驱动 27 个用例）
- [x] 核心库可脱离 ROS 独立构建、测试、安装，并可被下游 `find_package` 消费
- [x] Livox 驱动在无 SDK 环境用 stub/mock 构建并测试全绿
- [ ] Livox 真实 SDK2 构建与 HIL（需要安装 SDK2 并连接雷达）
- [ ] 本仓能独立 `colcon build`（需在 ROS 2 环境/CI 验证）
- [ ] CI 绿灯
