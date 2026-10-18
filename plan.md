# 开发计划

## 任务列表

- [x] 定义时间与时钟抽象（含 `use_sim_time` 语义）
- [x] 搭参数系统骨架（schema + 校验 + 热加载）
- [ ] 定义 HAL 抽象接口（传感器 / 执行器 / 总线 / 设备生命周期）
- [ ] 接一个真实驱动跑通「一个传感器 → 一个话题」
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

## 当前 Agent State

M5_DONE

## Before Snapshot

commit hash:   52619e5
branch:        feat/l1-foundation
modified files: 包合并（foray_platform_core → foray_platform）；新增 result/；更新 README/tree/decision/ci/plan
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

- [x] 每个新增模块带独立可执行测试（112 个用例）
- [x] 核心库可脱离 ROS 独立构建、测试、安装，并可被下游 `find_package` 消费
- [ ] 本仓能独立 `colcon build`（需在 ROS 2 环境/CI 验证）
- [ ] CI 绿灯
