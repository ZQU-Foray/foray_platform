# 开发计划

## 任务列表

- [x] 定义时间与时钟抽象（含 `use_sim_time` 语义）
- [x] 搭参数系统骨架（schema + 校验 + 热加载）
- [x] 定义 HAL 抽象接口（传感器 / 执行器 / 总线 / 设备生命周期）
- [ ] 接一个真实驱动跑通「一个传感器 → 一个话题」
  - Livox 驱动代码与 mock 测试已完成；真实 SDK 构建 + HIL 已通过
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

### M6+ 相机配置扩展（2026-10-24，已完成）

- 目标：上游 robots 仓只写 conf，就能覆盖相机的设备选择、几何、采集、触发、传输、
  缓冲与时间语义；设计见 [docs/camera_config.md](docs/camera_config.md)；
- `hal/types.hpp` 新增 `AutoMode` / `TimestampSource`；`CameraConfig` 扩到 40 个键，
  新增 binning/decimation、自动曝光/增益及其上下限、包长/包间隔/心跳、
  帧池内存上限、`timestamp_source`；`TriggerConfig` 增加显式 `source`；
- `ParamManager` 增加变更监听：`add_change_listener` / `remove_change_listener`、
  `ParamChange`、`diff_snapshots` 增量比对、锁外派发、回调异常隔离；
- `Camera` 新增非纯虚 `apply(CameraTuning)` / `trigger()` / `supported_keys()`，
  子类不实现也能编译（默认 `kNotSupported`）；
- 单测：`config_binder_test` +5、`manager_test` +5，全套 163 个用例通过；
- 未做：驱动侧填充 `supported_keys()`、`SensorInfo` 补同步可观测字段。

### M7/M8 交付物（Hikvision）

- **方案已定，代码未开工**；设计与取舍见
  [docs/driver_hikvision.md](docs/driver_hikvision.md)。
- 隔离方式沿用 Livox：`detail::MvsApi` 虚接口，`src/hikvision/mvs_api_real.cpp`
  是唯一 include `MvCameraControl.h` 的翻译单元；公共头只有 HAL 类型；
  `FORAY_BUILD_DRIVER_HIKVISION=ON/OFF` + `FORAY_MVS_ROOT`，ON 找不到 SDK 直接
  `FATAL_ERROR`；core/backend 两个 OBJECT 目标，mock 测试不链 MVS。
- 取流模型不同于 Livox：`MV_CC_GetImageBuffer` 是阻塞调用，取流线程即生产者，
  链路为 `GetImageBuffer → memcpy 到 FramePool → FreeImageBuffer`（同线程），
  不需要原始包环与解析线程；超时 100 ms 走正常控制流，不 `sleep` 轮询。
- 设备选择严格按 HAL 约定（serial > user_defined_name > model+index > ip）：
  空 selector 返回 `kInvalidArgument`，多候选返回 `kDeviceAmbiguous`，不默认取第 0 个。
- 时间戳：只用 `nDevTimeStampHigh/Low` + 标定偏移和/或主机接收时刻；
  `nHostTimeStamp` 单位未定义，不使用。
- 实施清单（未开始）：
  - 补齐 HAL 参数缺口：camera schema 与 binder 的 `device_to_host_offset_ns` /
    `exposure_to_host_latency_ns`、`trigger.debounce_us`；
  - `foray_platform_driver_hikvision/`：`camera.hpp` 公共工厂、`camera.cpp`
    生命周期与取流、`mvs_nodes.cpp` 节点映射、`timestamp.cpp` 时间换算、stub；
  - 单测：生命周期、配置映射顺序（`SetImageNodeNum` 早于 `StartGrabbing`）、
    时间三条路径、超时/掉线/recover、帧池计数；backend OFF 断言 `kSdkUnavailable`；
  - `foray_hikvision_dump` 工具与 HIL（1000 帧、拔插恢复、30 分钟长跑）；
  - 部署补进 `docs/driver_deployment.md` §4。

### M9/M10 交付物（Livox）

- `foray_platform_driver_livox/`（纯 CMake，不依赖 ROS）
  - 公共工厂 `driver::livox::make_lidar(const hal::LidarConfig&)`，公共头无 SDK 类型；
  - `livox_api.hpp` 函数表抽象 + stub/真实实现，`FORAY_BUILD_DRIVER_LIVOX` 开关；
  - `LivoxRuntime`：进程级 SDK 引用计数、handle 路由、sink 墓地；
  - 原始包环（`SpscQueue<PacketSlot>`）+ 解析线程 + `FramePool` 组帧；
  - 包解析：0x01 高精度 / 0x02 低精度 / 0x03 球坐标 / 0x00 IMU；
  - 时间映射：`time_type` 0/1/2/3/4/未知 → `TimestampSync`；
  - Mid-360s（dev_type 35）：JSON 同时生成 `MID360` + `Mid360s`；
    SDK handle（`s_addr`）与按 IP 预绑定不一致时按 serial/IP 回退；
  - 非重复扫描时间切帧：`frame_duration_ms`（默认 100 ms）；
  - IMU 加速度 g → m/s²（`9.80665`）；
  - JSON 生成（0600）、本机 IP 校验、默认单播、多雷达、退避恢复；
  - SDK 依赖：`FindLivoxSDK2.cmake` + `LivoxSDK2::shared`；
    `FORAY_LIVOX_SDK_PROVIDER=SYSTEM|PREBUILT`；
  - core/backend 目标拆分：mock 测试只链 core，任何开关下不依赖 SDK；
  - `FORAY_DRIVER_LIVOX_HAS_SDK=0/1` 作为已安装产物能力导出；
  - `foray_livox_dump`：PCD + 俯视 PPM 导出、`--self-test` 自检；
    `--stream` 逐点流式写 stdout（每帧 flush、统计走 stderr、
    `--frames 0` 持续输出、EPIPE 视为正常结束）；
- 文档：[docs/driver_livox.md](docs/driver_livox.md)、
  [docs/driver_deployment.md](docs/driver_deployment.md)；
- 单测：30 个用例（29 core + 1 backend），mock 覆盖生命周期、路由、恢复、解析、组帧；
  常规 / ASan+UBSan / TSan 全绿；安装后下游 `find_package` 通过；
- 跨仓：`foray_ws/.repos` pin Livox-SDK2 v1.3.1，新增依赖/构建/环境脚本；
- **已完成**：真实 SDK2 构建、ON 编译/链接、下游消费；实机 HIL：
  Mid-360s `192.168.1.195`，2–3 帧 × 20064 点、IMU、PCD/PPM；
- **未完成**：双回波 `0x11`。

### M11 交付物（雷达网络自举与参数注入）

- `foray_platform/include/foray/platform/net/`
  - `net_types.hpp`：`Ipv4` / `InterfaceInfo` / `Topology` / 只读 `probe_topology()`；
  - `process.hpp`：`ProcessRunner` 抽象（不经过 shell）；
  - `backend.hpp` + `nm_backend.hpp` + `netlink_backend.hpp`：NM 优先、netlink 回退；
  - `setup.hpp`：`plan_link()` 纯规划、`ensure_link()` 执行 + 逆序回滚、`select_backend()`；
- `foray_platform/tools/foray_net_helper.cpp`：libexec helper，参数白名单，
  polkit 动作 `org.foray.netconfig.bootstrap` 只放行该程序；
- 驱动控制面：`LivoxApi` 增 set_lidar_ip / set_install_attitude / set_pps_sync_mode /
  set_host_port_cfg / set_rmc_sync_time / query_internal_info；`LivoxControl`
  统一重试与写后读回；`foray_livox_config` 工具；
- `foray_lidar_up`：链路就绪检测 + pkexec 调 helper + 驱动启动 + 读帧健康确认
  （退出码 2/3/4/5/6 区分驱动、网络与无帧）；
- 文档：[docs/net_bootstrap.md](docs/net_bootstrap.md)，
  `docs/driver_deployment.md` §2.2 改为 helper 自举；
- 单测：平台 150 个用例（net 新增 32 个）、驱动 36 个（控制面 6 个）全绿；
- **待 HIL**：实机雷达当前断线（`enp0s31f6` 无载波），helper 已能经 NM
  创建 `foray-enp0s31f6-slave` 并给出"没有载波"警告。

## 当前 Agent State

M11_NET_BOOTSTRAP_DONE（实机 HIL 待雷达恢复连接）

## Before Snapshot

commit hash:   51d451b
branch:        feat/l1-foundation
modified files: 新增 FindLivoxSDK2、core/backend 拆分、HAS_SDK、SDK ON CI；
  跨仓 foray_ws：`.repos` pin、scripts、文档
risk level:    L1

## 模糊点与待确认项

- 本仓接口尚未冻结，任务清单为**方向性**的，落地顺序以 `foray_docs/algorithm_structure.md` §9 演进阶段为准
- 依赖的自研仓尚未建齐，跨仓任务需等对方 `README.md` 明确接口后再启动
- **ROS 适配层尚未开始**：`use_sim_time` 的映射已在文档中冻结，但 `/clock` 订阅与 `rclcpp` 适配留到后续单元
- **`package.xml` 的 `license` 字段为 `TODO`**，需组织确认许可证后填写
- 参数系统暂不支持序列参数与变更回调；配置栈的数组/动态列表支持按需扩展
- 配置解析栈按计划在 core 内扩展（`config/`）；机器人 YAML 归 `foray_robots`
- HAL 目标的驱动与 ROS 适配尚未开始，按 M2–M12 里程碑推进
- Livox SDK2 开发机默认装到 `~/.cache/foray-platform-prefix/livox-sdk2`；
  实机镜像固定 `/opt/foray/livox-sdk2`；版本由 `foray_ws/.repos` pin
- 实机雷达为 **Livox Mid-360s**（dev_type 35，SN `ARMCP7D0030195`），
  IP `192.168.1.195`，主机 `br-lidar` `192.168.1.51`；HIL 已通过

## Vector Backend Status

Backend: Markdown
Status:  ready
Environment: 仓库内 Markdown 文档（无外部向量后端）
Index: 本仓 `README.md` / `tree.md` / `decision.md` / `docs/**`
Initialization: 2026-02-19
Commit: （首次提交）

## Acceptance Criteria

- [x] 每个新增模块带独立可执行测试（平台 151 个 + Livox 驱动 36 个用例）
- [x] 核心库可脱离 ROS 独立构建、测试、安装，并可被下游 `find_package` 消费
- [x] Livox 驱动在无 SDK 环境用 stub/mock 构建并测试全绿
- [x] Livox 真实 SDK2 构建、ON 编译/链接与下游消费
- [x] Livox 实机读取：Mid-360s 点云 + IMU + PCD/PPM
- [x] 网络自举：probe/plan/rollback/nmcli 命令行断言覆盖，`foray-net-helper`
      经 pkexec 实机生效（NM profile `foray-enp0s31f6-slave` 持久）
- [x] 常规 / ASan+UBSan / TSan 三套构建全绿（平台 151 + 驱动 36）
- [ ] 雷达网络自举 + 流式输出的实机 HIL（雷达恢复连接后跑 `foray_lidar_up`）
- [ ] 本仓能独立 `colcon build`（需在 ROS 2 环境/CI 验证）
- [ ] CI 绿灯
