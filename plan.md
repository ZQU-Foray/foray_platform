# 开发计划

## 任务列表

- [x] 定义时间与时钟抽象（含 `use_sim_time` 语义）
- [x] 搭参数系统骨架（schema + 校验 + 热加载）
- [x] 定义 HAL 抽象接口（传感器 / 执行器 / 总线 / 设备生命周期）
- [x] 建立 USB CDC 设备打开/关闭的最小构建与无硬件测试骨架
- [x] 验证 USB CDC 二进制字节收发与 CRC-32/ISO-HDLC 的离线测试向量
- [x] 候选 RoboLink 零槽保活帧离线组帧与独立黄金字节测试（不代表电控端已支持）
- [x] 候选 RoboLink 字节流解析器：半帧、连帧、垃圾前缀、坏头及 CRC 失败恢复的离线测试
- [x] 候选 RoboLink 零槽/单槽/多槽帧与八类消息载荷的编解码/校验，完成 USB 字节通道离线串接
- [x] 按已合入 `foray_interfaces` 的 RoboLink v0.2 候选稿更新云台角 rad 与姿态反馈的离线测试向量
- [x] 离线实现主机握手、心跳、状态回读、500 ms 超时及设备重开状态机
- [ ] 验证发送调度的频率/延迟，并与 MCU 固件进行只读/禁能联调
- [ ] 完成 v0.2 与电控固件逐字节对账及失联动作验收；在此之前不发送控制指令到硬件
- [ ] 与自瞄组补齐云台系目标到世界系目标角的转换和图像—IMU 时间对齐，再开放真实云台控制
- [ ] USB CDC 设备参数配置与错误恢复策略细化（波特率/超时/重试）
- [ ] 接一个真实驱动跑通「一个传感器 → 一个话题」
  - Livox 驱动代码与 mock 测试已完成；真实 SDK 构建 + HIL 已通过
  - Hikvision 相机驱动代码、81 个 mock 单测与 MV-CS004-10GC 实机取流已通过
- [ ] 实现急停硬路径（独立实时线程 + 超时定时器）

### 任务 1 交付物

- `src/foray_platform_core/include/foray/platform/time/`
  - `Duration` / `Timestamp` 强类型；
  - `Clock` 接口 + `SystemClock` / `SteadyClock` / `ManualClock`；
  - 三个时钟域与 `use_sim_time` 就绪/跳变语义；
- 设计说明：[docs/time_and_clock.md](docs/time_and_clock.md)；
- 单测：`src/foray_platform_core/test/time/clock_test.cpp`。

### 任务 2 交付物

- `src/foray_platform_core/include/foray/platform/param/`
  - 类型化 schema、声明校验、范围/枚举校验；
  - 不可变快照 + 原子替换的存储；
  - 多来源分层（对应 `shared.yaml` + `<robot>.yaml`）；
  - YAML 文件来源（严格未知键、按 schema 解析类型）；
  - 初始加载 / 热加载门面，整批原子生效；
- 设计说明：[docs/parameter_system.md](docs/parameter_system.md)；
- 单测：`src/foray_platform_core/test/param/`。

### M1 交付物（HAL 前置）

- 包合并：`foray_platform_core/` → `foray_platform/`；
  `foray_platform_core` 与后续 `foray_platform_hal` 为同包两个 target；
- `src/foray_platform_core/include/foray/platform/result/`
  - `ErrorCode` / `Error` / `Result<T>` / `Result<void>`；
  - 驱动原始错误码保留在 `Error::vendor_code`；
- 单测：`src/foray_platform_core/test/result/result_test.cpp`；
- 决策记录：`decision.md` 的包合并、配置栈归属、通用错误类型条目。

### M2 交付物

- `src/foray_platform_core/include/foray/platform/container/spsc_queue.hpp`
  - 有界、无锁、单生产者/单消费者；
  - 容量向上取 2 的幂；满时 `try_push` 返回 false 并累计 `dropped()`；
  - `T` 要求可默认构造、移动构造/赋值不抛异常；
- 单测：`src/foray_platform_core/test/container/spsc_queue_test.cpp`；
- TSan 并发用例通过。

### M3 交付物

- `src/foray_platform_core/include/foray/platform/config/`
  - `ConfigDocument` / `ConfigLayer`：分层与可选层；
  - `ConfigResolver`：YAML 加载、顶层 `include`、`${ENV}` 替换、
    `FORAY_<PARAM>` 覆盖、来源追踪；
  - `ConfigSnapshot`：已校验值 + provenance；
  - `ConfigExporter`：dump / Markdown schema / JSON Schema；
  - `ConfigTool`：validate/dump/schema 门面（可执行文件待 HAL schema 落地）；
- 复用 `param` 的 `ParamSchema` / `ParamValue` / `ParamStore` 做校验与宽化；
- 设计说明：[docs/config_stack.md](docs/config_stack.md)；
- 单测：`src/foray_platform_core/test/config/`；ASan/UBSan 通过。

### M4/M5 交付物

- `src/foray_platform_core/include/foray/platform/hal/`
  - `Device` / `Camera` / `Lidar` 生命周期与读取接口；
  - `ImageFrame` / `PointCloudFrame` / `ImuSample` / `TimestampSync`；
  - `CameraConfig` / `LidarConfig` / `DeviceSelector` / `TriggerConfig`；
  - `FrameLease`（move-only）+ `FramePool`（最新帧邮箱、固定槽位、无锁读写）；
- 同包新增 `foray_platform_hal` interface target，导出 `foray_platform::hal`；
- 单测：类型/默认值、发布/获取、替换丢帧、槽位耗尽、租约晚于池销毁、
  单写单读并发；ASan/UBSan/TSan 通过；
- 设计说明：[docs/hal_architecture.md](docs/hal_architecture.md)。

### M6 交付物

- `src/foray_platform_core/include/foray/platform/hal/config_binder.hpp`
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

### M7/M8 交付物（Hikvision）（2026-10-24，已完成）

设计与取舍见 [docs/driver_hikvision.md](docs/driver_hikvision.md)。
- 隔离方式沿用 Livox：`detail::MvsApi` 虚接口，`src/hikvision/mvs_api_real.cpp`
  是唯一 include `MvCameraControl.h` 的翻译单元；公共头只有 HAL 类型；
  `FORAY_BUILD_DRIVER_HIKVISION=ON/OFF` + `FORAY_MVS_ROOT`，ON 找不到 SDK 直接
  `FATAL_ERROR`；core/backend 两个 OBJECT 目标，mock 测试不链 MVS。
- 取流模型不同于 Livox：`MV_CC_GetImageBuffer` 是阻塞调用，取流线程即生产者，
  链路为 `GetImageBuffer → memcpy 到 FramePool → FreeImageBuffer`（同线程），
  不需要原始包环与解析线程；超时 100 ms 走正常控制流，不 `sleep` 轮询。
- 设备选择严格按 HAL 约定（serial > user_defined_name > model+index > ip）：
  空 selector 返回 `kInvalidArgument`，多候选返回 `kDeviceAmbiguous`，不默认取第 0 个；
  枚举结果按 serial 去重（主机双 IP 会让同一相机出现两次）。
- 时间戳：只用 `nDevTimeStampHigh/Low` + 标定偏移和/或主机接收时刻；
  `nHostTimeStamp` 单位未定义，不使用。
- 已完成：
  - HAL 参数缺口 1/2/3 已补齐（`device_to_host_offset_ns` /
    `exposure_to_host_latency_ns`、`trigger.debounce_us` 进 schema 与 binder）；
  - `src/foray_platform_driver_hikvision/`：公共工厂、生命周期与取流、节点映射、
    时间换算、错误映射、stub 与真实后端（`mvs_api_real.cpp`）；
  - 81 个单测（生命周期 / 配置映射顺序 / 时间三路径 / 超时与掉线 / 帧池计数 /
    后端能力），`FORAY_BUILD_DRIVER_HIKVISION=OFF` 与 `=ON` 两种配置全绿；
  - `foray_hikvision_dump`：配置 + 取 N 帧 + 帧率/丢帧统计 + 软触发循环；
  - `foray_hikvision_dump --view`（2026-10-25）：构建期找到 SDL2 时开窗口
    实时预览，保持原始宽高比、首帧到达后才建窗，ESC/q/关窗即停流；
    合成棋盘帧截屏采样验证无偏色且 4:3 不拉伸；
  - 实机 HIL（MV-CS004-10GC）：720×540 @ 118–127 Hz，200 帧 0 丢包，
    ROI 640×480、曝光/增益下发、软触发均验证通过。
- 仍未做：拔插恢复的实机验证（`recover()` 退避逻辑有单测覆盖）、30 分钟长跑；
  部署步骤补进 `docs/driver_deployment.md` §4 也待办。

### M9/M10 交付物（Livox）

- `src/foray_platform_driver_livox/`（纯 CMake，不依赖 ROS）
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

- `src/foray_platform_core/include/foray/platform/net/`
  - `net_types.hpp`：`Ipv4` / `InterfaceInfo` / `Topology` / 只读 `probe_topology()`；
  - `process.hpp`：`ProcessRunner` 抽象（不经过 shell）；
  - `backend.hpp` + `nm_backend.hpp` + `netlink_backend.hpp`：NM 优先、netlink 回退；
  - `setup.hpp`：`plan_link()` 纯规划、`ensure_link()` 执行 + 逆序回滚、`select_backend()`；
- `src/foray_platform_core/apps/foray_net_helper.cpp`：libexec helper，参数白名单，
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

### M12 交付物（USB CDC 与候选 RoboLink）

- `foray_platform/include/foray/platform/link/`（命名空间 `foray::platform::link`）
  - `usb_cdc_port.hpp`：设备路径与 fd 生命周期、原始字节读写；
  - `robolink_crc.hpp`：CRC-32/ISO-HDLC；
  - `robolink_frame.hpp`：零槽保活组帧；`robolink_frame_parser.hpp`：字节流切帧与重同步；
  - `robolink_codec.hpp`：零槽/单槽/多槽帧 + 八类载荷的显式小端编解码；
  - `robolink_byte_channel.hpp`：设备↔帧层桥接（单帧待发、部分写入）；
  - `robolink_host_link.hpp`：主机握手/心跳/回读/超时/重开状态机；
- 同包新增 `foray_platform_link` 库目标（EXPORT_NAME `link`），并入
  `foray_platform_tests`；
- 单测：`foray_platform/test/link/` 21 个用例（设备生命周期、CRC 向量、组帧黄金字节、
  半帧/连帧/坏头恢复、载荷往返、字节通道、主机链路）；
- **本地无硬件验证**：`cmake -S foray_platform -B build/platform -DCMAKE_BUILD_TYPE=Debug`
  + `cmake --build` + `ctest` → 192/192 通过（含 link 21 个）；
- 文档：[docs/robolink_protocol_review.md](docs/robolink_protocol_review.md)；
- **未完成**：与 MCU 固件逐字节对账、发送频率/延迟验收、实机联调、失联动作验收。

### M13 交付物（设备网络自举：Hikvision GigE + MTU + 跨桥保护）

- `net` 层：`InterfaceInfo::mtu`（sysfs 读，0=未知）、`NetBackend::set_mtu`
  （NM 走 `802-3-ethernet.mtu`，netlink 走 `IFLA_MTU`）；
- `plan_link`：动作序加 `set_mtu(物理口) → set_mtu(网桥)`，MTU 失败只 warning；
  物理口已属另一桥时默认 `kInvalidArgument`，`allow_reassign_master` 显式放行；
- `link_ready`：`(topology, iface, bridge, host_ip=nullopt, min_prefix=16,
  min_mtu=nullopt)`，空地址只校验挂桥/载波，MTU 0 视为未知；
- `net/bootstrap.hpp`：`bootstrap_link()` 编排（探拓扑 → 判就绪 → pkexec →
  helper），雷达与相机共用；`foray_lidar_up` 删除本地拼装，改为调用它；
- `foray-net-helper`：新增 `--mtu 576-65535` 与 `--allow-reassign-master`；
- Hikvision：`camera_bootstrap`（LLA/同网段地址规划）、
  `foray_hikvision_up`（挂 `br-camera` → 地址 → MTU → 枚举 → 取 3 帧），
  USB3 只做可见性检测 + TODO；
- 文档：[docs/net_bootstrap.md](docs/net_bootstrap.md) 去雷达限定，
  `docs/driver_deployment.md` §2 加桥隔离/MTU、§4 加一键自举与 USB3 TODO；
- 单测：平台 178 个用例（net MTU / 跨桥 / bootstrap），Hikvision 88 个
  （新增地址规划 5 个），Livox 36 个回归全绿；
- **待 HIL**：相机口 `enp0s13f0u1` 的实机自举与取帧（需现场授权改网络）。

### M14 交付物（目录布局与包命名规范化）

- 仓根 = colcon 空间根：三个包迁入 `src/`，仓根只保留文档五件套、`docs/`、`.github/`
  与构建产物目录（`build/`、`install/`、`log/`，不入库）；
- 核心包更名 `foray_platform_core`（`package.xml` 名 / CMake `project()` / 包配置导出
  文件名），`find_package(foray_platform_core)` 供两个驱动与后续 ROS 适配层消费；
  **导出命名空间仍为 `foray_platform::core|hal|net`**，下游链接不变；
- 包内骨架固化：C++ 可执行小工具进 `apps/`（core `foray_net_helper`；hikvision
  `camera_dump`/`camera_up`/`viewer`；livox `livox_dump`/`livox_config`/`lidar_up`），
  polkit 动作与免密规则进 `data/polkit/`，包内不再有 `tools/`；
- `README.md`、`tree.md`、`docs/**` 的路径与包名同步；
- CI 新增 `dir-layout`：包不得平铺仓根、`src/` 下包名须与目录一致、工程包须有
  `include/` + `test/`、包内禁用 `tools/`（脚本类包豁免）；
- 决策留痕：[decision.md](decision.md)「2026-10-25 目录布局与包命名规范化」。

## 当前 Agent State

M14_LAYOUT_REFACTOR_DONE + HOST_LINK_V02_OFFLINE_TESTED + LOCAL_AUDIT_REMEDIATION_DONE（目录规范化完成；本地审计 28 条整改全部落地，全量 406 个用例 0 失败；M13 设备网络自举的实机 HIL 仍待现场授权；RoboLink 仅离线验收，未接真机）

## 本地审计整改（2026-11-01，28 条）

依据 `audit/tasks/01-foray_platform.md` 与 `audit/hal-localization-navigation-audit.md`，在
`fix/hal-audit-remediation` 分支上一条一提交完成 28 条整改（27 笔提交），全量回归
`colcon test-result --all` = 406 tests / 0 errors / 0 failures（整改前基线 341，未下降）。

| 优先级 | 条目 | 一句话结果 |
|---|---|---|
| P0 | H8 | 帧池句柄加互斥快照，`read()` 不再与 `close()` 竞争同一 `shared_ptr`（TSan 改前实报、改后零报告） |
| P0 | H9 | `frame_id` 在 `open()` 期冻结，运行期改写改为 `kNotSupported` |
| P1 | H12 | PTP / 标定偏移 / 主机接收三条时间路径分家，无同步时置零并标 `kNoSync` |
| P1 | H1 | HELLO 序号每次自增，ACK 按 4 个在途序号的窗口匹配 |
| P1 | H17 | Mid-360 不再用 `frame_cnt` 切帧，只按时间窗；`frame_cnt` 仅作一致性计数 |
| P1 | H18 | 帧基时时间加单调水线，乱序包与整段回退分开计数 |
| P1 | H19 | 帧时长的点偏移改为整包范围校验，越界整包拒收（不改宽 32 位字段） |
| P1 | H10 | 异常回调注册失败不再吞掉，句柄销毁先于桥表摘除 |
| P1 | H11 | 自动曝光/增益模式下不下发手动值，`apply_tuning` 明确拒绝 |
| P2 | H2 | CRC 失败只丢一字节重扫，不再按坏帧头吃掉后随真帧 |
| P2 | H4 | 外部进程 stdout/stderr 用 `poll` 并发读，消除互等死锁 |
| P2 | H5 | 控制快照改为邮箱补发，通道忙不再静默丢弃 |
| P2 | H13 | 区分"没数据"与"链路失败"，自由运行沉默超 2 s 判故障 |
| P2 | H14 | 帧几何与池不一致时整帧拒绝，不再截断后仍报新尺寸 |
| P2 | H15 | `recover()` 不再清零 `sequence_` 与会话统计 |
| P2 | H20 | SDK 双回调生产者串行化，SPSC 队列不再被两线程同时写 |
| P2 | H22 | `recover()` / `start()` 的等待移出控制锁，卡住的调用可被打断 |
| P2 | H23 | 用 `udp_cnt` 连续性把网络丢包变成可观测指标（`packets_lost`） |
| P2 | H24 | 回调侧选择条件字符串改为加锁值拷贝 |
| P2 | H25 | 内部信息 TLV 解析加边界，游标不再越过响应缓冲 |
| P3 | H3 | HELLO 的 `heartbeat_period_ms = 0` 不再被悄悄改成 50 |
| P3 | H6 | 打开串口后 `tcflush`，输入输出队列里的陈旧字节不再被当帧 |
| P3 | H7 | 自动摘桥旧地址前先写 warning 留痕 |
| P3 | H21 | Livox 包头偏移表补全 `rsvd` / `crc32`，`time_interval` 语义改正 |
| P3 | H26 | 设备自报 length 写成契约；ROI/binning/decimation 允许显式归零；删死语句；PTP 定期重探测 |

只能实机验证、本次**未验证**的三项：H23 真实丢包率、H12 PTP 同步精度、H17 真实帧率。
详见 [decision.md](decision.md)「2026-11-01 本地审计整改的取舍」。

## Before Snapshot

commit hash:   af2dd95（origin/dev，合并上游后）
branch:        feat/usb-cdc
modified files: 新增 `foray/platform/link/` 头文件、`src/link/` 源码、`test/link/` 测试；
  CMakeLists 新增 link 库目标与 install；README/tree/plan/decision 增补 link 章节
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
- [x] 本仓能独立 `colcon build`（在 `foray_sim:humble-fortress` 容器里 `colcon build --base-paths src` 通过，Jazzy/CI 待验）
- [x] link 模块 21 个用例并入平台测试目标（本地 Debug 构建 192/192 通过）
- [ ] RoboLink v0.2 与 MCU 固件逐字节对账 + 实机联调（对账前不向硬件发控制指令）
- [x] 本地审计 28 条整改落地，全量 406 个用例 0 失败，一条一提交（决策留痕见
      [decision.md](decision.md)「2026-11-01 本地审计整改的取舍」）
- [ ] CI 绿灯
