# 关键工程决策日志

> 记录 fork patch、换版本、**破坏性接口变更**、架构取舍。
> 格式见 `foray_docs/repository_structure.md` §7.2。

---

## Decision

Date: 2026-02-19

Context: 本仓从 0 初始化。需要确定它在整体架构中的位置与依赖边界。

Decision: 本仓定位为 **L1** 层，owner 角色为「平台」，
依赖层级 X2。

Reason:
- 切分判据 C1–C6 见 `foray_docs/repository_structure.md` §1
- 粒度结论：**一个仓 = 一个团队角色 + 一个变更率档位**
- 零业务语义，被所有层依赖，**最稳定**

Alternatives:
- 并入相邻仓（减少仓库数量，但违反团队边界或变更率差异）

Rejected:
- 按 ROS 2 package 粒度切仓 —— 会导致版本矩阵地狱
- 按分层机械切仓 —— 会切断团队边界（Conway）

---

## Decision

Date: 2026-10-18

Context: L1 必须能脱离 ROS 单测，且本机开发环境（Arch）没有 ROS 2 Humble。
需要确定 L1 代码与 ROS 的耦合边界。

Decision: 新增 `foray_platform_core`，采用**纯 CMake + C++17**，
不链接 `rclcpp`/`ament_cmake`；`package.xml` 声明 `build_type=cmake`，
使 colcon 与普通 `cmake` 都能构建。ROS 适配层后续单独成包。

Reason:
- L1 的验收要求包含「模块独立可测」；核心库不引入 ROS 才能在任意环境跑单测；
- 符合 README 纪律「不直接到处使用 ROS 2」与依赖倒置；
- `yaml-cpp` 与 GTest 均为 rosdep 已收录的键（`yaml-cpp` / `gtest`），
  ROS 2 工作空间与纯 CMake 环境都能安装。

Alternatives:
- 核心库使用 `ament_cmake`：`colcon test` 集成更顺，但本地无 ROS 时无法构建；
- 单个 `ament_cmake` 包同时容纳核心与 ROS 适配：简单，但会把 ROS 依赖
  传染给所有下游。

Rejected:
- 在核心公共头文件中暴露 `rclcpp` 类型；
- 为本地开发另建一套非 CMake 构建脚本（双重构建源）。

---

## Decision

Date: 2026-10-18

Context: `use_sim_time=true` 时 `/clock` 可能停流或跳变，安全相关超时不能
假设时间单调；急停硬路径要求不依赖中间件。

Decision: 时间抽象划分三个时钟域：`kSystemWall` / `kSteady` / `kSimulated`。
- `use_sim_time=false`：节点时间用 `SystemClock`，本地间隔用 `SteadyClock`；
- `use_sim_time=true`：节点时间用由 `/clock` 驱动的 `ManualClock`；
- **急停硬路径与本地看门狗一律使用 `SteadyClock`**；
- 仿真时钟在首帧 `/clock` 之前 `is_ready()==false`，`now()` 为零，
  调用方不得把零时刻用于超时判定；
- 时钟域在启动期选定，运行期不切换。

Reason:
- 仿真器暂停时仿真时间停止，基于它的急停定时器永远不会触发；
- NTP 校时会让墙钟回退，不能用于超时；
- 显式域标签（文档约定，类型上不携带）避免零开销抽象被破坏。

Alternatives:
- 只暴露一个 `now()`：接口简单，但无法表达就绪、单调与域约束；
- 在 `Timestamp` 内携带域标签：运行时开销与 ABI 复杂度上升。

Rejected:
- 用系统墙钟实现急停看门狗；
- 在仿真时间停流时仍按「最后已知时间」推进超时。

---

## Decision

Date: 2026-10-18

Context: 参数系统需要类型化校验与热加载，同时不能让运行期读取阻塞实时线程。

Decision: 采用「**不可变快照 + 原子替换**」：
- `ParamSchema` 声明类型、默认值、边界、枚举与 `hot_reloadable`；
- `ParamStore` 写入串行化，读取通过 `shared_ptr<const ParamSnapshot>` 无锁完成；
- 一次 `apply()` 要么整批生效，要么完全回滚；
- YAML 来源严格校验未知键，类型按 schema 解析；只允许 `int -> double` 宽化；
- `hot_reloadable=false` 的参数只能在启动期初始加载时写入；
- 多来源按顺序覆盖，对应 `shared.yaml` + `<robot>.yaml` 两级配置。

Reason:
- 实时线程只读快照，不持有写锁，也不会看到半新半旧的值；
- 严格未知键能把配置拼写错误在启动期暴露，而不是静默使用默认值；
- `int -> double` 宽化符合 YAML 使用习惯，反向窄化会静默截断。

Alternatives:
- 互斥锁保护一张可变映射：实现简单，但每次读取都加锁；
- 直接使用 ROS 2 参数服务：核心库将依赖 ROS，且类型校验能力有限。

Rejected:
- 忽略 YAML 中未声明的键；
- `double -> int` 隐式截断；
- 在实时线程中执行参数写入或 YAML 解析。

---

## Decision

Date: 2026-10-18

Context: 本单元只交付 L1 抽象，尚未引入 ROS。需要明确 `use_sim_time`
的落地位置，避免后续实现时语义漂移。

Decision: `use_sim_time` 的读取与 `/clock` 订阅放到后续的 ROS 适配包；
适配层构造并注入 `Clock&`，业务模块不直接持有 `rclcpp::Clock`。
本单元在 `docs/time_and_clock.md` 中冻结映射与就绪语义。

Reason:
- 保持核心库零 ROS 依赖；
- 注入 `Clock&` 使模块可在单测中使用 `ManualClock` 复现时序；
- 适配层可独立替换（ROS 2 / 回放 / 仿真器直连）。

Alternatives:
- 在核心库中预留 `rclcpp` 前向声明：仍会污染公共头文件；
- 各节点自行读取 `use_sim_time`：语义会再次分散。

Rejected:
- 在核心库中包含 ROS 头文件。

---

## Decision

Date: 2026-10-18

Context: 原计划把 L1 core 与 L2 HAL 拆成两个包（`foray_platform_core`
与 `foray_platform_hal`）。两者同仓、同 owner、同变更率，额外包会带来
重复的 package.xml、CMake 导出与 CI 条目。

Decision: 合并为一个包 **`foray_platform`**，包内保留两个 CMake target：
`foray_platform_core`（L1）与 `foray_platform_hal`（L2）；
别名仍为 `foray_platform::core` / `foray_platform::hal`。

Reason:
- 符合「一个仓 = 一个团队角色 + 一个变更率档位」；
- 减少重复的构建/导出/CI 配置；
- L1/L2 边界由 target 与 `include/foray/platform/{core 模块,hal}` 路径保留。

Alternatives:
- 保持两个独立包：边界更硬，但当前收益不足以抵消重复配置；
- 合并为单库：更简单，但会让只依赖 L1 的下游被迫链接 HAL。

Rejected:
- 继续按 ROS package 粒度细分。

---

## Decision

Date: 2026-10-18

Context: 整个软件系统需要统一的配置文件解析栈。需要确认归属，避免各仓重复实现。

Decision: **不新建仓**。解析机制归 `foray_platform` 的 core target
（在现有 `param` 上扩展 `config`：分层、include、env 替换、来源追踪、CLI）；
机器人相关配置值与装配归 `foray_robots`。

Reason:
- `foray_docs/repository_structure.md` §2 已把「参数系统」划给 `foray_platform`；
- 同文档 §1 明确把「把 config 拆出独立仓」列为反模式，config 属于装配根；
- `foray_robots` 已有远端仓，但目前只有骨架，尚无代码/配置；
- 平台组作为 owner，配置栈被所有仓依赖、变更率低，符合 C1/C2/C3。

Alternatives:
- 新建 `foray_config`：仅在未来出现非 C++ 消费者、独立发布节奏或仓外工具链时再评估；
- 放到 `foray_interfaces`：该仓是通信契约仓，不适合运行时配置解析。

Rejected:
- 各仓自行解析 YAML；
- 在装配根里实现通用解析库。

---

## Decision

Date: 2026-10-18

Context: HAL 与驱动需要统一的错误语义，现有 `ParamResult` 只服务参数系统。

Decision: 在 core 增加通用 `ErrorCode` / `Error` / `Result<T>` / `Result<void>`，
公共 API 不抛异常；驱动把 MVS/Livox 原始返回码放入 `Error::vendor_code`。
`ParamResult` 暂不迁移，后续再统一。

Reason:
- HAL/驱动/未来算法层都需要同一套错误语义；
- 保持现有参数 API 兼容，避免一次性大改。

Rejected:
- 每个模块自定义错误类型；
- 在公共 API 中使用异常。

---

## Decision

Date: 2026-10-18

Context: HAL 需要在不依赖 ROS/OpenCV 的前提下，为图像/点云提供稳定的读取接口，
并明确零拷贝边界与并发模型。

Decision:
- 采用 `FramePool<T>` + `FrameLease<T>`：
  - 固定槽位、预分配载荷、单写单读；
  - 最新帧邮箱，消费者慢时丢旧帧并计数；
  - `FrameLease` move-only；deleter 持有帧池 `shared_ptr`，租约可晚于设备销毁；
- **HAL 边界零拷贝**：SDK 取流/回调处允许一次入队拷贝，交给消费者后不再拷贝；
- 海康 `Get/Free` 固定在取流线程，规避官方未承诺的跨线程释放；
- 严格 SDK 缓冲直通模式作为后续可选项，不作为首期默认。

Reason:
- 一次 memcpy 相对于链路带宽与算法耗时占比很小；
- 固定槽位 + 无锁读写满足进程内软实时要求；
- 租约生命周期清晰，避免裸指针被下一帧覆盖。

Alternatives:
- 返回裸指针：需要调用方自己管理生命周期，易悬空；
- 每帧分配/返回 vector：热路径分配，不可接受；
- 严格零拷贝直通：需要跨线程 Free 或自建 UDP 收包，首期收益不足。

Rejected:
- 在 HAL 公共头中暴露 SDK 缓冲类型；
- 消费者直接调用 `MV_CC_FreeImageBuffer`。

---

## Decision

Date: 2026-10-18

Context: Livox SDK2 的 `LivoxLidarSdkInit/Uninit` 是进程级单例，
多雷达共享全局回调；回调线程不允许加锁/分配，且关闭一个雷达时可能有回调在飞行。

Decision:
- 增加 `LivoxRuntime`：引用计数管理 Init/Uninit，按 handle 路由全局回调；
- 每个雷达一个 `CallbackSink` + `SpscQueue<PacketSlot>`；
- `close()` 只把 sink 从路由表移除并移入墓地，等 `LivoxLidarSdkUninit()`
  返回后统一销毁；
- 回调只做长度校验、接收时间戳与一次 memcpy，解析/组帧在独立线程。

Reason:
- 避免在 SDK 回调中加锁或分配；
- 墓地机制保证回调线程不会访问已释放内存；
- 多雷达共享一份 SDK 初始化，符合 SDK 的进程级约束。

Alternatives:
- 每个雷达独立初始化 SDK：SDK 不支持；
- 回调中直接解析：会增加回调延迟并可能阻塞收包。

Rejected:
- 在回调中调用控制接口或重连；
- `close()` 后立即销毁 sink。

---

## Decision

Date: 2026-10-18

Context: Livox 包不携带 line 字段，`time_type` 语义与绝对时间可用性不一致，
双回波 0x11 在 SDK v1.3.1 公共头中无结构定义。

Decision:
- `line = i % line_num`，按包内点序号计算，与 `livox_ros_driver2` 一致；
- `time_type` 0 → NoSync、1 → PTP、2 → GPS PPS、3/4 → PPS、其他 → Unknown；
- 只有 PTP / GPS PPS 才把设备时间写入 `base_time` / `stamp`；
  NoSync / PPS / Unknown 一律置零；
- 双回波 0x11 返回 `kUnsupportedFormat`，不猜测布局。

Reason:
- 不伪造绝对时间，避免与其它传感器静默混用时间域；
- 与社区驱动保持字段语义一致，便于 point_lio/FAST-LIO 兼容；
- 未定义结构不能靠猜测解析。

Alternatives:
- PPS 时用主机时间补齐：会引入未标定的偏差；
- 自行猜测双回波布局：跨固件版本风险高。

Rejected:
- 在 HAL 内做跨传感器时间对齐；
- NoSync 时用 `SystemClock` 替换设备时间。

---

## Decision

Date: 2026-10-18

Context: 本机开发/CI 环境不一定安装 Livox SDK2，驱动仍需要可构建、可测试。

Decision:
- `FORAY_BUILD_DRIVER_LIVOX` 默认 OFF；
- OFF 时构建 stub，所有控制接口返回 `kSdkUnavailable`，下游仍可链接；
- ON 时用 `LIVOX_SDK2_ROOT`/系统路径 `find_path` + `find_library`，
  找不到直接 `FATAL_ERROR`，不静默降级；
- 单元测试通过注入 `LivoxApi` mock 运行，完全不依赖 SDK/硬件。

Reason:
- 保持 CI 与无 SDK 开发环境绿灯；
- SDK 缺失在配置期显式失败，避免运行时才暴露；
- mock 覆盖协议解析、组帧、路由与恢复逻辑。

Rejected:
- 把 SDK 二进制提交到仓库；
- OFF 时仍假装初始化成功。

---

## Decision

Date: 2026-10-19

Context: 需要明确 Livox SDK2 的组织内依赖方式，并解决两个构建问题：
ON 时 mock 测试被迫链接真实 SDK；静态驱动库把构建机 SDK 绝对路径导出给下游。

Decision:
- **不 fork、不使用 submodule、不使用 FetchContent**；Livox-SDK2 v1.3.1
  以 commit `f5d9375f84efe2b15bc0a052d3e18482ed13adf4` 在
  `foray_ws/.repos` 中 pin 到 `third_party/livox_sdk2`；
- 开发机默认安装到 `~/.cache/foray-platform-prefix/livox-sdk2`；
  实机镜像固定 `/opt/foray/livox-sdk2`；
- CMake 新增 `FindLivoxSDK2.cmake`，只暴露 `LivoxSDK2::shared`；
  `FORAY_LIVOX_SDK_PROVIDER=SYSTEM|PREBUILT`，PREBUILT 要求 SDK 位于
  `LIVOX_SDK2_ROOT` 之下；
- 驱动拆成 `foray_livox_core_objects`（无 SDK）与
  `foray_livox_backend_objects`（real/stub）；mock 测试只链 core；
- `FORAY_DRIVER_LIVOX_HAS_SDK` 作为最终 target 的 PUBLIC 定义（0/1），
  描述已安装产物能力；下游用 `#if` 判断；
- 静态库的 SDK 依赖以 `$<LINK_ONLY:LivoxSDK2::shared>` 导出，
  下游由包 Config 的 `find_dependency(LivoxSDK2)` 重新定位。

Reason:
- 符合组织规范：不提交厂商 SDK、不使用 submodule、不做无必要的 fork；
- 开发/CI 默认零 SDK 依赖，真实构建又能在任意部署前缀复现；
- imported target 消除绝对路径泄漏，`HAS_SDK` 让下游在编译期识别能力。

Alternatives:
- fork `Livox-SDK2-foray-fork`：仅在未来必须打 patch 时启用；
- 单一目标 + 源码宏切换：仍会让 mock 测试链接 SDK；
- 共享库驱动：可避免 LINK_ONLY，但会改变现有静态库部署形态。

Rejected:
- 在 `src/` 下放第三方 SDK 源码；
- 让 ON 构建的 mock 测试依赖 SDK 运行库；
- 在安装导出中写死构建机 SDK 路径。

---

## Decision

Date: 2026-10-19

Context: 实机 HIL 发现三类问题：SDK 只识别 Mid-360s（dev_type 35）但 JSON 只有
`MID360`；SDK 的 handle 与按 IP 手工拼接的字节序不一致；Mid-360 非重复扫描下
`frame_cnt` 无效导致帧永远不发布。另有 IMU 加速度单位差异。

Decision:
- 生成 JSON 同时写入 `"MID360"`（dev_type 9）与 `"Mid360s"`（dev_type 35）；
  缺少对应段时 SDK 会在检测阶段直接返回，不触发 InfoChange；
- `ipv4_to_handle` / `handle_to_ipv4` 使用 `inet_pton` / `inet_ntop`，
  handle 即 `sockaddr_in::sin_addr.s_addr`；InfoChange 仍按 serial/IP 回退匹配，
  并覆盖为 SDK 的实际 handle；
- 新增 `LidarConfig::frame_duration_ms`（默认 100 ms）；帧边界为
  `frame_cnt` 变化、`udp_cnt == 0` 或时间窗口到期；
- Livox IMU 原始加速度单位 g，驱动乘以 `9.80665` 转为 HAL 约定的 m/s²；
- 默认 JSON 使用**单播**（不写 `multicast_ip`），组播作为可选自定义配置。

Reason:
- 设备类型键与 handle 表示是 SDK 的硬约束，靠猜测或手工字节序会静默失败；
- 非重复扫描没有可靠帧号，时间窗口是唯一稳定的切帧方式；
- 单位统一在驱动边界完成，避免上层误用 g；
- 单播避开网桥 IGMP/组播转发问题，部署更稳。

Alternatives:
- 每个机型单独维护 JSON：可用，但驱动在启动前不知道设备型号；
- 用 `frame_timeout_ms` 兼作时间窗口：语义混淆，且空闲超时可能更长；
- 在 HAL 保留 g：会让 ROS `sensor_msgs/Imu` 等消费者得到错误量纲。

Rejected:
- 只写 `MID360` 并让 Mid-360s 走默认配置；
- 在回调里按 `(a<<24)|...` 解释 handle；
- 仅依赖 `frame_cnt` 切帧；
- 把 IMU 原始 g 直接透传给上层。

---

## Decision

Date: 2026-10-20

Context: 手动测试需要观察驱动长时间取流是否稳定。原 `foray_livox_dump` 必须先在内存里
攒满 N 帧再一次性写 PCD，帧数一大就吃内存，也看不到实时进度；PCD 头要求预先声明
`WIDTH`/`POINTS`，天然不适合不定长流。

Decision:
- 新增 `--stream`：点云逐点写 stdout，每行 7 列
  `frame x y z intensity time_s line`，首行 `#` 表头；每帧 `fflush`；
- 每帧统计（seq / 点数 / 间隔 / 帧率）与收尾汇总写 **stderr**；
- `--stream --frames 0` 表示持续输出；`SIGINT` / `SIGTERM` 只置标志，
  由主循环收尾并打印汇总；
- 管道 `EPIPE`（下游如 `head -1` 提前关闭）视为正常结束，退出码 `0`；
  其他写失败（如磁盘满）退出码 `7`；
- 流式模式**不写** PCD/PPM，点云不累积在内存；
- `--self-test --stream` 复用同一写出路径，使格式可在无雷达时验证。

Reason:
- 行式文本流不需要预先知道总点数，适合不定长、可持续的测试场景；
- 统计走 stderr 才能让 stdout 直接被管道/解析器消费；
- 逐帧 flush 让 HIL 观察到的输出节奏与传感器一致；
- 区分 EPIPE 与真错误，避免管道用法被误判为失败。

Alternatives:
- 每帧写一个 PCD 文件：管道场景不友好，还会堆积大量文件；
- 二进制流：带宽更省，但无法用 `head` / `awk` 直接检查；
- 让 `--frames N` 兼作持续模式：`N` 语义不清，容易误用。

Rejected:
- 在 stdout 混入统计日志（破坏下游解析）；
- 依赖 SIGPIPE 默认行为终止进程（管道关闭被判成失败）。

---

Date: 2026-10-20
Title: 雷达网络自举（NM 优先 + polkit helper）与参数注入读回
Status: accepted
Context: 比赛现场重配雷达链路要操作手手工 `sudo ip link set ... master br-lidar`，
既慢又易错；本机 NetworkManager 还会在几秒内拆掉手工创建的 slave 关系，
桥变成空的、抓包为 0。雷达参数（IP / 安装姿态 / 主机端口）此前只能改 JSON，
改错 IP 会直接失联且无法从雷达侧读回验证。

Decision:
- 新增 `foray_platform::net`：只读 `probe_topology()` → 纯函数 `plan_link()` →
  `ensure_link()` 执行，失败按逆序回滚；
- 后端**NetworkManager 优先**（创建 `foray-<iface>-slave` profile，重启保留），
  无 NM 时回退 rtnetlink；两者都实现同一 `NetBackend` 接口，可 mock；
- 特权不用 setuid：`foray-net-helper` 装到 libexec，polkit 动作
  `org.foray.netconfig.bootstrap` 用 `exec.path` 只放行该程序，参数白名单在工具内；
  可选 `60-foray-netconfig.rules` 让 `foray` 组免密；
- 雷达参数注入走 `LivoxControl`：总尝试 3 次、间隔 100 ms、单次等待 800 ms；
  只对 kTimeout/kSdkError/kBusy/kNoData 重试；
- 写参数后**必须读回**：`QueryLivoxLidarInternalInfo` 解析 `kKeyLidarIpCfg`
  并比对；`foray_livox_config` 默认 `--dry-run` 只读，`--set-lidar-ip` 显式才改。

Reason:
- NM 是这台机（也是大部分桌面 Ubuntu）真正的网络配置权威，绕过它必然被拆；
- polkit + 参数白名单把 root 能力收敛到一个程序，比 setuid 更容易审计；
- 现场改 IP 一旦写错就无法再连上，读回校验是唯一可靠的防线；
- 纯函数规划 + 假后端让"顺序/幂等/回滚"可以在没有 root 的 CI 里测。

Alternatives:
- 直接 `sudo ip link`（现状）：被 NM 拆掉，且要求操作手会敲命令；
- setuid helper：审计面大、升级易出漏洞；
- 只靠 netlink：桌面机上与 NM 打架；
- 只改 JSON 不读回：无法确认雷达真的接受了新 IP。

Rejected:
- 让 helper 接受任意 shell 命令（提权面过大）；
- 自动改写雷达 IP 而无显式开关（失联风险不可接受）。

## 2026-10-20 流式输出独占 stdout

Background:
- Livox SDK2 的 console 日志写 fd 1；`foray_livox_dump --stream` 实机输出里混入 18 行
  `[console] [info] ...`，管道消费方无法直接解析。

Decision:
- 流式模式启动时 `dup(STDOUT_FILENO)` 保存真实 stdout，`freopen("/dev/null", "w", stdout)`
  把 fd 1 让给 SDK 日志；点云与表头一律用 `write_all()` 写到保存的 fd（`g_stream_fd`）；
- 非流式模式保持原样（日志与 PCD/PPM 汇总都在 stdout）。

Consequences:
- 管道输出纯数据；`| head -1` 的 EPIPE 仍按正常结束（exit 0），真写失败 exit 7；
- 代价：流式模式下看不到 SDK 日志（需要时改用非流式或抓包）。

Alternatives:
- 配置 SDK 静音日志：SDK 头文件未暴露稳定的运行时关闭接口，且日志行跨多个模块；
- 点云改走 stderr：会与统计争用同一通道，违背既有约定。

Rejected:
- 依赖 `2>/dev/null` / `stdbuf` 等调用方约定（工具自身应保证输出契约）。
