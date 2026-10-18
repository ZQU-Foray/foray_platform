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
