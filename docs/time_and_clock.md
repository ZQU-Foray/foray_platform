# 时间与时钟

> 所属层：L1 平台基础 · 对应横切面：X3 时间与同步
> 代码：`foray_platform/include/foray/platform/time/`

## 1. 目标

- 提供**与 ROS 无关**的时间类型与时钟接口，使 L1 库可以脱离 ROS 单测；
- 把 `use_sim_time` 的语义固定在抽象层，而不是散落在各节点；
- 明确「哪些时间可以用于超时与安全判定」，避免仿真时间停流时急停失效。

## 2. 类型

| 类型 | 含义 | 精度 |
|---|---|---|
| `Duration` | 有符号时间跨度 | 纳秒（`int64_t`） |
| `Timestamp` | 时间点 | 纳秒（`int64_t`） |
| `Clock` | 时钟接口 | — |
| `ClockDomain` | 时钟域标签 | — |

`Duration` 与 `Timestamp` 是强类型，不能相互隐式转换。常用运算：

```cpp
using namespace foray::platform::time;

const Timestamp now = Clock::now();                 // 伪代码：通过注入的 Clock 获取
const Timestamp deadline = now + Duration::from_milliseconds(20);
const Duration remaining = deadline - now;
```

## 3. 时钟域

| 域 | 实现 | 单调 | 纪元 | 用途 |
|---|---|---|---|---|
| `kSystemWall` | `SystemClock` | 否 | Unix 纪元 | 消息时间戳、与外部系统对时 |
| `kSteady` | `SteadyClock` | 是 | 未指定 | 测量间隔、实时循环周期、本地看门狗 |
| `kSimulated` | `ManualClock` | 否 | 仿真定义 | `use_sim_time=true`、回放、单测 |

**铁律：不同域的时间点不得相互比较或相减。** `Timestamp` 不携带域标签，
这是为了保持运算零开销；正确性由调用方保证。需要跨域时，先在边界处显式转换。

## 4. `use_sim_time` 语义

ROS 2 的 `use_sim_time` 在启动期读取一次，**运行期不切换**。映射如下：

| 场景 | `use_sim_time=false` | `use_sim_time=true` |
|---|---|---|
| 节点时间 / 消息时间戳 | `SystemClock` | 由 `/clock` 驱动的 `ManualClock` |
| 本地间隔测量 | `SteadyClock` | `SteadyClock` |
| 安全硬路径（急停） | `SteadyClock` | **`SteadyClock`** |

### 4.1 仿真时钟可能未就绪

`use_sim_time=true` 时，在第一帧 `/clock` 到达之前：

- `clock.is_ready() == false`；
- `clock.now()` 返回零时间点。

调用方**必须**检查 `is_ready()`，不得把零时间点当成真实时间参与超时判定。
典型写法：

```cpp
if (!clock.is_ready()) {
    // 首帧 /clock 尚未到达：保持上一次状态或等待，不启动超时计时。
    return;
}
```

`ManualClock` 通过 `set()` 表示「收到一帧外部时间」，通过 `advance()` 表示推进。
ROS 适配层应在 `/clock` 回调中调用 `set(now)`，不要自行累加时间。

### 4.2 仿真时间可能跳变

- `ManualClock::is_steady() == false`，`set()` 可以回退；
- `SystemClock::is_steady() == false`，NTP 校时可能回退。

因此，凡是以「截止时刻 = now + timeout」表达的判定，必须在每次循环重新读取
`now()`，不能假设时间只增不减。

### 4.3 急停硬路径必须使用 `SteadyClock`

急停的最终安全降级（`cmd_vel` 超时归零、发射机构断电）**不依赖中间件**，
也不能依赖仿真时间：

- 仿真器暂停或 `/clock` 停流时，仿真时间会停止；若急停定时器使用仿真时间，
  超时永远不会触发；
- `std::chrono::steady_clock` 不受系统校时影响，是唯一适合本地看门狗的时间源。

该约束在后续实现 HAL 执行器抽象时必须落实。

## 5. ROS 2 适配（后续任务）

本单元只交付抽象，未引入 `rclcpp`。后续适配层计划：

1. 启动期读取 `use_sim_time` 参数；
2. `false`：`SystemClock` 负责节点时间；
3. `true`：订阅 `/clock`，回调中调用 `ManualClock::set()`；
4. 以 `Clock&` 注入到需要时间的模块，模块不直接持有 `rclcpp::Clock`；
5. 适配层单独成包，`foray_platform` 的 core target 永远不链接 `rclcpp`。

## 6. 线程安全

- `SystemClock` / `SteadyClock`：无状态，可并发调用；
- `ManualClock`：`now()`、`is_ready()`、`set()`、`advance()` 均为原子操作，
  可并发调用；
- `Clock` 接口的实现必须保证上述读取接口可并发调用。

## 7. 测试覆盖

`foray_platform/test/time/clock_test.cpp` 覆盖：

- `Duration` / `Timestamp` 运算与比较；
- 三种时钟的域、单调性与就绪语义；
- 仿真时间未就绪、回退、跳变。
