# 相机配置架构：键空间、分级与动态参数

> 适用范围：`foray_platform` 的 HAL 相机层，以及 `foray_platform_driver_livox` /
> `foray_platform_driver_hikvision` 两个驱动包。
> 目标读者：上游 `robots` 仓的配置编写者、驱动实现者。

## 1. 目标

1. **上游只写 conf**：`robots` 仓不需要 C++ 代码就能调整相机行为，键空间覆盖
   MVS 常用节点的绝大部分。
2. **参数可分层覆盖**：全兵种共享 → 兵种/机型 → 实车，逐层覆盖，来源可追溯。
3. **支持动态参数**：能在运行期安全修改的项，改 conf 后热加载生效；
   不能在运行期改的项，明确拒绝并给出原因，而不是静默忽略。
4. **能力协商**：驱动声明"这台相机支持哪些键"，配置了不支持项时在**配置期**
   报错（fail fast），而不是连上之后运行时才发现。

## 2. 三层关系

```
robots/config/*.yaml          ← 上游只碰这一层（纯 YAML）
        │  ConfigResolver（include / ${ENV} / 分层覆盖 / provenance）
        ▼
param::ParamSchema            ← 键的声明：类型、范围、枚举、hot_reloadable
        │  config_binder（键 → hal::CameraConfig）
        ▼
hal::CameraConfig             ← 类型安全的结构体
        │  driver::make_camera()
        ▼
hal::Camera（含 apply(CameraTuning)）  ← 运行期热改入口
```

`ConfigResolver` 只负责**加载 + 校验 + 记录来源**，不负责热加载；
热加载由 `param::ParamManager` 负责，两者共用同一个 `ParamSchema`。

## 3. 配置键空间

前缀统一为 `camera.`。下表「级」列决定该键能否热改，映射到
`ParamSpec::hot_reloadable`（`live` → `true`，其余 → `false`）。

### 3.1 设备选择（`static`）

| 键 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `camera.serial` | string | `""` | 最高优先级的匹配条件 |
| `camera.user_defined_name` | string | `""` | 次优先级 |
| `camera.model` | string | `""` | 与 `camera.index` 组合 |
| `camera.ip_address` | string | `""` | 仅 GigE 有效 |
| `camera.transport` | enum `any\|gige\|usb3` | `any` | 限定枚举的传输层 |
| `camera.index` | int ≥ 0 | `0` | 显式回退，不单独使用 |

匹配优先级：`serial` > `user_defined_name` > `(model + index)` > `ip_address`。
未提供任何条件时驱动返回 `kInvalidArgument`，**不得**默认取第 0 个设备。

### 3.2 几何与像素（`streaming`）

| 键 | 类型 | 默认 | MVS 节点 |
|---|---|---|---|
| `camera.width` | int | `0`（设备当前值） | `Width` |
| `camera.height` | int | `0` | `Height` |
| `camera.offset_x` | int | `0` | `OffsetX` |
| `camera.offset_y` | int | `0` | `OffsetY` |
| `camera.binning_horizontal` | int | `1` | `BinningHorizontal` |
| `camera.binning_vertical` | int | `1` | `BinningVertical` |
| `camera.decimation_horizontal` | int | `1` | `DecimationHorizontal` |
| `camera.decimation_vertical` | int | `1` | `DecimationVertical` |
| `camera.pixel_format` | enum | `bayer_rg8` | `PixelFormat` |

`width/height` 为 0 表示沿用设备当前值；组合约束（`offset + size ≤ 传感器范围`）
由驱动在写入时按设备实际能力校验，**不**在 schema 里硬编码传感器尺寸。

`pixel_format` 枚举：`mono8`、`bayer_rg8`、`bayer_gr8`、`bayer_gb8`、`bayer_bg8`、
`mono10`、`mono12`、`bayer_rg10`、`bayer_rg12`、`rgb8`、`bgr8`、`yuv422`。
驱动对不支持的格式返回 `kNotSupported`，由上层显式失败。

### 3.3 采集控制（`live`）

| 键 | 类型 | 默认 | MVS 节点 |
|---|---|---|---|
| `camera.frame_rate_hz` | double | 空（不改） | `AcquisitionFrameRate` |
| `camera.frame_rate_enable` | bool | `true` | `AcquisitionFrameRateEnable` |
| `camera.exposure_us` | double | 空（自动/当前值） | `ExposureTime` |
| `camera.exposure_auto` | enum `off\|once\|continuous` | `off` | `ExposureAuto` |
| `camera.exposure_auto_lower_us` | double | 空 | `AutoExposureTimeLowerLimit` |
| `camera.exposure_auto_upper_us` | double | 空 | `AutoExposureTimeUpperLimit` |
| `camera.gain_db` | double | 空 | `Gain` |
| `camera.gain_auto` | enum `off\|once\|continuous` | `off` | `GainAuto` |
| `camera.gain_auto_lower_db` | double | 空 | `AutoGainLowerLimit` |
| `camera.gain_auto_upper_db` | double | 空 | `AutoGainUpperLimit` |

`frame_rate_hz` 在触发模式下由驱动忽略（与 `TriggerMode` 非 `off` 互斥），
并输出一次 `kNotSupported` 级别的一行日志，不视为错误。

`exposure_auto` 非 `off` 时 `exposure_us` 由设备接管，上层写入的 `exposure_us`
被忽略；两者同时配置时驱动以 `exposure_auto != off` 为准并提示。

### 3.4 触发（`streaming` / `live` / 动作）

| 键 | 类型 | 默认 | 级 | MVS 节点 |
|---|---|---|---|---|
| `camera.trigger_mode` | enum `off\|software\|line0..line3` | `off` | streaming | `TriggerMode` |
| `camera.trigger_activation` | enum `rising\|falling\|level_high\|level_low` | `rising` | streaming | `TriggerActivation` |
| `camera.trigger_source` | enum `software\|line0..line3` | 空 | streaming | `TriggerSource` |
| `camera.trigger_debounce_us` | double | 空 | live | `LineDebouncerTime` |

`trigger_software`（命令，不是配置）由 HAL 的
`Camera::trigger()` 暴露，见 §5。它在 conf 里**没有**对应键。

`trigger_source` 缺省时由 `trigger_mode` 推导：`software` → `TriggerSource=Software`，
`lineN` → `TriggerSource=LineN`。显式配置时必须以 `trigger_mode` 为 `software`/同号
线路为前提，否则配置期报错。

### 3.5 传输层（GigE 专有，`static`）

| 键 | 类型 | 默认 | MVS 节点 |
|---|---|---|---|
| `camera.packet_size` | int | 0（不改） | `GevSCPSPacketSize` |
| `camera.inter_packet_delay_us` | int | 空 | `GevSCPD` |
| `camera.heartbeat_timeout_ms` | int | 空 | `GevHeartbeatTimeout` |
| `camera.net_export` | int | 空（自动） | 枚举结果里的 `nNetExport` |

`packet_size` 默认 0 表示不动设备值；巨帧场景由上游显式配置（常见 9000）。
这些键在设备已打开后修改无效，必须重新 `open()`，故为 `static`。

### 3.6 缓冲与流水线（`static` / `streaming`）

| 键 | 类型 | 默认 | 级 | 说明 |
|---|---|---|---|---|
| `camera.buffer_count` | int ≥ 1 | `4` | static | HAL 帧池槽数 |
| `camera.sdk_node_count` | int ≥ 1 | `5` | streaming | `MV_CC_SetImageNodeNum` |
| `camera.pool_memory_limit_bytes` | int | `0`（不限） | static | 帧池总内存上限，0 关闭 |

### 3.7 时间与同步

| 键 | 类型 | 默认 | 级 | 说明 |
|---|---|---|---|---|
| `camera.enable_ptp` | bool | `false` | static | `GevIEEE1588` |
| `camera.timestamp_source` | enum `auto\|device\|host_receive` | `auto` | live | 时间戳来源策略 |
| `camera.device_to_host_offset_ns` | int | 空 | live | 设备→主机标定偏移 |
| `camera.exposure_to_host_latency_ns` | double | 空 | live | 曝光→接收标定延迟 |
| `camera.allow_unsynced_stamp` | bool | `true` | live | 无同步时是否允许主机时刻填充 |
| `camera.frame_id` | string | `"camera"` | live | `ImageFrame::frame_id` |

`timestamp_source` 语义：
- `auto`：PTP 从时钟在线 → 设备时间；否则按 `device_to_host_offset_ns` →
  再否则按 `allow_unsynced_stamp`。即 `docs/driver_hikvision.md` §7 的三条路径。
- `device`：强制用设备计数换算，时钟不可用时 `sync = kNoSync`（**不**回退主机时刻）。
- `host_receive`：强制主机接收时刻。

### 3.8 键总览

`static` 12 项、`streaming` 13 项、`live` 15 项，合计 40 个键（含设备选择 6 项）。

## 4. 配置分级与生效语义

| 级 | 何时可改 | 变更后的动作 | `hot_reloadable` |
|---|---|---|---|
| `static` | 仅 `open()` 前 | 必须 `close()` → 改 `CameraConfig` → `open()` | `false` |
| `streaming` | `open()` 后、`start()` 前；或 `stop()` 后 | `stop()` → 应用 → `start()` | `false` |
| `live` | 任意时刻（含取流中） | 直接写设备节点 | `true` |

分级不改变 `ParamManager` 的整批语义：一批 `reload()` 里只要有一个
`hot_reloadable=false` 的键值发生变化，**整批失败**并保持原快照
（这是现有实现，不修改）。

`live` 键的修改通过变更通知下发给驱动，见 §5。

## 5. 动态参数契约

### 5.1 `param` 层：变更通知

`ParamManager` 新增监听器（非实时线程回调，在调用 `reload()` 的线程上同步执行）：

```cpp
struct ParamChange {
    std::string name;      // 点分键名
    ParamValue old_value;
    ParamValue new_value;
};

using ParamChangeListener = std::function<void(const std::vector<ParamChange>&)>;

/// @brief 注册变更监听器；返回的 id 用于注销。reload() 成功且值确实变化时回调。
ListenerId add_change_listener(ParamChangeListener listener);
void remove_change_listener(ListenerId id);
```

约束：
- **只通知真正变化的键**（按值比较），未变化的键不出现。
- 监听器在 `reload()` 返回前被调用；监听器抛异常时被捕获，不影响 `reload()` 结果，
  但会记录一条错误日志并继续通知后续监听器。
- 监听器在持锁状态下不得调用 `reload()`（文档标注，不做运行时检测）。

### 5.2 `hal` 层：`Camera::apply`

```cpp
/// @brief 运行期可热改的字段集合。未设置的字段保持不变。
struct CameraTuning {
    std::optional<double> exposure_us;
    std::optional<double> gain_db;
    std::optional<std::optional<double>> frame_rate_hz;  // 外层有值=要改，内层空=关闭限速
    std::optional<TriggerMode> trigger_mode;
    std::optional<double> trigger_debounce_us;
    std::optional<TimestampSource> timestamp_source;
    std::optional<std::int64_t> device_to_host_offset_ns;
    std::optional<double> exposure_to_host_latency_ns;
    std::optional<bool> allow_unsynced_stamp;
    std::optional<std::string> frame_id;
};
```

`Camera::apply(const CameraTuning&)` 返回 `Result<void>`；部分字段失败时返回首个错误，
**已成功的字段不回滚**（设备节点写入是逐项事务，回滚反而引入更复杂的状态），
但必须通过日志明确列出哪一项失败。

`Camera` 新增能力表：

```cpp
/// @brief 该相机支持的配置键集合（由驱动在 open() 成功后填充）。
virtual const std::vector<std::string>& supported_keys() const = 0;
```

上游在配置期用 `foray-platform camera-config --check <file>`（后续提供）比对
`supported_keys()`，配置了不支持项即报错。

### 5.3 驱动层：订阅与映射

```cpp
// 伪代码
manager.add_change_listener([&](const std::vector<ParamChange>& changes) {
    CameraTuning tuning;
    bool any = false;
    for (const auto& c : changes) {
        if (!starts_with(c.name, "camera.")) continue;
        if (auto ok = tuning_from_change(c, tuning)) any = true;
    }
    if (!any) return;
    if (auto r = camera.apply(tuning); !r) log_error(...);
});
```

注意：`apply` 可能被并发调用。驱动的 `apply` 必须自己对设备写入加锁
（取流线程已经在读，设备节点写入与取流可以并发，但多个 `apply` 之间要串行）。

### 5.4 上游使用示例

`robots/config/shared.yaml`：

```yaml
camera:
  serial: DB0985207
  width: 1440
  height: 1080
  pixel_format: bayer_rg8
  exposure_us: 3000.0
  gain_db: 4.0
  trigger_mode: software
  enable_ptp: true
```

`robots/config/sentry.yaml`（覆盖）：

```yaml
include: shared.yaml
camera:
  exposure_us: 1500.0     # 哨兵需要更短曝光
```

运行期动态调整（不在 conf 里写死）：

```bash
# 或由上层通过 ParamManager::reload() 触发
foray-platform camera-tune --key camera.exposure_us --value 2500
```

## 6. 与 `docs/driver_hikvision.md` §7 的关系

§7 的时间语义三条路径由 `camera.timestamp_source` 显式选择，`auto` 保留原来的
优先级推导。两者不冲突：§7 描述**算法**，本文描述**开关**。

## 7. 落地清单（已完成，2026-10-24）

- [x] `hal/config.hpp`：`CameraConfig` 新增字段、`CameraTuning`、`TimestampSource`
- [x] `Camera` 接口：`apply()` 与 `supported_keys()`
- [x] `config_binder.cpp`：40 个键的 schema 与赋值 + 交叉校验
- [x] `param/manager`：`add_change_listener` / `remove_change_listener`
- [x] `foray_platform_driver_hikvision`：MVS 节点映射与 `apply` 实现
- [x] `docs/driver_hikvision.md` §6 改为指向本文

驱动侧的实际消费情况（节点名、`0` 值语义、`apply()` 的内层空值含义）见
[`driver_hikvision.md`](driver_hikvision.md) §6 与 §13；两处需要注意的实机约束：

- `width`/`height` 为 `0` 表示「保留设备当前分辨率」，驱动**不会下发**（裸写 0 会被
  MVS 判成 `MV_E_GC_RANGE`）；
- `exposure_us`/`gain_db` 帧内回报可能为 0（未开 chunk），驱动退回读回的节点值，
  所以 `ImageFrame::exposure_time_us` 是「设备当前配置」而非逐帧真值。
