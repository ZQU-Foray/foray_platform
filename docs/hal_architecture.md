# L2 HAL 架构

> 所属层：L2 下行 HAL · 代码：`foray_platform/include/foray/platform/hal/`
> 包内 target：`foray_platform_hal`（与 L1 `core` 同包）

## 1. 目标

- 对上层暴露稳定的设备抽象，屏蔽 MVS / Livox SDK；
- 驱动可插拔，HAL 不依赖 ROS / OpenCV / PCL；
- 读路径不拷贝（SDK 边界允许一次入队拷贝）；
- 时间戳、错误与生命周期语义统一。

## 2. 分层与依赖

```text
foray_platform_ros  ->  foray_platform::hal  ->  foray_platform::core
                              ^
                     driver_hikvision / driver_livox
```

- `core`：时间、参数、配置栈、`Result`、SPSC；
- `hal`：设备/帧/租约/帧池/配置结构；
- 驱动：SDK 函数表 + 解析/取流；公共头不出现 SDK 类型；
- ROS：消息转换与发布，必须有一次 HAL→消息拷贝。

## 3. 设备生命周期

```text
kClosed -> kOpening -> kOpen -> kStreaming -> kStopping -> kOpen -> kClosed
              |            |
              +-> kError <-+
                     |
                kRecovering -> kOpen ...
```

约定：

- `open/start/stop/close/recover` 由**单一控制线程**调用；
- `close()` 幂等；即使中间步骤失败也必须尝试释放句柄；
- 默认不自动重连；上层按退避策略调用 `recover()`；
- 回调线程只置错误标志/入队，不调用控制接口；
- 掉线期间已发出的 `FrameLease` 仍有效（帧池由 `shared_ptr` 保活）。

## 4. 线程模型

| 线程 | 职责 | 禁止 |
|---|---|---|
| SDK 回调线程 | 拷贝原始包/帧、入队 | 解析、分配、日志、锁、重连 |
| 取流/解析线程 | SDK 调用、格式解析、填充帧池 | 业务算法 |
| 消费者线程 | `read()` 取租约、发布/算法 | 直接调用 SDK |

- 每个传感器只允许**一个写入者**和一个**消费者线程**；
- 队列使用 `SpscQueue`；最新帧邮箱在 `FramePool` 内；
- 热路径无 `new`/`malloc`、无互斥锁、无日志。

## 5. 帧池与租约

```cpp
auto pool = std::make_shared<FramePool<ImageFrame>>(4, payload_bytes);
auto writer = pool->try_acquire_writer();   // 取流线程
writer->frame().sequence = 42;
writer->publish();

FrameLease<ImageFrame> lease = pool->try_acquire_latest(); // 消费线程
```

槽位状态：

```text
kFree -> kWriting -> kPublished -> kReading -> kFree
                          |
                          +-> kFree（被更新帧替换，dropped++）
```

- **最新帧邮箱**：消费者慢时新帧替换旧帧，`dropped()` 累计；
- **槽位耗尽**：`try_acquire_writer()` 返回空，`no_slot()` 累计；
- `Writer` 未 `publish()` 就析构时自动归还槽位；
- `FrameLease` move-only；deleter 持有 `shared_ptr<FramePool>`，
  因此租约可以晚于设备销毁；
- 帧池必须由 `std::shared_ptr` 持有（`std::make_shared<FramePool<T>>`）；
  `try_acquire_latest()` 的 `noexcept` 边界不依赖该约定被遵守——违反时返回
  空租约而非抛出 `bad_weak_ptr`（Debug 构建以断言报告误用），不会触发
  `std::terminate`；失败发生在槽位状态变更之前，已发布帧不会被卡在 `kReading`；
- `payload()` 是预分配字节缓冲；图像/点云的 `data`/`points` 指向它。

### 5.1 零拷贝边界

- 海康：`GetImageBuffer` → 一次 memcpy 到 HAL 池 → `FreeImageBuffer`（同线程）；
  消费者拿到 lease 后不再拷贝；
- Livox：SDK 回调一次 memcpy 原始包到环 → 解析写入点云帧槽；
  消费者拿到 lease 后不再拷贝；
- 严格「SDK 缓冲直通」模式留到后续；当前实现优先保证线程安全与可测性；
- ROS 适配仍有一次 HAL→`sensor_msgs` 的拷贝，文档中明确。

## 6. 时间戳

每帧带：

- `stamp`：绝对墙钟；无绝对同步时为零；
- `receive_stamp`：`SteadyClock` 接收时刻；
- `device_time_ns` / `device_tick_frequency_hz`：原始设备计数；
- `sync`：`TimestampSync` 枚举。

语义：

| 值 | 含义 |
|---|---|
| `kHostReceive` | 主机接收时刻，不是曝光时刻 |
| `kDeviceCalibrated` | 已按偏移/频率换算 |
| `kPtp` / `kGpsPps` | 设备时间已是绝对时间 |
| `kPps` | 只有秒脉冲基准，不保证 Unix 纪元 |
| `kNoSync` | 设备开机时间；`stamp` 置零 |
| `kUnknown` | 无法判断 |

禁止静默混用时间域；未同步时保留原始设备时间。

## 7. 错误语义

- 公共 API 返回 `Result<void>` 或 `Result<T>`；
- `Error::vendor_code` 保存 MVS/Livox 原始返回码；
- `kTimeout` / `kNoData` 是正常控制流，不是设备错误；
- 连续超时由上层决定是否 `recover()`；
- 批量配置校验使用 `ParamResult`，一次返回全部错误。

## 8. 配置绑定

HAL 只接收 `CameraConfig` / `LidarConfig`，不解析 YAML。
`hal/config_binder.hpp` 提供：

- `make_camera_param_schema()` / `make_lidar_param_schema()`：字段名与默认值；
- `bind_camera_config()` / `bind_lidar_config()`：把已校验的
  `param::ParamSnapshot` 映射为 HAL 配置。

跨字段约束在 binder 中实现：

- USB3 必须有 serial；GigE 必须有 serial 或 IP；
- 触发开启时 `frame_rate_hz` 必须为 0；
- 像素格式必须是本期支持的 8 位 Mono/Bayer；
- 雷达必须有 `ip_address` 与 `host_ip`。

设备能力（如某个 GenICam 节点是否存在）由驱动在 `open()` 时报告。

## 9. 测试

- `types_test`：枚举、像素格式、配置默认值；
- `frame_pool_test`：发布/获取、替换丢帧、槽位耗尽、未发布归还、
  租约晚于池销毁、单写单读并发；
- 驱动 mock：SDK 函数表注入，覆盖正常/坏包/超时/掉线/恢复；
- 全部测试不依赖硬件；HIL 见 `plan.md`。
