# 海康 / Livox SDK 封装调研

> 范围：L2 下行 HAL 的相机与激光雷达驱动封装。
> 结论用于后续 `foray_platform_hal` 与驱动插件的设计，不代表接口已冻结。

## 1. 结论摘要

1. **两家 SDK 都用「C 风格句柄 + 回调/阻塞取流」模型**，可以收敛到同一套 HAL：
   设备生命周期 + 类型化帧 + 时间戳契约 + 错误恢复。
2. **SDK 类型不得进入 HAL 公共头文件**。每个驱动内部包一层 `Api` 函数表
   （生产实现绑定真实 SDK，测试绑定 mock），这是 CI 无硬件、无 SDK 也能测的关键。
3. **海康 MVS 是闭源 SDK**：不要把 `.so`/`.a`/头文件提交进仓库；用系统安装或
   `FORAY_MVS_ROOT` 指定。官方提供 `aarch64` 包，Jetson 可用。
4. **Livox-SDK2 是 MIT 许可**，但**没有 CMake package config**，需要
   `find_path` / `find_library` 或 pin commit 后源码集成。Mid-360 由 SDK2 支持。
5. **`livox_ros_driver2` 官方明确声明「不推荐用于量产，仅用于测试」**。
   它适合作为解析逻辑参考，不适合作为 HAL 的运行时依赖。
6. **时间同步是封装里最难的部分**：
   - Livox Mid-360 支持 PTP/gPTP；未同步时时间戳是设备开机时间，不能与主机时间混用；
   - 参考驱动在 `NoSync` 时**静默替换为主机时间**（`GetEthPacketTimestamp`），
     这会掩盖时间域错误，HAL 不应照搬；
   - 海康 USB3 相机通常不参与 PTP，需要「主机接收时刻 + 标定偏移」或硬件触发方案。
7. **HAL 不应该依赖 OpenCV/PCL/ROS**。驱动只交付原始像素/点云缓冲，
   Bayer→BGR、点云转换由上层完成。

## 2. 现有资产与约束

### 2.1 本仓纪律

- L2 HAL 抽象在本仓，具体驱动放本仓之下的插件目录；
- 依赖方向恒为「实现依赖抽象」；
- 抽象层不得出现 ROS 类型（`foray_docs/algorithm_structure.md` §1.6）；
- 硬实时路径禁止动态内存分配与锁（§7.1）。

### 2.2 `sp_vision_25` 的海康封装（可直接参考，但不要照搬）

位置：`sp_vision_25/io/hikrobot/hikrobot.{hpp,cpp}`。

可借鉴：

- 使用 `MV_CC_GetImageBuffer` + `MV_CC_FreeImageBuffer` 配对取流；
- 有独立取流线程；
- 有 USB 复位恢复尝试。

需要改进的点：

| 问题 | 影响 |
|---|---|
| 直接选 `device_list.pDeviceInfo[0]` | 多相机时可能打开错设备 |
| 在驱动内用 OpenCV 做 Bayer→RGB | HAL 被 OpenCV 污染；取流线程承担转换开销 |
| 时间戳用 `steady_clock::now()`（取到帧之后） | 不是曝光时刻；未利用设备时间戳 |
| 取流循环 `sleep(1ms)` + 100ms 超时 | 增加延迟与抖动；`GetImageBuffer` 本身可阻塞 |
| 守护线程每 100ms 尝试 USB reset | 过于激进；`libusb` 按 VID/PID 复位会命中错误设备 |
| `capture_stop()` 中途 `return` | 后续 `CloseDevice`/`DestroyHandle` 可能被跳过 |
| 未注册异常回调 | 掉线只能靠取流失败发现 |
| 未校验帧率节点是否支持 | 不同型号能力不同，`SetFrameRate` 可能无效 |

### 2.3 Livox 现状

- `foray_sentry_nav` 当前方案是 `Livox Mid360 + livox_ros_driver2 + point_lio`；
- 组织规范（`foray_docs/repository_structure.md` §4.2）把 `livox_ros_driver2`
  列为「传感器驱动，fork + pin commit」；
- 但 `livox_ros_driver2` README 明确写着：**「not recommended for mass production
  but limited to test scenarios」**。

**建议**：HAL 直接封装 Livox-SDK2；`livox_ros_driver2` 仅作为解析逻辑参考与
对比测试工具。若组织坚持 fork 驱动，应在 `decision.md` 中记录「为什么 HAL 仍直接接 SDK」。

## 3. 统一 HAL 抽象建议

### 3.1 设备生命周期

```text
CLOSED -> OPENING -> STREAMING -> ERROR -> RECOVERING -> STREAMING
                         ^                         |
                         +-------------------------+
```

- `open()`：枚举、选择设备、配置、启动；
- `close()`：停止、释放句柄，必须幂等；
- `recover()`：重新枚举并打开；带退避，不在回调线程里做；
- 回调只置 `ERROR` 标志，不执行重连。

### 3.2 帧类型（草案）

```cpp
enum class PixelFormat { kMono8, kBayerRG8, kBayerGR8, kBayerGB8, kBayerBG8, kRgb8, kBgr8 };

enum class TimestampSync {
    kUnknown,
    kNoSync,       // 设备开机时间／无法与主机对齐
    kPtp,          // IEEE 1588 / gPTP
    kGpsPps,       // GPS/PPS
    kPps,
    kHostReceive,  // 仅主机接收时刻
};

struct ImageFrame {
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t stride;
    PixelFormat pixel_format;
    const std::uint8_t* data;      // 生命周期由 FrameLease 保证
    std::size_t size;
    std::uint64_t sequence;
    time::Timestamp host_stamp;    // 主机接收时刻（SteadyClock）
    std::uint64_t device_ticks;    // 原始设备时间戳，0 表示不可用
    TimestampSync sync;
    const char* frame_id;          // 传感器安装系，不做外参变换
};

struct PointXYZITL {
    float x, y, z;
    float intensity;
    double time_offset_ns;         // 相对 base_time
    std::uint8_t line;
    std::uint8_t tag;
};

struct PointCloudFrame {
    const PointXYZITL* points;
    std::size_t count;
    time::Timestamp base_time;
    TimestampSync sync;
    std::uint64_t sequence;
    const char* frame_id;
};

struct ImuSample {
    float gyro_x, gyro_y, gyro_z;  // rad/s
    float acc_x, acc_y, acc_z;     // m/s^2
    time::Timestamp stamp;
    TimestampSync sync;
};
```

### 3.3 驱动接口（草案）

```cpp
class Camera {
public:
    virtual ~Camera() = default;
    virtual Result<void> open(const CameraConfig&) = 0;
    virtual Result<void> start() = 0;
    virtual Result<void> stop() = 0;
    virtual Result<void> close() = 0;
    virtual Result<void> recover() = 0;
    virtual SensorState state() const = 0;
    virtual Result<void> read(FrameLease<ImageFrame>&, Duration timeout) = 0;
    virtual Result<void> set_exposure(Duration) = 0;
    virtual Result<void> set_gain(double) = 0;
    virtual SensorStats stats() const = 0;
};

class Lidar {
public:
    virtual ~Lidar() = default;
    virtual Result<void> open(const LidarConfig&) = 0;
    virtual Result<void> start() = 0;
    virtual Result<void> stop() = 0;
    virtual Result<void> close() = 0;
    virtual Result<void> recover() = 0;
    virtual SensorState state() const = 0;
    virtual Result<void> read(FrameLease<PointCloudFrame>&, Duration timeout) = 0;
    virtual Result<void> read_imu(ImuSample&) = 0;
    virtual SensorStats stats() const = 0;
};
```

`FrameLease` 持有缓冲槽的引用计数，读者用完自动归还；避免「返回裸指针后被下一帧覆盖」。

### 3.4 线程与缓冲模型

| 线程 | 职责 | 禁止 |
|---|---|---|
| SDK 回调线程 | 只做数据拷贝/入队 | 解析、分配、日志、锁、重连 |
| 取流/解析线程 | SDK 调用、格式解析、时间戳计算 | 业务算法 |
| 消费者线程 | 读帧、发布/算法 | 直接调用 SDK |

- 队列使用 SPSC/MPSC 无锁环或三缓冲；
- 满队列策略：视觉/感知取**最新帧**（丢旧），并累计 `dropped_frames`；
- 任何回调里都不做 `new`/`malloc`、`std::mutex`、`printf`。

### 3.5 SDK 隔离

每个驱动内部定义只包含**实际用到**的 SDK 函数表：

```cpp
struct MvsApi {
    decltype(&MV_CC_EnumDevicesEx2) enum_devices_ex2;
    decltype(&MV_CC_CreateHandleWithoutLog) create_handle;
    // ...
};

struct LivoxApi {
    decltype(&LivoxLidarSdkInit) sdk_init;
    decltype(&SetLivoxLidarPointCloudCallBack) set_point_callback;
    // ...
};
```

- 生产：静态绑定真实 SDK；
- 测试/CI：绑定 mock，构造正常帧、坏帧、掉线、超时；
- 好处：驱动逻辑可单测；无 SDK 的机器也能构建 HAL 与测试。

## 4. 海康 MVS SDK 封装

### 4.1 SDK 事实

- 传输层：USB3 / GigE，枚举参数分别为 `MV_USB_DEVICE` / `MV_GIGE_DEVICE`；
- 设备信息 `MV_CC_DEVICE_INFO` 含 `chSerialNumber`、`chUserDefinedName`、
  `chModelName`、`idVendor`/`idProduct`（USB3）；
- 官方提供 Linux `x86_64` 与 `aarch64` 包；Jetson 使用 `aarch64`；
- SDK 为闭源；`MVS` Linux 安装脚本通常安装 `libMvCameraControl.so`、头文件与 udev 规则；
- 本仓参考头文件位于 `sp_vision_25/io/hikrobot/include/`（含
  `MvCameraControl.h`、`CameraParams.h`）。

### 4.2 推荐调用序列

```text
MV_CC_EnumDevicesEx2
  -> 按序列号/用户自定义名匹配（不要取 [0]）
MV_CC_CreateHandleWithoutLog
MV_CC_OpenDevice
MV_CC_RegisterExceptionCallBack      // 掉线检测
MV_CC_SetImageNodeNum(3~5)           // StartGrabbing 之前
配置节点：
  TriggerMode / AcquisitionFrameRateEnable / AcquisitionFrameRate
  ExposureAuto=Off / ExposureTime
  GainAuto=Off / Gain
  BalanceWhiteAuto=Off
  PixelFormat（优先 BayerRG8 / Mono8）
MV_CC_StartGrabbing
循环：
  MV_CC_GetImageBuffer(handle, &frame, timeout_ms)
  读取 MV_FRAME_OUT_INFO_EX（宽高、像素格式、帧号、设备时间戳、主机时间戳）
  拷贝到 HAL 缓冲槽
  MV_CC_FreeImageBuffer(handle, &frame)
MV_CC_StopGrabbing
MV_CC_CloseDevice
MV_CC_DestroyHandle
```

### 4.3 关键决策

1. **像素格式**：HAL 只暴露 `PixelFormat` + 原始缓冲；
   Bayer→BGR 放视觉层。这样驱动无 OpenCV 依赖，也避免取流线程做重 CPU 工作。
2. **取流方式**：用 `GetImageBuffer` 的独立线程，超时参数取 50–100ms；
   不要 `sleep(1ms)` 轮询。回调模式也可用，但回调线程里必须只拷贝。
3. **时间戳**：
   - `MV_FRAME_OUT_INFO_EX` 同时给出 `nDevTimeStampHigh/Low`（64 位设备计数）和
     `nHostTimeStamp`；
   - 本地头文件与官方 C SDK 指南只写「Timestamp generated by host」，
     **没有给出单位**；不要假设毫秒；
   - 推荐：HAL 记录 `SteadyClock::now()`（取到帧的第一时刻）作为 `host_stamp`，
     同时保留设备计数；若需要曝光时刻，先查询设备的时间戳 tick 频率节点
     （不同传输层节点名不同，如 `TimestampTickFrequency` / `GevTimestampTickFrequency`），
     标定「设备计数 → 主机时钟」的偏移与漂移；
   - 相机-雷达融合时不要直接混用两家的原始时间戳。
4. **多相机**：按序列号选择；不要依赖枚举顺序。
5. **恢复**：
   - 异常回调置 `ERROR`；
   - `recover()` 重新枚举、重开；
   - USB reset 只能作为最后手段，且必须按序列号定位设备；频率要限流。
6. **句柄释放**：`close()` 中即使 `StopGrabbing`/`CloseDevice` 失败，
   也必须尝试 `DestroyHandle`，避免句柄泄漏。

### 4.4 构建与部署

- 不提交 SDK 二进制与头文件；用 `FORAY_MVS_ROOT`（或 `find_path`/`find_library`）定位；
- 链接 `MvCameraControl`，运行时确保 `LD_LIBRARY_PATH` 或 rpath 能找到；
- 部署时执行 MVS 安装包的 udev 规则，否则普通用户无权限打开 USB 相机；
- CI 使用 mock API，不安装 MVS SDK；
- 驱动包用 CMake 选项显式开关：`FORAY_BUILD_DRIVER_HIKVISION=ON/OFF`；
  开关为 ON 但找不到 SDK 时**直接失败**，不要静默跳过。

## 5. Livox SDK2 封装

### 5.1 SDK 事实

- 仓库：`Livox-SDK/Livox-SDK2`，MIT 许可，C++11，Linux x86/ARM；
- 当前 tag 示例：`v1.3.1`（以实际 pin 为准）；
- 支持 HAP、Mid-360(S)、Avia2、Mid-360l 等；
- 安装：`liblivox_lidar_sdk_shared.so`、`liblivox_lidar_sdk_static.a`、
  头文件到 `/usr/local`；**没有 CMake package config**；
- 核心 API（`livox_lidar_api.h`）：
  - `LivoxLidarSdkInit(config_path, host_ip, log_cfg)`
  - `LivoxLidarSdkStart()` / `LivoxLidarSdkUninit()`
  - `SetLivoxLidarPointCloudCallBack(cb, client_data)`
  - `SetLivoxLidarImuDataCallback(cb, client_data)`
  - `SetLivoxLidarInfoChangeCallback(cb, client_data)`
  - `SetLivoxLidarPclDataType(handle, type, cb, client_data)`
  - `SetLivoxLidarScanPattern` / `SetLivoxLidarDualEmit` /
    `SetLivoxLidarBlindSpot` / `SetLivoxLidarImuDataHostIPCfg`
  - `LivoxLidarRequestReset` / `LivoxLidarRequestReboot`

### 5.2 配置

SDK 要求一个 JSON 配置文件，内容包含设备类型、网络端口、主机 IP、组播地址。
Mid-360 示例：

```json
{
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": [
      {
        "host_ip": "192.168.1.5",
        "multicast_ip": "224.1.1.5",
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501
      }
    ]
  }
}
```

建议：由参数系统生成临时 JSON（或写入运行目录），并在 `open()` 前校验
`host_ip` 是否存在于本机网卡；多雷达时写多个 `host_net_info` 条目。

### 5.3 回调与解析

SDK 的点云与 IMU 共用同一个 `LivoxLidarEthernetPacket`，用 `data_type` 区分：

| `data_type` | 含义 | 点结构 |
|---|---|---|
| `0x00` | IMU | `LivoxLidarImuRawPoint`（6×float） |
| `0x01` | 笛卡尔高精度 | `LivoxLidarCartesianHighRawPoint`（int32 mm） |
| `0x02` | 笛卡尔低精度 | `LivoxLidarCartesianLowRawPoint`（int16 cm） |
| `0x03` | 球坐标 | `LivoxLidarSpherPoint` |
| `0x11` | 双回波 | `LivoxLidarDoubleEchoRawPoint` |

包字段（`livox_lidar_def.h`）：

- `time_interval`：单位 **0.1 µs**；
- `dot_num`：本包点数；
- `data_type` / `time_type`；
- `timestamp[8]`：8 字节时间戳；
- `length`：包总长；数据区长度 = `length - sizeof(LivoxLidarEthernetPacket) + 1`。

逐点时间：

```text
point_interval_ns = time_interval * 100 / dot_num
point_time_ns      = packet_timestamp + i * point_interval_ns
```

`line` 可取 `i % line_num`（Mid-360 的 `line_num` 由机型常量给出）。
**不要在 HAL 里应用外参**；输出传感器安装系，外参交给 TF。

### 5.4 时间同步

- Mid-360 支持 PTPv2 / gPTP、GPS、PPS；
- LiDAR 作为 PTP slave；主机建议用 linuxptp：
  - `ptp4l -i eth0 -l 6 -m`
  - `phc2sys -c eth0 -s CLOCK_REALTIME -O 0`
  - 网卡需支持硬件或软件时间戳（`ethtool -T eth0`）；
- 通过包头的 `time_type` 判断同步状态；**同步时为 ns**；
- 未同步时，时间戳是设备开机时间，不是 Unix 时间；
- 参考驱动 `livox_ros_driver2` 的 `GetEthPacketTimestamp` 在未同步时
  直接返回 `high_resolution_clock::now()`，即**静默换成主机时间**。
  HAL 不应这样做：应上报 `TimestampSync::kNoSync`，保留原始设备时间，
  由上层决定是否使用。

**注意枚举差异**：`livox_ros_driver2` 内部枚举是
`0=NoSync / 1=PTP或gPTP / 2=GPS`，而 Livox 协议文档给出的是
`0=NoSync / 1=PTP / 3=PPS+GPS / 4=PPS`。实现时以所用固件/SDK 头文件为准，
未知值一律按 `kUnknown` 上报，不要猜测。

### 5.5 恢复与多雷达

- `SetLivoxLidarInfoChangeCallback` 处理上线/下线；
- 掉线后可 `LivoxLidarRequestReset` 或 `LivoxLidarRequestReboot`，
  再走重新初始化；
- `handle` 实际是雷达 IP 的数值形式；多雷达按 `handle` 维护独立状态；
- 网络配置错误（host_ip 不匹配、端口冲突）通常表现为「初始化成功但无数据」，
  必须有超时诊断。

### 5.6 构建与部署

- SDK2 没有 CMake config，建议：
  - `find_path(livox_lidar_api.h)` + `find_library(livox_lidar_sdk_shared)`；
  - 或 pin tag 后用 `FetchContent` 直接链接 `livox_lidar_sdk_shared`；
- 不提交 SDK 二进制；在元仓 `.repos` 中 pin `Livox-SDK2` 的 tag/commit；
- 网络：主机网卡与雷达同网段静态 IP；放通 UDP 端口；需要组播时确认网卡与防火墙；
- `livox_ros_driver2` 只作参考/调试，不作为 HAL 运行时依赖。

## 6. 相机-雷达时间同步（跨传感器）

这是 P1 之后「一个传感器 → 一个话题」能否升级为融合链路的前提。

| 方案 | 适用 | 精度 | 代价 |
|---|---|---|---|
| Livox PTP/gPTP + 主机 PTP master | Mid-360 | 高（宣称百 ns 级） | 主机需 linuxptp、网卡时间戳 |
| 相机硬件触发 / PPS | 有同步接口的相机 | 高 | 额外线缆/同步板 |
| 主机接收时刻 + 标定偏移 | USB3 相机（如 MV-CS016-10UC） | 中（受 USB 传输抖动影响） | 需标定偏移与漂移 |
| 各自设备时间 + 离线对齐 | 回放/离线 | 取决于标定 | 不适合在线闭环 |

**建议**：

1. 所有帧都带 `TimestampSync` 与原始设备时间戳；
2. Livox 优先上 PTP；主机用 `ptp4l` + `phc2sys`；
3. 海康 USB3 相机先用主机接收时刻，并标定「相机曝光 → 主机接收」的固定延迟；
4. 若后续换成支持 PTP 的 GigE 相机，再统一到 PTP 时间域；
5. **禁止**在没有标定的情况下把设备时间与主机时间直接混用。

## 7. 测试策略

| 层级 | 内容 | 是否需要硬件 |
|---|---|---|
| HAL 单测 | 帧结构、队列、`FrameLease`、状态机 | 否 |
| 驱动单测 | mock `Api` 注入：正常帧、坏包、超时、掉线、恢复 | 否 |
| 解析单测 | Livox 高/低/球/双回波包；海康 Bayer/Mono 元数据 | 否 |
| CI | `FORAY_BUILD_DRIVER_*=OFF`，只构建 HAL + mock | 否 |
| HIL | 实机取流 N 帧；时间戳单调；拔插恢复；30 分钟无泄漏 | 是 |
| 融合冒烟 | Mid-360 + point_lio；相机 + 自瞄识别 | 是 |

## 8. 建议的包划分

```text
foray_platform_hal/                 # 纯 C++：Sensor/Camera/Lidar 接口、帧类型、FrameLease
foray_platform_driver_hikvision/    # 海康 MVS 驱动插件，依赖 hal
foray_platform_driver_livox/        # Livox SDK2 驱动插件，依赖 hal
foray_platform_ros/                 # ROS 2 适配：Image / PointCloud2 / Imu
```

依赖方向：

```text
ros -> hal <- drivers
        ^
        |
      core
```

- 驱动不依赖 ROS；
- HAL 不依赖 OpenCV/PCL；
- 驱动缺 SDK 时显式失败，不静默降级。

## 9. 实施顺序建议

1. `foray_platform_hal`：接口 + 帧类型 + `FrameLease` + mock 驱动 + 单测；
2. 海康驱动：先跑通 `GetImageBuffer` → HAL 帧 → 测试程序，不做格式转换；
3. Livox 驱动：先跑通单雷达点云/IMU → HAL 帧，再补多雷达与恢复；
4. ROS 适配：`Image` / `PointCloud2` / `Imu` 话题，完成 P1 验收；
5. 时间同步：Livox PTP；相机偏移标定；把 `TimestampSync` 暴露到诊断；
6. HIL 与 30 分钟稳定性测试。

## 10. 待确认问题

1. 相机具体型号与传输层：本地资料是 `MV-CS016-10UC`（USB3）。
   是否所有兵种都用 USB3？是否有 GigE 型号可上 PTP？
2. 相机-雷达时间同步方案：主机接收时刻 + 偏移标定，还是引入硬件触发？
3. 像素格式：Bayer 原始格式直接进 HAL（推荐），还是驱动内转换？
4. 帧缓冲：先拷贝（简单、可测），还是一开始做零拷贝（复杂、收益待测）？
5. 海康 SDK 版本与部署方式：系统安装 MVS 还是固定版本 tar 包？
6. Livox SDK2 的 pin 版本：当前建议 `v1.3.1`，需在元仓 `.repos` 固定。
7. 组织规范要求 fork `livox_ros_driver2`，但官方声明其非量产；
   需在 `decision.md` 明确「HAL 直接接 SDK2」的取舍。

## 11. 参考资料

- 海康机器人 MVS 下载中心：<https://www.hikrobotics.com/en/machinevision/service/download?module=0>
- 海康 MVS C SDK 开发指南（`MV_FRAME_OUT_INFO_EX` / `MV_CC_GetImageBuffer` 等）
- 本地参考封装：`sp_vision_25/io/hikrobot/`（含 `MvCameraControl.h`、`CameraParams.h`）
- Livox-SDK2：<https://github.com/Livox-SDK/Livox-SDK2>
- Livox 时间同步说明：<https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html>
- livox_ros_driver2：<https://github.com/Livox-SDK/livox_ros_driver2>
- 组织第三方驱动处置策略：`foray_docs/repository_structure.md` §4.2
- 本仓 HAL 纪律：`foray_docs/algorithm_structure.md` §1.6 / §7.1
