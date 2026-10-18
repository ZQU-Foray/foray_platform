# Livox 驱动（foray_platform_driver_livox）

> 依赖：`foray_platform::hal` · Livox SDK2 v1.3.1（可选）
> 目标机型：Mid-360 / Mid-360s（4 线）、HAP（6 线）

## 1. 包结构

```text
foray_platform_driver_livox/
├── include/foray/platform/driver/livox/lidar.hpp   公共工厂（无 SDK 类型）
├── src/livox/
│   ├── lidar.cpp               生命周期、配置、取流线程
│   ├── lidar_impl.hpp
│   ├── livox_runtime.cpp/.hpp  进程级 SDK 单例、handle 路由、回调接收器
│   ├── livox_api.hpp           SDK 函数表抽象
│   ├── livox_api_real.cpp      真实 SDK（仅 FORAY_BUILD_DRIVER_LIVOX=ON）
│   ├── livox_api_stub.cpp      stub（默认，open 返回 kSdkUnavailable）
│   ├── packet_parser.cpp/.hpp  36 字节头 + 点/IMU 解析
│   ├── frame_assembler.cpp/.hpp frame_cnt 组帧、逐点时间、帧池写入
│   └── config_json.cpp/.hpp    生成 Livox 网络配置 JSON
└── test/livox/                 mock 函数表与单元测试
```

公共头只暴露 `make_lidar(const hal::LidarConfig&)`，不出现任何 SDK 类型。

## 2. 线程模型

```text
SDK 回调线程
  └─ CallbackSink::push()：长度校验 → 打接收时间戳 → memcpy 到 SpscQueue
解析线程（每个雷达一个）
  └─ try_pop → 解析头 → IMU 入队 / 点云写入 FramePool → frame_cnt 变帧 → publish
消费线程（每个雷达一个）
  └─ read() 取最新帧租约；read_imu() 取 IMU 队列
控制线程
  └─ open/start/stop/close/recover；commands 不在回调里执行
```

- 回调路径不加锁、不分配、不打日志；队列满时计数并丢包；
- 原始包环默认 4096 槽 × 1500 B（约 6 MB/雷达）；
- 解析线程轮询间隔 200 µs，10 Hz 点云下额外延迟可忽略；
- `FramePool` 采用最新帧邮箱，消费者慢时丢旧帧并累计 `dropped`。

## 3. SDK 运行时与多雷达

Livox SDK2 的 `LivoxLidarSdkInit/Uninit` 是**进程级单例**，不能并发重复调用。

`LivoxRuntime`：

- 引用计数：第一个 `Lidar::open()` 初始化并 `LivoxLidarSdkStart()`，
  最后一个 `close()` 反初始化；
- 所有雷达共享一份 JSON 配置与全局回调；
- 回调按 `handle`（雷达 IP 的网络字节序）路由到 `CallbackSink`；
- 只配置 `serial` 时，`InfoChangeCallback` 按序列号/ IP 绑定 handle；
- `close()` 不立即销毁 sink，而是移入墓地；`LivoxLidarSdkUninit()` 返回后
  再统一释放，避免回调在飞行时访问已释放内存；
- 进程内只允许一个 `LivoxApi` 实现；混用真实/stub/mock 返回 `kNotSupported`。

## 4. 包解析

固定头 36 字节：

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0 | version | 协议版本 |
| 1 | length | 头 + 载荷总长（小端 u16） |
| 3 | time_interval | 点间隔，单位 0.1 µs |
| 5 | dot_num | 包内点数 |
| 7 | udp_cnt | 帧内包序号，0 为首包 |
| 9 | frame_cnt | 帧号（u8，回绕） |
| 10 | data_type | 0x00 IMU / 0x01 高精度 / 0x02 低精度 / 0x03 球坐标 / 0x11 双回波 |
| 11 | time_type | 0 NoSync / 1 PTP / 2 GPS / 3、4 PPS |
| 28 | timestamp | 首点时间（ns） |

点转换：

- 高精度 `int32 mm` → m；低精度 `int16 cm` → m；
- 球坐标：`depth` mm、`theta/phi` 0.01°；`x = d·cosθ·cosφ`，`y = d·sinθ·cosφ`，`z = d·sinφ`；
- `line = i % line_num`，与 `livox_ros_driver2` 一致（按包内点序号）；
- 逐点时间：`(i × time_interval × 100) / dot_num` ns，先乘后除避免累计误差；
- 双回波 `0x11` 在 SDK v1.3.1 公共头中无结构定义，返回 `kUnsupportedFormat`。

## 5. 组帧

- `frame_cnt` 变化或 `udp_cnt == 0` 时结束上一帧；
- 首包的 `timestamp` 作为 `base_time` / `device_base_time_ns`；
- 点缓冲满时丢弃当前帧剩余包，`points_dropped` 累计；
- 无空闲帧槽时丢帧并 `dropped_frames++`，不阻塞解析线程；
- 超过 `frame_timeout_ms` 无新包时发布当前帧（防止最后一帧滞留）；
- `stop()/close()` 时 `flush()` 当前帧；
- 输出传感器安装系，外参不在此处应用。

## 6. 时间语义

| `time_type` | HAL `TimestampSync` | `base_time` / `stamp` |
|---|---|---|
| 0 | `kNoSync` | 置零，保留 `device_base_time_ns` |
| 1 | `kPtp` | 设备时间直接作为绝对时间 |
| 2 | `kGpsPps` | 同上 |
| 3、4 | `kPps` | 置零（PPS 不保证 Unix 纪元） |
| 其他 | `kUnknown` | 置零，不猜测 |

- `receive_stamp` 始终是 `SteadyClock` 接收时刻；
- IMU 单独带 `stamp` / `device_time_ns` / `sync`；
- 驱动**不会**用主机时间替换设备时间；需要绝对时间时由 PTP/GPS 或上层换算；
- 与 `livox_ros_driver2` 的 NoSync 行为不同，避免静默混用时间域。

## 7. 配置 JSON

`config_json_path` 为空时，驱动按 `host_ip` 生成：

```json
{
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100, "push_msg_port": 56200,
      "point_data_port": 56300, "imu_data_port": 56400, "log_data_port": 56500
    },
    "host_net_info": [{
      "host_ip": "192.168.1.51", "multicast_ip": "224.1.1.5",
      "cmd_data_port": 56101, "push_msg_port": 56201,
      "point_data_port": 56301, "imu_data_port": 56401, "log_data_port": 56501
    }]
  }
}
```

- 端口取 Livox-SDK2 v1.3.1 sample 默认值；
- 文件权限 `0600`，目录默认 `$XDG_RUNTIME_DIR/foray_livox`；
- 生成前检查 `host_ip` 是本机网卡地址；自定义 JSON 时跳过该检查；
- 多雷达共用一个 host 配置，SDK 按 handle 区分数据来源。

## 8. 生命周期与恢复

```text
kClosed -> kOpening -> kOpen -> kStreaming -> kStopping -> kOpen -> kClosed
              |            |
              +-> kError <-+
                     |
                kRecovering -> kOpen ...
```

- `open()`：校验配置 → 初始化 SDK → 建立队列/帧池 → 注册 sink；
- `start()`：等待 `InfoChange`（最长 5 s）→ 下发工作模式/点类型/扫描模式
  （5 次重试，间隔 100 ms）→ 启动解析线程；
- `stop()`：置 sink 非活跃、停止并 join 解析线程、flush 当前帧；
- `close()`：幂等；释放队列/帧池，注销 sink，最后一个实例触发 SDK Uninit；
- `recover()`：stop → 释放 → 按 100/200/400/800/1600 ms 退避重新 open/start；
  成功后 `reconnects++`；
- 掉线期间已发出的 `FrameLease` 仍有效（帧池由 `shared_ptr` 保活）。

## 9. 错误映射

| 场景 | `ErrorCode` |
|---|---|
| 缺少 serial/ip、非法参数、host_ip 不合法 | `kInvalidArgument` |
| 未启用 SDK（stub） | `kSdkUnavailable` |
| SDK init/命令失败 | `kSdkError`（`vendor_code` 保存 `livox_status`） |
| 等待设备上线超时 | `kTimeout` |
| 读取无数据 | `kTimeout` / `kNoData` |
| 未 open/start | `kNotOpen` |
| 双回波、未知点类型 | `kUnsupportedFormat` |

## 10. 测试

`foray_platform_driver_livox_tests`（27 个用例，不需要 SDK/硬件）：

- `packet_parser_test`：头字段、高/低/球坐标转换、逐点时间、line、IMU、
  time_type 映射、越界与双回波；
- `config_json_test`：端口、host_ip、文件权限、默认路径；
- `frame_assembler_test`：变帧发布、空闲超时、NoSync 时间置零、溢出、槽位耗尽；
- `lidar_test`：完整生命周期、点云/IMU、统计、超时、多 handle 路由、恢复、
  默认工厂在 stub 下返回 `kSdkUnavailable`。

验证记录：

- 常规构建：27/27 通过；
- ASan + UBSan：27/27 通过；
- TSan：27/27 通过（mock 的 `uninit()` 模拟 SDK 等待回调归零）；
- 安装后下游 `find_package(foray_platform_driver_livox)` 可链接
  `foray_platform::driver_livox`。

## 11. 已知限制

- 真实 SDK2 构建与 HIL 尚未在本机执行（SDK 需另行安装，见
  [驱动部署](driver_deployment.md)）；
- 双回波 `0x11` 未实现，需要确认固件/SDK 版本与结构；
- `host_ip` 变化需要重新创建运行时（进程内只初始化一次）；
- 未做数据包 CRC 校验（SDK 已校验）；
- 多雷达共用一份 host JSON；如需不同主机 IP/端口，需扩展运行时配置。
