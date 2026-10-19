# Livox 驱动（foray_platform_driver_livox）

> 依赖：`foray_platform::hal` · Livox SDK2 v1.3.1（可选）
> 目标机型：Mid-360 / Mid-360s（4 线）、HAP（6 线）

## 1. 包结构

```text
foray_platform_driver_livox/
├── cmake/
│   ├── FindLivoxSDK2.cmake     暴露 LivoxSDK2::shared
│   └── foray_platform_driver_livoxConfig.cmake.in
├── include/foray/platform/driver/livox/lidar.hpp   公共工厂（无 SDK 类型）
├── src/livox/
│   ├── lidar.cpp               生命周期、配置、取流线程（核心对象）
│   ├── lidar_impl.hpp
│   ├── lidar_public_factory.cpp 公共 make_lidar（后端对象）
│   ├── livox_runtime.cpp/.hpp  进程级 SDK 单例、handle 路由、回调接收器
│   ├── livox_api.hpp           SDK 函数表抽象
│   ├── livox_api_real.cpp      真实 SDK（仅 FORAY_BUILD_DRIVER_LIVOX=ON）
│   ├── livox_api_stub.cpp      stub（默认，open 返回 kSdkUnavailable）
│   ├── packet_parser.cpp/.hpp  36 字节头 + 点/IMU 解析
│   ├── frame_assembler.cpp/.hpp frame_cnt 组帧、逐点时间、帧池写入
│   └── config_json.cpp/.hpp    生成 Livox 网络配置 JSON
└── test/livox/
    ├── ..._test.cpp            协议 / 组帧 / mock 生命周期（core，不含 SDK）
    └── backend_test.cpp        后端能力（OFF 断言 kSdkUnavailable）
```

公共头只暴露 `make_lidar(const hal::LidarConfig&)`，不出现任何 SDK 类型。

构建目标：

- `foray_livox_core_objects`：协议、运行时、组帧、JSON，**不链接 SDK**；
- `foray_livox_backend_objects`：`livox_api_real.cpp` 或 `livox_api_stub.cpp`；
- 最终库 = core + backend；mock 测试只链 core，因此任何开关下都不需要 SDK；
- `FORAY_LIVOX_SDK_PROVIDER=SYSTEM|PREBUILT` 决定 `FindLivoxSDK2.cmake` 的搜索范围。

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

帧边界（任一条件）：

- `frame_cnt` 变化（重复扫描的有效帧号）；
- `udp_cnt == 0` 且当前帧非空（有意义的帧起始）；
- 距当前帧首包达到 `frame_duration_ms`（默认 100 ms）。

> **Mid-360 非重复扫描下 `frame_cnt` 无效**，`udp_cnt` 也可能不回零；
> 必须依赖时间窗口切帧，否则会一直累积、永远不发布。

其他约定：

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
- **Livox IMU 原始加速度单位是 g**，驱动乘以标准重力 `9.80665`，
  输出 HAL 约定的 `m/s²`；陀螺保持 `rad/s`；
- 驱动**不会**用主机时间替换设备时间；需要绝对时间时由 PTP/GPS 或上层换算；
- 与 `livox_ros_driver2` 的 NoSync 行为不同，避免静默混用时间域。

## 7. 配置 JSON

`config_json_path` 为空时，驱动按 `host_ip` 生成**两个**设备段：

```json
{
  "MID360": { "lidar_net_info": { "...": 0 }, "host_net_info": [{ "...": "..." }] },
  "Mid360s": { "lidar_net_info": { "...": 0 }, "host_net_info": [{ "...": "..." }] }
}
```

- SDK v1.3.1 用 **`"MID360"`** 表示 Mid-360（dev_type 9），
  用大小写敏感的 **`"Mid360s"`** 表示 Mid-360s（dev_type 35）；
- 缺少对应段时，SDK 检测到设备后会在 `type_lidars_cfg_map_` 查不到类型而直接返回，
  **不会触发 InfoChange**；
- 默认使用**单播**（不写 `multicast_ip`）；需要组播时在自定义 JSON 中加入
  `multicast_ip`，并确保主机加入该组播组、网桥允许转发；
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

两个测试可执行文件，合计 30 个用例：

- `foray_platform_driver_livox_tests`（29 个，只链 core + mock，不需要 SDK/硬件）：
  - `packet_parser_test`：头字段、高/低/球坐标转换、逐点时间、line、IMU、
    time_type 映射、handle 字节序、越界与双回波；
  - `config_json_test`：端口、host_ip、文件权限、默认路径、`MID360` + `Mid360s`；
  - `frame_assembler_test`：变帧发布、时间窗口切帧、空闲超时、NoSync 时间置零、
    溢出、槽位耗尽；
  - `lidar_test`：完整生命周期、点云/IMU（含 g→m/s²）、统计、超时、
    多 handle 路由、SDK handle 与 IP 预绑定不一致时的回退、恢复。
- `foray_platform_driver_livox_backend_tests`（1 个）：
  - OFF 断言公共工厂 `open()` 返回 `kSdkUnavailable`；
  - ON 只验证真实后端能构造，不打开设备。

验证记录：

- 常规构建：30/30 通过（OFF 与 ON）；
- ASan + UBSan：30/30 通过；
- TSan：30/30 通过（mock 的 `uninit()` 模拟 SDK 等待回调归零）；
- `ldd`：core 测试不出现 Livox SDK；backend 测试在 ON 下链接 SDK；
- 安装后下游 `find_package(foray_platform_driver_livox)` 可链接
  `foray_platform::driver_livox`；ON 静态库经 `find_dependency(LivoxSDK2)` 重新定位。

## 11. 已知限制

- 实机已验证：Livox Mid-360s（dev_type 35，SN `ARMCP7D0030195`），
  `192.168.1.195`，100 Mb/s；3 帧点云、IMU、PCD/PPM 输出均正常；
- 双回波 `0x11` 未实现，需要确认固件/SDK 版本与结构；
- `host_ip` 变化需要重新创建运行时（进程内只初始化一次）；
- 未做数据包 CRC 校验（SDK 已校验）；
- 多雷达共用一份 host JSON；如需不同主机 IP/端口，需扩展运行时配置；
- Mid-360s 的 SDK JSON 键是大小写敏感的 `"Mid360s"`，自定义 JSON 必须包含；
- 默认单播；组播需要自定义 JSON 中的 `multicast_ip` 并确保网桥转发。

## 12. 手动测试与点云查看

### 12.1 点云导出工具

构建时打开（需要 SDK ON）：

```bash
cmake -S foray_platform_driver_livox -B ~/.cache/foray-platform-build/livox-on \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HOME/.cache/foray-platform-prefix" \
  -DFORAY_BUILD_DRIVER_LIVOX=ON \
  -DFORAY_LIVOX_SDK_PROVIDER=PREBUILT \
  -DLIVOX_SDK2_ROOT="$HOME/.cache/foray-platform-prefix/livox-sdk2" \
  -DFORAY_LIVOX_BUILD_TOOLS=ON
cmake --build ~/.cache/foray-platform-build/livox-on -j"$(nproc)"
```

运行：

```bash
export LD_LIBRARY_PATH="$HOME/.cache/foray-platform-prefix/livox-sdk2/lib:$LD_LIBRARY_PATH"

~/.cache/foray-platform-build/livox-on/foray_livox_dump \
  --host-ip 192.168.1.51 \
  --lidar-ip 192.168.1.195 \
  --frames 3 \
  --out /tmp/livox_cloud
```

输出：

- `/tmp/livox_cloud.pcd`：`x y z intensity time line`，可用 CloudCompare、
  MeshLab 或 Open3D 打开；
- `/tmp/livox_cloud.ppm`：俯视 XY 投影（按高度着色），直接查看：

```bash
imv /tmp/livox_cloud.ppm
# 或 xdg-open /tmp/livox_cloud.ppm
```

自检（不需要雷达，验证输出格式）：

```bash
~/.cache/foray-platform-build/livox-on/foray_livox_dump \
  --self-test --out /tmp/livox_selftest
imv /tmp/livox_selftest.ppm
```

### 12.2 官方工具（建议先做）

- **Livox Viewer 2**：官方 GUI，直接看实时点云、设备 IP、工作模式；
  如果它也发现不了设备，问题在硬件/网络，不在驱动；
- **livox_ros_driver2 + RViz**：已有 ROS 2 时运行
  `ros2 launch livox_ros_driver2 msg_MID360_launch.py`，在 RViz 添加
  `PointCloud2`；
- **Livox SDK sample**：`livox_lidar_quick_start <config.json>` 打印包统计，
  用于独立验证数据流。

### 12.3 发现/取流失败排查

`start()` 返回 `kTimeout` 时按顺序检查：

1. `ip -brief link show <iface>`：carrier 为 1，速率 100 Mb/s；
2. `ping <lidar-ip>`、`ip neigh`：确认 IP 与 MAC；
3. `ip addr`：确认 `host_ip` 确实在本机；
4. 用 tcpdump 看 56000 广播应答：
   - 若应答的 `dev_type=35`，JSON 必须包含 **`"Mid360s"`**（大小写敏感）；
   - 若应答的 `dev_type=9`，JSON 包含 `"MID360"`；
   - 两者都写入即可兼容；
5. 若日志出现 `Handle detection data` 但驱动仍 `kTimeout`：
   - SDK 的 handle 是 `s_addr`，与 `inet_pton` 结果一致；
   - 驱动会按 IP/serial 回退匹配，并覆盖为 SDK 的 handle；
6. 若 `packets_received > 0` 但 `frames_received == 0`：
   - Mid-360 非重复扫描的 `frame_cnt` 无效；
   - 检查 `frame_duration_ms` 是否大于 0（默认 100 ms）；
7. 组播配置下检查 `ip maddr`/`/proc/net/igmp` 与网桥
   `multicast_snooping`；默认单播不需要这些；
8. 直连网卡（不经网桥）排除中间设备。

### 12.4 实测记录（2026-10-19）

- 设备：**Livox Mid-360s**，`dev_type=35`，SN `ARMCP7D0030195`，
  IP `192.168.1.195`；
- 主机：`br-lidar` `192.168.1.51/24`，100 Mb/s；
- 配置：生成 JSON 同时包含 `MID360` + `Mid360s`，默认单播，
  `frame_duration_ms=100`；
- 结果：2 帧 × 20064 点，`packets_dropped=0`；
  `foray_livox_dump` 输出 PCD/PPM，俯视图可见场景轮廓；
- IMU：`acc_z≈9.05 m/s²`（设备有倾斜），`gyro` 正常；
- 关键修复：Mid-360s 配置键、`s_addr` handle、时间窗口切帧、g→m/s²。
