# 驱动部署

> 适用：`foray_platform_driver_livox`（已落地）· `foray_platform_driver_hikvision`（待落地）
> 平台：Linux（x86_64 / aarch64）· CMake ≥ 3.16 · C++17

## 1. Livox SDK2

### 1.1 依赖来源

- **不 fork、不 submodule、不 FetchContent**；
- 版本 **v1.3.1**，commit `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`；
- 在 `foray_ws/.repos` 中 pin 到 `third_party/livox_sdk2`（`src/` 之外）；
- 源码由 vcstool 拉取，不进仓库；构建安装由 `foray_ws/scripts` 完成；
- 只有必须修改上游时才 fork `Livox-SDK2-foray-fork` 并记录 patch。

### 1.2 开发机：拉取与构建

```bash
cd foray_ws
source scripts/env.sh
scripts/import_repos.sh          # 拉取 third_party/livox_sdk2
scripts/build_livox_sdk2.sh      # 构建并安装到本地前缀
```

默认前缀：`~/.cache/foray-platform-prefix/livox-sdk2`。

脚本等价的 CMake 参数（v1.3.1 的兼容处理）：

```bash
cmake -S third_party/livox_sdk2 -B ~/.cache/foray-build/livox-sdk2 \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$HOME/.cache/foray-platform-prefix/livox-sdk2" \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_CXX_FLAGS="-include cstdint"
cmake --build ~/.cache/foray-build/livox-sdk2 -j"$(nproc)"
cmake --install ~/.cache/foray-build/livox-sdk2
```

安装后：

```text
<prefix>/include/livox_lidar_api.h
<prefix>/include/livox_lidar_def.h
<prefix>/lib/liblivox_lidar_sdk_shared.so
```

> SDK2 没有 CMake package config；驱动用随包安装的
> `FindLivoxSDK2.cmake` 暴露 `LivoxSDK2::shared`。

### 1.3 驱动 provider

| provider | 用途 | 搜索方式 |
|---|---|---|
| `SYSTEM`（默认） | 系统安装或 `CMAKE_PREFIX_PATH` | 默认路径 + 前缀，支持 `lib`/`lib64`/multiarch |
| `PREBUILT` | 开发机/实机私有前缀 | 必须有 `LIVOX_SDK2_ROOT`，且头/库必须位于该前缀下 |

```bash
# 开发机真实后端
cmake -S foray_platform_driver_livox -B ~/.cache/foray-platform-build/livox-on \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$HOME/.cache/foray-platform-prefix" \
      -DFORAY_BUILD_DRIVER_LIVOX=ON \
      -DFORAY_LIVOX_SDK_PROVIDER=PREBUILT \
      -DLIVOX_SDK2_ROOT="$HOME/.cache/foray-platform-prefix/livox-sdk2"
cmake --build ~/.cache/foray-platform-build/livox-on -j"$(nproc)"
ctest --test-dir ~/.cache/foray-platform-build/livox-on --output-on-failure
```

- ON 时找不到 SDK：`FATAL_ERROR`，不静默退回 stub；
- 只接受 `liblivox_lidar_sdk_shared.so`；找到静态库直接失败；
- OFF 时仍构建库与工厂，`open()` 返回 `kSdkUnavailable`；
- 安装后的 target 导出 `FORAY_DRIVER_LIVOX_HAS_SDK=0/1`，下游用
  `#if FORAY_DRIVER_LIVOX_HAS_SDK`（不能 `#ifdef`）。

### 1.4 实机部署

- 镜像构建时从 `.repos` pin 的 commit 构建 SDK，安装到
  **`/opt/foray/livox-sdk2`**；
- 镜像只保留安装产物，不保留 SDK 源码/构建目录；
- 运行时库路径二选一：

```bash
# 系统级
echo "/opt/foray/livox-sdk2/lib" | sudo tee /etc/ld.so.conf.d/foray-livox-sdk2.conf
sudo ldconfig

# 会话级
export LD_LIBRARY_PATH=/opt/foray/livox-sdk2/lib:$LD_LIBRARY_PATH
```

构建：

```bash
FORAY_LIVOX_SDK2_PREFIX=/opt/foray/livox-sdk2 source foray_ws/scripts/env.sh
FORAY_LIVOX_SDK=ON foray_ws/scripts/colcon_build.sh
```

或直接传 colcon 参数：

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORAY_BUILD_DRIVER_LIVOX=ON \
    -DFORAY_LIVOX_SDK_PROVIDER=PREBUILT \
    -DLIVOX_SDK2_ROOT=/opt/foray/livox-sdk2
```

验证：

```bash
ldconfig -p | grep livox_lidar_sdk_shared
# 最终库为共享库时：
ldd install/foray_platform_driver_livox/lib/libforay_platform_driver_livox.so | grep livox
# 最终库为静态库时，对实际可执行文件执行 ldd
```

## 2. 网络

### 2.1 地址规划

| 设备 | 示例 |
|---|---|
| 主机（有线网卡 / 网桥） | `192.168.1.51/24` |
| Livox Mid-360 | `192.168.1.1xx`（默认由序列号决定） |
| 子网掩码 | `255.255.255.0` |
| 网关 | 不需要 |

### 2.2 网桥（本机当前约定）

本机使用 `br-lidar` 承载雷达网段：

```bash
# 将以太网口挂到网桥（需要 root）
sudo ip link set enp0s31f6 master br-lidar
sudo ip link set br-lidar up
ip -brief addr show br-lidar
```

- Mid-360 为 100BASE-TX，`ethtool` 应显示 `Speed: 100Mb/s`；
- 确认 `host_ip` 与雷达同网段，且确实配置在本机网卡上；
- 放通 UDP 56100–56500（本驱动默认端口段）；
- 多雷达时保持同一 host 配置，SDK 按 handle（雷达 IP）路由。

### 2.3 发现与验证

```bash
ping 192.168.1.255          # 广播探测
ip neigh show dev br-lidar  # 查看 ARP
```

驱动日志/统计中 `packets_received` 持续增长即说明数据到达。

## 3. 时间同步

### 3.1 首期：PPS

- Mid-360 M12 连接器 **pin 8** 接受 3.3 V TTL 1PPS；
- 1 Hz，高电平 20–200 ms；
- 与相机触发共用同一晶振/定时器分频，避免累积漂移；
- 必须共地。

`time_type` 为 3/4 时 HAL 标记 `kPps`，**不保证 Unix 纪元**，
`base_time` 保持为零；需要绝对时间时由同步板提供纪元或改用 PTP。

### 3.2 PTP

```bash
sudo ptp4l -i br-lidar -l 6 -m
sudo phc2sys -c br-lidar -s CLOCK_REALTIME -O 0
ethtool -T br-lidar        # 确认硬件时间戳能力
```

PTP 同步后 `time_type=1`，HAL 标记 `kPtp`，`base_time` 可直接作为绝对时间。

### 3.3 不做的事

- 驱动不调用主机时钟替换设备时间；
- NoSync 时不用主机时间伪造 `base_time`；
- 不在 HAL 内做跨传感器时间对齐。

## 4. Hikvision MVS（待落地）

M7/M8 尚未实现。部署时预计需要：

- MVS SDK 安装在 `/opt/MVS`，头文件与 `lib/aarch64`/`lib/64`；
- udev 规则由 MVS 安装包提供；
- `FORAY_MVS_ROOT` 指向 SDK 根，`FORAY_BUILD_DRIVER_HIKVISION=ON`；
- GigE 需巨帧（MTU ≥ 9014）与 `GevSCPSPacketSize`/`GevSCPD` 配置；
- USB3 硬件触发接 6-pin P7：pin 2 `OPTO_IN+`、pin 5 `OPTO_GND`（5–24 V）。

## 5. 部署检查清单

- [ ] Livox SDK2 v1.3.1 已安装，`ldd` 能找到 `liblivox_lidar_sdk_shared.so`
- [ ] 主机网卡与雷达同网段，`host_ip` 在本机
- [ ] UDP 56100–56500 未被防火墙拦截
- [ ] PPS 电平/极性正确，共地
- [ ] 驱动 `ctest` 全绿（无 SDK 环境用 stub）
- [ ] 30 分钟长跑：RSS/FD/句柄平稳，`packets_dropped` 可解释
