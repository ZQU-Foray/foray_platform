# 驱动部署

> 适用：`foray_platform_driver_livox`（已落地）· `foray_platform_driver_hikvision`（待落地）
> 平台：Linux（x86_64 / aarch64）· CMake ≥ 3.16 · C++17

## 1. Livox SDK2

### 1.1 版本与获取

- 版本：**v1.3.1**（与调研一致）；
- 许可：MIT；
- 不提交 SDK 源码/二进制到本仓；由部署环境或 `foray_ws/.repos` pin 版本。

```bash
git clone --depth 1 --branch v1.3.1 \
    https://github.com/Livox-SDK/Livox-SDK2.git
```

### 1.2 构建（Arch / 新 CMake 注意事项）

Livox-SDK2 v1.3.1 的 CMake 最低版本声明较旧，新 CMake 需要放宽策略；
新 GCC 下 `file_manager.h` 缺少 `<cstdint>`，通过编译参数注入：

```bash
cmake -S Livox-SDK2 -B Livox-SDK2/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_CXX_FLAGS="-include cstdint" \
      -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build Livox-SDK2/build -j"$(nproc)"
cmake --install Livox-SDK2/build
```

安装后应存在：

```text
/usr/local/include/livox_lidar_api.h
/usr/local/include/livox_lidar_def.h
/usr/local/lib/liblivox_lidar_sdk_shared.so
```

> SDK2 没有 CMake package config，驱动用 `find_path` + `find_library` 定位。

### 1.3 构建驱动

```bash
cmake -S foray_platform_driver_livox -B ~/.cache/foray-platform-build/livox \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$HOME/.cache/foray-platform-prefix" \
      -DFORAY_BUILD_DRIVER_LIVOX=ON \
      -DLIVOX_SDK2_ROOT=/usr/local
cmake --build ~/.cache/foray-platform-build/livox -j"$(nproc)"
ctest --test-dir ~/.cache/foray-platform-build/livox --output-on-failure
```

- `LIVOX_SDK2_ROOT` 可省略，默认搜索 `/usr/local` 与 `/usr`；
- ON 时找不到头/库会 `FATAL_ERROR`，不会静默退回 stub；
- OFF 时仍构建库与工厂，`open()` 返回 `kSdkUnavailable`。

### 1.4 运行时库路径

SDK 不在系统库路径时：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

长期部署建议用 rpath 或 `ldconfig` 配置。

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
