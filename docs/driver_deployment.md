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
  `#if FORAY_DRIVER_LIVOX_HAS_SDK`（不能 `#ifdef`）；
- 静态驱动库的下游链接时需在自身构建中设置 `LIVOX_SDK2_ROOT`，或把 SDK
  前缀加入 `CMAKE_PREFIX_PATH`；包 Config 会用 `find_dependency(LivoxSDK2)` 重新定位。

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
| Livox Mid-360 / Mid-360s | `192.168.1.1xx`（默认由序列号决定） |
| 子网掩码 | `255.255.255.0` |
| 网关 | 不需要 |

### 2.2 网桥（推荐用 `foray-net-helper` 一键自举）

本机使用 `br-lidar` 承载雷达网段。不要手工 `ip link`——NetworkManager 会在几秒内
把私下的 slave 关系拆掉；正确做法是让 helper 通过 NM 创建 `foray-<iface>-slave`
profile（掉电重启也保留）：

```bash
# 一次性安装 polkit 动作后，普通用户即可授权执行（见 §2.2.1）
pkexec /usr/libexec/foray_platform/foray-net-helper \
    --iface enp0s31f6 --bridge br-lidar --host-ip 192.168.1.51
ip -brief addr show br-lidar      # 期望 UP + LOWER_UP + 192.168.1.51/24
```

helper 输出 `applied:` 行说明实际改了什么，`warning:` 行给出「没有载波」等提示；
失败会自动回滚已做的步骤，退出码：0 成功 / 1 用法 / 2 权限 / 3 配置失败 / 4 环境不支持。

#### 2.2.1 polkit 授权

安装（`cmake --install` 已包含）会放置：

| 文件 | 作用 |
|---|---|
| `/usr/share/polkit-1/actions/org.foray.netconfig.policy` | 只放行 helper 一个程序 |
| `/usr/share/polkit-1/rules.d/60-foray-netconfig.rules` | 可选：`foray` 组成员免密 |

把操作手加入 `foray` 组即可无人值守；不加组时每次弹一次图形授权。

#### 2.2.2 没有 NetworkManager 的环境

`select_backend()` 会自动回退到 rtnetlink（`NetBackend` 的另一种实现），
helper 仍可工作；此时 `applied:` 走内核 netlink，不会创建 NM profile。

- Mid-360 为 100BASE-TX，`ethtool` 应显示 `Speed: 100Mb/s`；
- 确认 `host_ip` 与雷达同网段，且确实配置在本机网卡上；
- 放通 UDP 56100–56500（本驱动默认端口段）；
- 多雷达时保持同一 host 配置，SDK 按 handle（雷达 IP）路由。

### 2.3 设备类型与数据链路

- SDK JSON 必须包含对应设备段：
  - **`"MID360"`** → Mid-360（dev_type 9）；
  - **`"Mid360s"`** → Mid-360s（dev_type 35，大小写敏感）；
  - 驱动自动生成时会同时写入两段。
- 默认使用**单播**：雷达发到 `host_ip:56301`（点云）与 `host_ip:56401`（IMU）；
  不需要 IGMP 或网桥组播配置。
- 如使用自定义 JSON 的 `multicast_ip`（组播）：

  ```bash
  echo 0 | sudo tee /sys/class/net/br-lidar/bridge/multicast_snooping
  ip maddr show br-lidar | grep 224.1.1.5
  ```

- 非重复扫描下 `frame_cnt` 无效；驱动用 `frame_duration_ms`（默认 100 ms）
  切帧。需要 10 Hz 时保持 100，5 Hz 可设 200。

### 2.4 一键起雷达（编排）

网络与驱动都有各自的工具，实战时用一条命令串起来：

```bash
foray_lidar_up --iface enp0s31f6 --bridge br-lidar --host-ip 192.168.1.51 \
               --lidar-ip 192.168.1.195 --frames 3
```

- 链路未就绪时自动经 `pkexec` 调 `foray-net-helper`（不要求操作手记命令）；
- 就绪后创建驱动、打开、`start()`，读 `--frames` 帧并打印
  `seq/points/dropped/sync` 与统计；
- 加 `--dry-run` 只经 helper 打印 `planned:` 计划并跳过驱动，适合现场先看会改什么；
- 退出码：0 成功 / 1 用法 / 2 驱动创建 / 3 open / 4 start / 5 网络自举 / 6 没读到帧；
- 要点云落盘或流式输出，继续用 `foray_livox_dump`。

### 2.5 发现与验证

```bash
ping 192.168.1.255          # 广播探测
ip neigh show dev br-lidar  # 查看 ARP
```

驱动日志/统计中 `packets_received` 持续增长即说明数据到达；
`frames_received` 增长说明时间切帧正常。

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
- [ ] JSON 包含设备类型段：`MID360`（9）或 `Mid360s`（35）
- [ ] `frame_duration_ms` 与目标帧率匹配（10 Hz = 100）
- [ ] PPS 电平/极性正确，共地
- [ ] 驱动 `ctest` 全绿（无 SDK 环境用 stub）
- [ ] 30 分钟长跑：RSS/FD/句柄平稳，`packets_dropped` 可解释
