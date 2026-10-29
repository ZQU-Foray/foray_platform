# 设备网络自举与参数注入

本机的雷达与 GigE 相机链路经常要在比赛现场重配：物理口挂桥、主机地址、
NetworkManager 凭据、巨帧 MTU、设备 IP 与主机端口。手工 `ip link` + `sudo`
既慢又容易忘，且 NetworkManager 会把私下的 slave 关系拆掉。本模块把这段流程
收敛成 **只读探测 → 幂等规划 → 授权执行 → 回滚** 四步，供
`foray-net-helper`、`foray_lidar_up` 与 `foray_hikvision_up` 复用。

## 1. 分层

| 层 | 头文件 | 职责 |
|---|---|---|
| 类型 | `include/foray/platform/net/net_types.hpp` | `Ipv4`、`InterfaceInfo`（含 `mtu`）、`Topology`、只读 `probe_topology()` |
| 进程 | `include/foray/platform/net/process.hpp` | `ProcessRunner` 抽象 + `ExecProcessRunner`（不经过 shell） |
| 后端 | `include/foray/platform/net/backend.hpp` | `NetBackend`：ensure_bridge / set_slave / set_up / set_mtu / add_ipv4 / remove_ipv4 |
| NM | `nm_backend.hpp` | `nmcli` 实现；slave profile 命名 `foray-<iface>-slave` |
| netlink | `netlink_backend.hpp` | rtnetlink 实现，无 NM 时回退 |
| 规划 | `setup.hpp` | `plan_link()` 纯函数 + `ensure_link()` 执行/回滚 + `select_backend()` |
| 编排 | `bootstrap.hpp` | `bootstrap_link()`：探拓扑 → 判就绪 → 非 root 走 pkexec → 调 helper |

`probe_topology()` 只在 sysfs 上读，不调用 `nmcli`，因此
`Topology::network_manager_present` 默认是 `false`，由 `select_backend()`
探测后回填；只调探测的消费方不要拿这个标志判断「有没有 NM」。

## 2. 幂等与回滚

- `link_ready(topology, iface, bridge, host_ip=nullopt, min_prefix=16,
  min_mtu=nullopt)` 纯函数判定「已挂桥 + master up + 有载波 + MTU 达标 +
  主机 IP 在位」；`host_ip` 留空只校验链路，`foray_hikvision_up` 在地址由
  `plan_host_address()` 推导前先用它判断是否需要授权；
- `plan_link(intent, topology)` 只读拓扑，输出动作序列
  `ensure_bridge → set_slave → set_up → set_mtu(物理口) → set_mtu(网桥)
  → remove_ipv4 → add_ipv4`，已就绪的步骤自动省略；物理口先于网桥，
  因为 Linux 要求 slave 的 MTU 不小于 master；
- 物理口已属于另一个网桥时 `plan_link` 返回 `kInvalidArgument`，错误信息给出
  两个桥名与迁移提示；只有 `NetIntent::allow_reassign_master = true`
  （CLI `--allow-reassign-master`）才放行，避免把雷达口从 `br-lidar` 静默抢走；
- `ensure_link(..., dry_run=true)` 只打印计划，不碰后端；
- MTU 设置失败只记 `warnings`（期望值、实际值、接口名）并继续，不进回滚：
  1500 下 GigE 720×540@125 Hz 实测可跑，9014 是推荐值；
- 其余任一步失败按逆序回滚已执行的动作（`add_ipv4 → remove_ipv4`、
  `set_slave → clear_slave`），回滚失败记入 `warnings` 而非吞掉；
- 所有后端接口都要求幂等：重复执行不产生副作用、不报错。

## 3. 特权边界

- 不写 setuid 程序；root 权限只来自 polkit/pkexec；
- `foray-net-helper` 安装到 `libexec`，参数只有
  iface/bridge/host-ip/prefix/mtu/allow-reassign-master，名字走白名单
  （`[A-Za-z0-9_.-]`、≤15 字符），IP 走 `Ipv4::parse`，MTU 限制 576–65535
  （576 = RFC 791 最小重组缓冲区，65535 = 16 位上限）；
- polkit 动作 `org.foray.netconfig.bootstrap` 用 `exec.path` 注解把 pkexec
  限定到该 helper 一个程序；
- 物理口必须是以太网口、不能是网桥本身；无线口只给 warning 不静默配置。

## 4. CLI

```bash
# 只做自举（幂等，可反复跑）
pkexec /usr/libexec/foray_platform_core/foray-net-helper \
    --iface enp0s31f6 --bridge br-lidar --host-ip 192.168.1.51

# 雷达：链路已就绪则跳过自举，否则自动经 pkexec 调 helper
foray_lidar_up --iface enp0s31f6 --bridge br-lidar --host-ip 192.168.1.51 \
               --lidar-ip 192.168.1.195 --frames 3

# 相机：自动规划主机地址（相机是 LLA 就用 169.254/16）、挂 br-camera、设巨帧
foray_hikvision_up --iface enp0s13f0u1 --bridge br-camera --frames 3

# 相机：固定地址 + 显式 MTU + 允许把口从旧桥迁过来
foray_hikvision_up --iface enp0s13f0u1 --host-ip 169.254.114.1 --mtu 9014 \
                   --allow-reassign-master --serial <SN>

# 只 preview 网络计划（不启动驱动、不写系统）
foray_lidar_up --iface enp0s31f6 --bridge br-lidar --host-ip 192.168.1.51 --dry-run
foray_hikvision_up --iface enp0s13f0u1 --dry-run

# 退出码：lidar_up 5=网络，2/3/4=驱动，6=没读到帧；
#         hikvision_up 3=环境/网络，4=没挑到相机，5=打开失败，6=没取到帧
```

## 5. 与驱动、工具的关系

- `foray_livox_dump` 只负责取流（PCD/PPM/`--stream`），不碰网络；
- `foray_livox_config` 通过 `detail::LivoxApi` 下发雷达参数
  （IP / 安装姿态 / 主机端口 / PPS），写后 `query_internal_info` 读回校验；
- `foray_hikvision_up` 在 GigE 下先自举 `br-camera`、再枚举/选设备/取帧；
  相机保持链路本地地址（LLA）时按 RFC 3927 规划 `169.254.x/16`；
- `foray_hikvision_up --transport usb3` 目前只枚举设备并打印
  `TODO: USB3 协议自举尚未完善`，USB3 Vision 控制通道留到后续单元；
- 网络自举失败时工具给出明确退出码，不静默继续。

## 6. 测试

- `test/net/net_types_test.cpp`：IPv4 解析/同网段、`Topology::find`；
- `test/net/probe_test.cpp`：临时 sysfs 树 + `getifaddrs` 真实读取，含 MTU
  缺失/非法回退 0；
- `test/net/setup_test.cpp`：计划顺序、幂等、dry-run、失败回滚、MTU 动作与
  软失败、跨桥拒绝/放行、`link_ready` 的 MTU 与空地址分支；
- `test/net/nm_backend_test.cpp`：`FakeRunner` 断言 `nmcli` 命令行（含
  `802-3-ethernet.mtu`）；
- `test/net/bootstrap_test.cpp`：就绪跳过、helper argv 顺序、非 0 退出码透传、
  dry-run 与 `--allow-reassign-master`；
- 驱动侧 `test/livox/livox_control_test.cpp`：重试、读回校验、不可用不重试；
- 驱动侧 `test/hikvision/camera_bootstrap_test.cpp`：LLA/同网段地址规划。

## 7. HIL 清单

1. 雷达上电，`ethtool enp0s31f6` 显示 `Link detected: yes`、`Speed: 100Mb/s`；
2. `pkexec ... foray-net-helper --iface enp0s31f6 --bridge br-lidar --host-ip <IP>`；
3. `ip -brief addr show br-lidar` 为 `UP + LOWER_UP`；
4. `foray_livox_dump --host-ip <IP> --lidar-ip <雷达IP> --stream --frames 0` 看到点；
5. 改雷达参数前先 `foray_livox_config --dry-run` 读回，确认 IP 再下发；
6. 相机上电，`ethtool enp0s13f0u1` 显示 `Link detected: yes`、`Speed: 1000Mb/s`；
7. `foray_hikvision_up --iface enp0s13f0u1 --frames 3` 无需手工配网即出帧；
8. `ip -brief addr show br-camera` 为 `UP + LOWER_UP`、MTU 9014、地址在
   `169.254/16` 或相机同网段；`ip -brief link show enp0s13f0u1` 的 master 为
   `br-camera`。
