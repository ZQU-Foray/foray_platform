# 雷达网络自举与参数注入

本机的雷达链路经常要在比赛现场重配：物理口挂桥、主机地址、NetworkManager
凭据、雷达 IP 与主机端口。手工 `ip link` + `sudo` 既慢又容易忘，且
NetworkManager 会把私下的 slave 关系拆掉。本模块把这段流程收敛成
**只读探测 → 幂等规划 → 授权执行 → 回滚** 四步，供 `foray-net-helper` 与
驱动控制面复用。

## 1. 分层

| 层 | 头文件 | 职责 |
|---|---|---|
| 类型 | `include/foray/platform/net/net_types.hpp` | `Ipv4`、`InterfaceInfo`、`Topology`、只读 `probe_topology()` |
| 进程 | `include/foray/platform/net/process.hpp` | `ProcessRunner` 抽象 + `ExecProcessRunner`（不经过 shell） |
| 后端 | `include/foray/platform/net/backend.hpp` | `NetBackend`：ensure_bridge / set_slave / set_up / add_ipv4 / remove_ipv4 |
| NM | `nm_backend.hpp` | `nmcli` 实现；slave profile 命名 `foray-<iface>-slave` |
| netlink | `netlink_backend.hpp` | rtnetlink 实现，无 NM 时回退 |
| 规划 | `setup.hpp` | `plan_link()` 纯函数 + `ensure_link()` 执行/回滚 + `select_backend()` |

## 2. 幂等与回滚

- `link_ready(topology, iface, bridge, host_ip, min_prefix=16)` 纯函数判定
  「已挂桥 + 有载波 + 主机 IP 在位」，`foray_lidar_up` 据此跳过不必要的授权；
- `plan_link(intent, topology)` 只读拓扑，输出动作序列
  `ensure_bridge → set_slave → set_up → add_ipv4`，已就绪的步骤自动省略；
- `ensure_link(..., dry_run=true)` 只打印计划，不碰后端；
- 任一步失败按逆序回滚已执行的动作（`add_ipv4 → remove_ipv4`、
  `set_slave → clear_slave`），回滚失败记入 `warnings` 而非吞掉；
- 所有后端接口都要求幂等：重复执行不产生副作用、不报错。

## 3. 特权边界

- 不写 setuid 程序；root 权限只来自 polkit/pkexec；
- `foray-net-helper` 安装到 `libexec`，参数只有 iface/bridge/host-ip/prefix，
  名字走白名单（`[A-Za-z0-9_.-]`、≤15 字符），IP 走 `Ipv4::parse`；
- polkit 动作 `org.foray.netconfig.bootstrap` 用 `exec.path` 注解把 pkexec
  限定到该 helper 一个程序；
- 物理口必须是以太网口、不能是网桥本身；无线口只给 warning 不静默配置。

## 4. CLI

```bash
# 只做自举（幂等，可反复跑）
pkexec /usr/libexec/foray_platform/foray-net-helper \
    --iface enp0s31f6 --bridge br-lidar --host-ip 192.168.1.51

# 一键起雷达：链路已就绪则跳过自举，否则自动经 pkexec 调 helper
foray_lidar_up --iface enp0s31f6 --bridge br-lidar --host-ip 192.168.1.51 \
               --lidar-ip 192.168.1.195 --frames 3

# 只 preview 网络计划（不启动驱动、不写系统）
foray_lidar_up --iface enp0s31f6 --bridge br-lidar --host-ip 192.168.1.51 --dry-run

# 自举 + 驱动都失败时退出码分开：5=网络，2/3/4=驱动，6=没读到帧
```

## 5. 与驱动、工具的关系

- `foray_livox_dump` 只负责取流（PCD/PPM/`--stream`），不碰网络；
- `foray_livox_config` 通过 `detail::LivoxApi` 下发雷达参数
  （IP / 安装姿态 / 主机端口 / PPS），写后 `query_internal_info` 读回校验；
- 网络自举失败时工具给出明确退出码，不静默继续。

## 6. 测试

- `test/net/net_types_test.cpp`：IPv4 解析/同网段、`Topology::find`；
- `test/net/probe_test.cpp`：临时 sysfs 树 + `getifaddrs` 真实读取；
- `test/net/setup_test.cpp`：计划顺序、幂等、dry-run、失败回滚；
- `test/net/nm_backend_test.cpp`：`FakeRunner` 断言 `nmcli` 命令行；
- 驱动侧 `test/livox/livox_control_test.cpp`：重试、读回校验、不可用不重试。

## 7. HIL 清单

1. 雷达上电，`ethtool enp0s31f6` 显示 `Link detected: yes`、`Speed: 100Mb/s`；
2. `pkexec ... foray-net-helper --iface enp0s31f6 --bridge br-lidar --host-ip <IP>`；
3. `ip -brief addr show br-lidar` 为 `UP + LOWER_UP`；
4. `foray_livox_dump --host-ip <IP> --lidar-ip <雷达IP> --stream --frames 0` 看到点；
5. 改雷达参数前先 `foray_livox_config --dry-run` 读回，确认 IP 再下发。
