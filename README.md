# foray_platform

> **L1** · owner 角色：平台

L1 平台基础 + L2 下行 HAL

零业务语义，被所有层依赖，**最稳定**。

---

## 快速开始

> USB CDC 原始字节通道已建立并通过无硬件测试；RoboLink 帧代码依据未冻结候选稿实现，尚未与 MCU 固件联调。

```bash
# 1. 拉取（仓齐备后改为从元仓 foray_ws 一键拉取）
git clone git@github.com:ZQU-Foray/foray_platform.git
cd foray_platform

# 2. 依赖安装
rosdep install -y -r --from-paths . --ignore-src --rosdistro humble

# 3. 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. 运行
# （待装配根就绪后补充）
```

## 依赖

| 类型 | 依赖 |
|---|---|
| **自研仓** | X2 |
| **第三方** | Eigen3 · ROS 2 Humble · 各传感器官方 SDK（Livox / Hikvision 等） |

## 接口

**L1 基础设施**

| 模块 | 内容 |
|---|---|
| 时间与时钟 | 统一时钟源 · `use_sim_time` 行为一致 |
| 数学/几何/滤波 | EKF · PID · 坐标变换 |
| 数据结构 | 无锁队列 · 线程池 |
| 参数系统 | 类型化 schema + 校验 + **热加载** |
| 错误码 | 结果类型 · 生命周期 |
| 日志/录制/回放 | 基础设施（X4 的底座） |
| 状态编码器 | **纯函数**，训练与部署共用 |

**L2 下行 HAL**（依赖倒置：抽象在本仓，具体驱动放本仓之下的插件目录）

| 模块 | 内容 |
|---|---|
| 传感器 | 激光 · IMU · 相机 |
| 执行器 | 底盘 · 云台 · 发射机构 |
| 总线 | 串口 · CAN · USB |
| 设备生命周期 | `open` / `close` / `recover`，`ERROR` 态尽量自修复 |

当前最小落地：`UsbCdcPort` 负责 Linux USB CDC 原始字节读写；候选 RoboLink 实现独立承担 CRC、组帧、字节流解析、零槽/单槽/多槽编解码及协议载荷的字段转换；`RoboLinkByteChannel` 只衔接设备与帧层，不负责业务调度。所有协议字段仍以双方最终确认的 `foray_interfaces` 契约为准；当前实现只供离线测试与 Draft PR 评审，未进行实机控制验收。

`RoboLinkHostLink` 已补上主机侧打开/重试、HELLO 握手、心跳、状态回读、超时断开和控制排队的离线运行逻辑。它没有独立可执行装配程序，也未与 MCU 上线；排队成功不等于电控接受或执行。

依据 `foray_interfaces` 的 RoboLink v0.2 候选契约，主机现可声明并离线解码 `GIMBAL_ATTITUDE`（0x30）：36 B 载荷含采样时刻、云台系到世界系 `w,x,y,z` 四元数、角速度、IMU 状态和有效位。接收缓存同时保留 MCU 帧头时间与 PC 本机接收时间，供后续时间对齐；本层不做跨端时钟换算、坐标变换或时间插值。真实设备联调仍未完成。

**急停硬路径放本仓**：最终安全降级（`cmd_vel` 超时归零、发射机构断电）**不依赖中间件**，
位于最靠近执行器的执行器抽象内部，独立实时线程 + 超时定时器。

## 纪律



- **零业务语义**——不知道该车是哨兵还是步兵

- **不直接到处使用 ROS 2**——否则无法单测、无法去 ROS 化、无法换中间件

- 依赖方向恒为「**实现依赖抽象**」：抽象接口在本仓，具体驱动在本仓之下的插件目录

- 接口稳定；生产构建可关闭可观测实现，但**接口不变**

## 上下游

| 方向 | 对象 |
|---|---|
| **上游**（本仓依赖谁） | `foray_interfaces` |
| **下游**（谁依赖本仓） | `foray_localization` · `foray_vision` · `foray_auto_aim` · `foray_navigation` · `foray_decision` · `foray_robots` · `foray_ws` |

## 参考

- [算法结构](https://github.com/ZQU-Foray/foray_docs/blob/main/algorithm_structure.md)
- [仓库结构](https://github.com/ZQU-Foray/foray_docs/blob/main/repository_structure.md)
- [组织贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)
