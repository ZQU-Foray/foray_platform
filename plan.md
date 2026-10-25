# 开发计划

## 任务列表

- [ ] 定义时间与时钟抽象（含 `use_sim_time` 语义）
- [ ] 搭参数系统骨架（schema + 校验 + 热加载）
- [ ] 定义 HAL 抽象接口（传感器 / 执行器 / 总线 / 设备生命周期）
- [x] 建立 USB CDC 设备打开/关闭的最小构建和无硬件测试骨架
- [x] 验证 USB CDC 二进制字节收发与 CRC-32/ISO-HDLC 的离线测试向量
- [x] 候选 RoboLink 零槽保活帧离线组帧与独立黄金字节测试（不代表电控端已支持）
- [x] 候选 RoboLink 字节流解析器：半帧、连帧、垃圾前缀、坏头及 CRC 失败恢复的离线测试
- [x] 候选 RoboLink 零槽、单槽与多槽帧及七类消息载荷的编解码/校验，完成 USB 字节通道离线串接
- [x] 按已合入 `foray_interfaces` 的 RoboLink v0.2 候选稿更新云台角 rad 与姿态反馈的离线测试向量
- [x] 离线实现主机握手、心跳、状态回读、500 ms 超时及设备重开状态机
- [ ] 验证发送调度的频率/延迟，并与 MCU 固件进行只读/禁能联调
- [ ] 完成 v0.2 与电控固件逐字节对账及失联动作验收；在此之前不发送控制指令到硬件
- [ ] 与自瞄组补齐云台系目标到世界系目标角的转换和图像—IMU 时间对齐，再开放真实云台控制
- [ ] 完成 USB CDC 设备读写、错误处理与设备参数配置
- [ ] 接一个真实驱动跑通「一个传感器 → 一个话题」
- [ ] 实现急停硬路径（独立实时线程 + 超时定时器）

## 当前 Agent State

HOST_LINK_V02_OFFLINE_TESTED：按已合入的 v0.2 候选协议补齐 0x30 接收及能力位；工作区尚无装配根，未接入真实电控。

## Before Snapshot

commit hash:   d113af5
branch:        feat/usb-cdc
modified files: README.md、decision.md、plan.md、tree.md，以及未跟踪的构建文件、头文件、源码和测试
risk level:    L1

## 模糊点与待确认项

- 本仓接口尚未冻结，任务清单为**方向性**的，落地顺序以 `foray_docs/algorithm_structure.md` §9 演进阶段为准
- 依赖的自研仓尚未建齐，跨仓任务需等对方 `README.md` 明确接口后再启动

## Vector Backend Status

Backend: Markdown
Status:  ready
Environment: 仓库内 Markdown 文档（无外部向量后端）
Index: 本仓 `README.md` / `tree.md` / `decision.md`
Initialization: 2026-02-19
Commit: （首次提交）

## Acceptance Criteria

- [ ] 本仓能独立 `colcon build`（无代码时跳过）
- [ ] CI 绿灯
- [ ] 每个新增模块带独立可执行测试
