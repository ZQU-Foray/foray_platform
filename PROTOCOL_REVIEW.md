# RoboLink 候选线协议对账（Draft）

本次只提交 PC 侧的 CRC、帧格式、字节流解析及消息编解码和离线测试，供电控逐字段对账；不含 USB 驱动、握手运行状态机或真实执行器控制。候选协议尚未冻结，测试通过不代表 MCU 已接入。

## 需要双方确认

1. 云台目标 `gimbal_yaw` / `gimbal_pitch`：按最新对接口径，线上单位改为 **rad**，与当前算法输出及候选文档 §4.7 一致；旧的 deg 口径已失效。yaw 向左为正、pitch 向下为正。
2. `CONTROL_SNAPSHOT` 的 48 字节字段位置、`flags` 位义、消息类型与槽位标志。当前视觉仅计划设置云台使能位，不请求底盘或发射。
3. HELLO 是否由双方主动发送；若 MCU 主动发 HELLO，PC 应如何回复。此 PR 只实现载荷和帧编解码，不实现建链行为。
4. `SYSTEM_STATUS` 的接收、接受、执行控制序号含义及回报时机；USB 写出不能作为执行证明。
5. `foray_interfaces` 已合入 RoboLink v0.2 候选稿：云台姿态独立使用 `GIMBAL_ATTITUDE`（0x30、36 B），四元数 `w,x,y,z` 表示云台系到世界系，另有采样时刻与有效位。本地已实现离线解码；仍需与电控固件逐字节对账和实机验收，不能把四元数塞入 `SYSTEM_STATUS` 保留字节。
6. **控制前阻断项**：v0.2 的 `CONTROL_SNAPSHOT.gimbal_yaw/pitch` 是世界系目标角；当前自瞄 `aim_at()` 从云台系目标向量直接算出的角度不可未经当前姿态转换就原样下发。完成相机/云台外参与图像—IMU 时间对齐、世界系目标角转换和禁能联调前，不启用真实云台控制。

离线验证：`colcon build --packages-select foray_platform` 与 `colcon test --packages-select foray_platform`；测试数量和结果以本地最新 `colcon test-result` 为准。请电控对照自己的独立测试向量，而非只依据 PC 侧编码后再解码的往返测试。
