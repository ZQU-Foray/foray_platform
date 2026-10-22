# RoboLink 候选线协议对账（Draft）

本次只提交 PC 侧的 CRC、帧格式、字节流解析及消息编解码和离线测试，供电控逐字段对账；不含 USB 驱动、握手运行状态机或真实执行器控制。候选协议尚未冻结，测试通过不代表 MCU 已接入。

## 需要双方确认

1. 云台目标 `gimbal_yaw` / `gimbal_pitch`：组长已确认线上单位为 **deg**，由视觉侧在协议边界转换；候选文档 §4.7 标为 rad，需统一。yaw 向左为正、pitch 向下为正。
2. `CONTROL_SNAPSHOT` 的 48 字节字段位置、`flags` 位义、消息类型与槽位标志。当前视觉仅计划设置云台使能位，不请求底盘或发射。
3. HELLO 是否由双方主动发送；若 MCU 主动发 HELLO，PC 应如何回复。此 PR 只实现载荷和帧编解码，不实现建链行为。
4. `SYSTEM_STATUS` 的接收、接受、执行控制序号含义及回报时机；USB 写出不能作为执行证明。

离线验证：`colcon build --packages-select foray_platform` 与 `colcon test --packages-select foray_platform`；本次精简范围内 42 项测试通过。请电控对照自己的独立测试向量，而非只依据 PC 侧编码后再解码的往返测试。
