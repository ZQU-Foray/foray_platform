#ifndef FORAY_PLATFORM_ROBOLINK_CODEC_HPP
#define FORAY_PLATFORM_ROBOLINK_CODEC_HPP

#include <cstddef>
#include <cstdint>
#include <array>
#include <optional>
#include <vector>

namespace foray_platform {

// 候选槽位一条消息的编号、序号与原始载荷
struct RoboLinkSlot {
    std::uint8_t type = 0;
    std::uint8_t flags = 0;
    std::uint16_t sequence = 0;
    std::vector<std::uint8_t> payload;
};

// 视觉联调所需的控制快照  字段顺序和长度来自 组织候选协议
// 云台目标角按 deg  其余角速度 rad/s
struct ControlSnapshot {
    std::uint32_t control_seq = 0;   // 指令编号
    std::uint8_t source = 2;        // 命令来源
    std::uint8_t mode = 2;          // 使用模式 (期望使用模式)
    std::uint16_t flags = 0;        // 哪些控制部分有效
    float chassis_vx = 0;
    float chassis_vy = 0;
    float chassis_wz = 0;
    // 希望云台达到的目标角度  (不是当前的反馈角度！！)
    float gimbal_yaw_deg = 0;
    float gimbal_pitch_deg = 0;

    float gimbal_yaw_rate = 0;
    float gimbal_pitch_rate = 0;
    float friction_rads = 0;
    float feeder_rads = 0;
    std::uint16_t validity_hint_ms = 0;
};

struct SystemDiagnostics {
    std::uint32_t rx_frames = 0;
    std::uint32_t tx_frames = 0;
    std::uint32_t crc_errors = 0;
    std::uint16_t rx_overflow = 0;
    std::uint16_t tx_busy_drop = 0;
    std::uint16_t queue_drop = 0;
    std::uint8_t event_queue_depth = 0;
};

// 电控回传状态 核心 28 B，诊断尾部可选 24 B
struct SystemStatus {
    std::uint32_t uptime_ms = 0;
    std::uint8_t mode = 0;
    std::uint8_t control_source = 0;
    std::uint8_t safety_state = 0;
    std::uint32_t last_rx_control_seq = 0;
    std::uint32_t last_accepted_control_seq = 0;
    std::uint32_t last_applied_control_seq = 0;
    std::uint16_t control_msg_age_ms = 0;
    std::uint8_t link_state = 0;
    std::uint32_t fault_flags = 0;
    std::optional<SystemDiagnostics> diagnostics;
};

// 双端建链消息    字段布局来自候选协议
struct Hello {
    std::uint8_t proto_ver_major = 2;
    std::uint8_t proto_ver_minor = 0;
    std::uint8_t device_type = 1;
    std::array<std::uint8_t, 3> fw_version{};
    std::uint32_t git_short = 0;
    std::uint64_t capabilities = 0;
    std::uint16_t heartbeat_period_ms = 20;
    std::uint16_t max_frame_bytes = 512;
    std::uint32_t device_id_digest = 0;
};

struct HelloAck {
    std::uint16_t acked_msg_seq = 0;
    std::uint8_t negotiated_major = 2;
    std::uint8_t status = 0;
    std::uint16_t heartbeat_period_ms = 20;
};

struct Ack {
    std::uint8_t acked_msg_type = 0;
    std::uint16_t acked_msg_seq = 0;
    std::uint8_t ack_state = 0;
    std::uint16_t error_code = 0;
};

struct ErrorMessage {
    std::uint16_t error_code = 0;
    std::vector<std::uint8_t> detail;
};

// 编解码完整帧 可包含多个槽位 空槽位集合表示零槽保活帧
std::vector<std::uint8_t> encode_frame(
    std::uint16_t frame_seq, std::uint32_t timestamp_us,
    const std::vector<RoboLinkSlot>& slots);
std::optional<std::vector<RoboLinkSlot>> decode_frame(
    const std::vector<std::uint8_t>& frame);

// 单槽位便捷入口；参数非法时抛异常
std::vector<std::uint8_t> encode_single_slot_frame(
    std::uint16_t frame_seq, std::uint32_t timestamp_us,
    const RoboLinkSlot& slot);

// 输入应为帧解析器给出的完整帧；本函数仍独立复核长度与 CRC
std::optional<RoboLinkSlot> decode_single_slot_frame(
    const std::vector<std::uint8_t>& frame);

// 只负责 48 B 载荷，不直接打开 USB，也不读取自瞄模块的 C++ 结构体。
std::vector<std::uint8_t> encode_control_snapshot_payload(
    const ControlSnapshot& snapshot);

// 只接受候选协议的 28 B 核心或 52 B（核心 + 诊断）载荷。
std::optional<SystemStatus> decode_system_status_payload(
    const std::uint8_t* data, std::size_t size);

// 握手、确认、错误与心跳载荷只处理字段，不处理发送周期和重连状态机。
std::vector<std::uint8_t> encode_hello_payload(const Hello& message);
std::optional<Hello> decode_hello_payload(const std::uint8_t* data, std::size_t size);
std::vector<std::uint8_t> encode_hello_ack_payload(const HelloAck& message);
std::optional<HelloAck> decode_hello_ack_payload(const std::uint8_t* data, std::size_t size);
std::vector<std::uint8_t> encode_ack_payload(const Ack& message);
std::optional<Ack> decode_ack_payload(const std::uint8_t* data, std::size_t size);
std::vector<std::uint8_t> encode_error_payload(const ErrorMessage& message);
std::optional<ErrorMessage> decode_error_payload(const std::uint8_t* data, std::size_t size);

// 视觉侧常用的一发一收入口，避免调用方自己拼消息类型和槽位标志。
std::vector<std::uint8_t> make_control_snapshot_frame(
    std::uint16_t frame_seq, std::uint32_t timestamp_us,
    std::uint16_t message_seq, const ControlSnapshot& snapshot);
std::optional<SystemStatus> decode_system_status_frame(
    const std::vector<std::uint8_t>& frame);

}  // namespace foray_platform

#endif
