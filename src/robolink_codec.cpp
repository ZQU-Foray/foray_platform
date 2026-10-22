#include "foray_platform/robolink_codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "foray_platform/robolink_crc.hpp"

namespace foray_platform {
namespace {

// 帧头和各载荷的偏移、长度来自组织的 foray_interfaces 候选协议
constexpr std::size_t kHeaderSize = 14;
constexpr std::size_t kSlotHeaderSize = 6;
constexpr std::size_t kCrcSize = 4;
constexpr std::size_t kMaxFrameSize = 512;
constexpr std::size_t kMaxPayloadSize = 486;
constexpr std::size_t kControlPayloadSize = 48;
constexpr std::size_t kStatusCoreSize = 28;
constexpr std::size_t kStatusFullSize = 52;
constexpr std::size_t kHelloSize = 26;
constexpr std::size_t kHelloAckSize = 8;
constexpr std::size_t kAckSize = 7;
constexpr std::size_t kMaxErrorDetailSize = 64;

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "RoboLink 需要 IEEE 754 单精度浮点数");

void append_u16(std::vector<std::uint8_t>& out, std::uint16_t value)
{
    out.push_back(static_cast<std::uint8_t>(value));
    out.push_back(static_cast<std::uint8_t>(value >> 8));
}
// 每次移动到最右  小端序
void append_u32(std::vector<std::uint8_t>& out, std::uint32_t value)
{
    for (unsigned shift = 0; shift < 32; shift += 8) {
        out.push_back(static_cast<std::uint8_t>(value >> shift));
    }
}

void append_u64(std::vector<std::uint8_t>& out, std::uint64_t value)
{
    for (unsigned shift = 0; shift < 64; shift += 8) {
        out.push_back(static_cast<std::uint8_t>(value >> shift));
    }
}

void append_f32(std::vector<std::uint8_t>& out, float value)
{
    if (!std::isfinite(value)) { // 判断是否有效
        throw std::invalid_argument("控制快照含非有限浮点数");
    }
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits)); // 复制float原始位置
    append_u32(out, bits);   // 小端序 写入
}

std::uint16_t read_u16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>(p[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[1]) << 8);
}

std::uint32_t read_u32(const std::uint8_t* p)
{
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}

std::uint64_t read_u64(const std::uint8_t* p)
{
    std::uint64_t value = 0;
    for (unsigned shift = 0; shift < 64; shift += 8) {
        value |= static_cast<std::uint64_t>(p[shift / 8]) << shift;
    }
    return value;
}

bool payload_size_matches(std::uint8_t type, std::size_t size)
{
    switch (type) {
    case 0x01: return size == kHelloSize;
    case 0x02: return size == kHelloAckSize;
    case 0x03: return size == 0;
    case 0x04: return size == kAckSize;
    case 0x05: return size >= 2 && size <= 2 + kMaxErrorDetailSize;
    case 0x10: return size == kStatusCoreSize || size == kStatusFullSize;
    case 0x20: return size == kControlPayloadSize;
    default: return false;
    }
}

bool class_flags_match(std::uint8_t type, std::uint8_t flags)
{
    // 候选协议   按消息类检查槽位标志，避免双方把事件误当状态
    switch (type) {
    case 0x01: return flags == 0x04;  // HELLO：事件
    case 0x02: return flags == 0x0C;  // HELLO_ACK：事件响应
    case 0x03: return flags == 0x00;  // HEARTBEAT：状态
    case 0x04: return flags == 0x08;  // ACK：状态响应
    case 0x05: return flags == 0x04;  // ERROR：事件
    case 0x10: return flags == 0x00;  // SYSTEM_STATUS：状态
    case 0x20: return flags == 0x02;  // CONTROL_SNAPSHOT：最新值
    default: return false;
    }
}

}  // 匿名命名空间

std::vector<std::uint8_t> encode_frame(
    std::uint16_t frame_seq, std::uint32_t timestamp_us,
    const std::vector<RoboLinkSlot>& slots)
{
    if (slots.size() > 255) {
        throw std::invalid_argument("槽位数量超过帧头可表示范围");
    }
    std::size_t frame_size = kHeaderSize + kCrcSize;
    bool has_ack = false;
    for (const auto& slot : slots) {
        if (slot.payload.size() > kMaxPayloadSize ||
            (slot.flags & static_cast<std::uint8_t>(~0x0Fu)) != 0 ||
            !payload_size_matches(slot.type, slot.payload.size()) ||
            !class_flags_match(slot.type, slot.flags)) {
            throw std::invalid_argument("槽位长度或标志非法");
        }
        frame_size += kSlotHeaderSize + slot.payload.size();
        has_ack |= slot.type == 0x04;
    }
    if (frame_size > kMaxFrameSize) {
        throw std::invalid_argument("整帧超过 512 字节");
    }

    std::vector<std::uint8_t> out;
    out.reserve(frame_size);
    out.push_back(0xD3);
    out.push_back(0x91);
    out.push_back(0x02);  // 协议主版本。
    out.push_back(static_cast<std::uint8_t>((has_ack ? 0x01 : 0) |
                                            (slots.empty() ? 0x02 : 0)));
    append_u16(out, static_cast<std::uint16_t>(frame_size));
    append_u16(out, frame_seq);
    append_u32(out, timestamp_us);
    out.push_back(static_cast<std::uint8_t>(slots.size()));
    out.push_back(0);  // 帧头保留位。

    for (const auto& slot : slots) {
        out.push_back(slot.type);
        out.push_back(slot.flags);
        append_u16(out, slot.sequence);
        append_u16(out, static_cast<std::uint16_t>(slot.payload.size()));
        out.insert(out.end(), slot.payload.begin(), slot.payload.end());
    }
    append_u32(out, crc32_iso_hdlc(out.data(), out.size()));
    return out;
}

std::optional<std::vector<RoboLinkSlot>> decode_frame(
    const std::vector<std::uint8_t>& frame)
{
    if (frame.size() < kHeaderSize + kCrcSize ||
        frame.size() > kMaxFrameSize || frame[0] != 0xD3 ||
        frame[1] != 0x91 || frame[2] != 0x02 ||
        (frame[3] & static_cast<std::uint8_t>(~0x03u)) != 0 ||
        read_u16(frame.data() + 4) != frame.size() ||
        frame[13] != 0 ||
        ((frame[3] & 0x02u) != 0) != (frame[12] == 0)) {
        return std::nullopt;
    }

    const std::uint32_t expected_crc = read_u32(frame.data() + frame.size() - kCrcSize);
    const std::uint32_t actual_crc = crc32_iso_hdlc(frame.data(), frame.size() - kCrcSize);
    if (expected_crc != actual_crc) {
        return std::nullopt;
    }

    std::vector<RoboLinkSlot> slots;
    slots.reserve(frame[12]);
    std::size_t offset = kHeaderSize;
    bool has_ack = false;
    for (std::size_t index = 0; index < frame[12]; ++index) {
        if (offset + kSlotHeaderSize > frame.size() - kCrcSize) return std::nullopt;
        RoboLinkSlot slot;
        slot.type = frame[offset];
        slot.flags = frame[offset + 1];
        slot.sequence = read_u16(frame.data() + offset + 2);
        const std::size_t payload_size = read_u16(frame.data() + offset + 4);
        offset += kSlotHeaderSize;
        if (payload_size > kMaxPayloadSize ||
            payload_size > frame.size() - kCrcSize - offset ||
            (slot.flags & static_cast<std::uint8_t>(~0x0Fu)) != 0 ||
            !payload_size_matches(slot.type, payload_size) ||
            !class_flags_match(slot.type, slot.flags)) return std::nullopt;
        slot.payload.assign(frame.begin() + offset, frame.begin() + offset + payload_size);
        offset += payload_size;
        has_ack |= slot.type == 0x04;
        slots.push_back(std::move(slot));
    }
    if (offset != frame.size() - kCrcSize ||
        ((frame[3] & 0x01u) != 0) != has_ack) return std::nullopt;
    return slots;
}

std::vector<std::uint8_t> encode_single_slot_frame(
    std::uint16_t frame_seq, std::uint32_t timestamp_us,
    const RoboLinkSlot& slot)
{
    return encode_frame(frame_seq, timestamp_us, {slot});
}

std::optional<RoboLinkSlot> decode_single_slot_frame(
    const std::vector<std::uint8_t>& frame)
{
    const auto slots = decode_frame(frame);
    if (!slots || slots->size() != 1) return std::nullopt;
    return slots->front();
}
// 检查来源和模式和使能位是否符合目前协议
std::vector<std::uint8_t> encode_control_snapshot_payload(
    const ControlSnapshot& snapshot)
{
    if (snapshot.source > 4 || snapshot.mode > 7 ||
        (snapshot.flags & static_cast<std::uint16_t>(~0x000Fu)) != 0) {
        throw std::invalid_argument("控制来源、模式或使能位非法");
    }

    std::vector<std::uint8_t> out;
    out.reserve(kControlPayloadSize); // 提前预留空间
    append_u32(out, snapshot.control_seq);
    out.push_back(snapshot.source);
    out.push_back(snapshot.mode);
    append_u16(out, snapshot.flags);
    append_f32(out, snapshot.chassis_vx);
    append_f32(out, snapshot.chassis_vy);
    append_f32(out, snapshot.chassis_wz);
    append_f32(out, snapshot.gimbal_yaw_deg);
    append_f32(out, snapshot.gimbal_pitch_deg);
    append_f32(out, snapshot.gimbal_yaw_rate);
    append_f32(out, snapshot.gimbal_pitch_rate);
    append_f32(out, snapshot.friction_rads);
    append_f32(out, snapshot.feeder_rads);
    append_u16(out, std::min<std::uint16_t>(snapshot.validity_hint_ms, 50));  // 有效期提示最多写入 50 ms；不能用提示值延长电控的硬超时
    append_u16(out, 0);  // 载荷末尾的保留字段
    return out;
}

std::optional<SystemStatus> decode_system_status_payload(
    const std::uint8_t* data, std::size_t size)
{
    if ((size != kStatusCoreSize && size != kStatusFullSize) || data == nullptr ||
        data[7] != 0 || data[23] != 0 || data[22] > 3) {
        return std::nullopt;
    }

    SystemStatus out;
    out.uptime_ms = read_u32(data);
    out.mode = data[4] <= 7 ? data[4] : 6;  // 未知模式按 SAFETY。
    out.control_source = data[5] <= 4 ? data[5] : 0;  // 未知来源按 NONE。
    out.safety_state = data[6];
    out.last_rx_control_seq = read_u32(data + 8);
    out.last_accepted_control_seq = read_u32(data + 12);
    out.last_applied_control_seq = read_u32(data + 16);
    out.control_msg_age_ms = read_u16(data + 20);
    out.link_state = data[22];
    out.fault_flags = read_u32(data + 24);

    if (size == kStatusFullSize) {
        if (data[47] != 0 || read_u32(data + 48) != 0) {
            return std::nullopt;
        }
        SystemDiagnostics diag;
        diag.rx_frames = read_u32(data + 28);
        diag.tx_frames = read_u32(data + 32);
        diag.crc_errors = read_u32(data + 36);
        diag.rx_overflow = read_u16(data + 40);
        diag.tx_busy_drop = read_u16(data + 42);
        diag.queue_drop = read_u16(data + 44);
        diag.event_queue_depth = data[46];
        out.diagnostics = diag;
    }
    return out;
}

std::vector<std::uint8_t> encode_hello_payload(const Hello& message)
{
    if (message.proto_ver_major != 2 || message.device_type > 1 ||
        message.heartbeat_period_ms == 0 ||
        message.max_frame_bytes < 96 || message.max_frame_bytes > 512) {
        throw std::invalid_argument("HELLO 版本、设备类型或周期非法");
    }
    std::vector<std::uint8_t> out;
    out.reserve(kHelloSize);
    out.push_back(message.proto_ver_major);
    out.push_back(message.proto_ver_minor);
    out.push_back(message.device_type);
    out.insert(out.end(), message.fw_version.begin(), message.fw_version.end());
    append_u32(out, message.git_short);
    append_u64(out, message.capabilities);
    append_u16(out, message.heartbeat_period_ms);
    append_u16(out, message.max_frame_bytes);
    append_u32(out, message.device_id_digest);
    return out;
}

std::optional<Hello> decode_hello_payload(const std::uint8_t* data, std::size_t size)
{
    if (size != kHelloSize || data == nullptr || data[2] > 1 ||
        read_u16(data + 20) < 96 ||
        read_u16(data + 20) > 512) {
        return std::nullopt;
    }
    Hello out;
    out.proto_ver_major = data[0];
    out.proto_ver_minor = data[1];
    out.device_type = data[2];
    std::copy(data + 3, data + 6, out.fw_version.begin());
    out.git_short = read_u32(data + 6);
    out.capabilities = read_u64(data + 10);
    out.heartbeat_period_ms = read_u16(data + 18) == 0 ? 50 : read_u16(data + 18);
    out.max_frame_bytes = read_u16(data + 20);
    out.device_id_digest = read_u32(data + 22);
    return out;
}

std::vector<std::uint8_t> encode_hello_ack_payload(const HelloAck& message)
{
    if (message.status > 2 ||
        message.heartbeat_period_ms == 0) {
        throw std::invalid_argument("HELLO_ACK 协商结果非法");
    }
    std::vector<std::uint8_t> out;
    out.reserve(kHelloAckSize);
    append_u16(out, message.acked_msg_seq);
    out.push_back(message.negotiated_major);
    out.push_back(message.status);
    append_u16(out, message.heartbeat_period_ms);
    append_u16(out, 0);
    return out;
}

std::optional<HelloAck> decode_hello_ack_payload(const std::uint8_t* data, std::size_t size)
{
    if (size != kHelloAckSize || data == nullptr || data[3] > 2 ||
        read_u16(data + 6) != 0) {
        return std::nullopt;
    }
    HelloAck out;
    out.acked_msg_seq = read_u16(data);
    out.negotiated_major = data[2];
    out.status = data[3];
    out.heartbeat_period_ms = read_u16(data + 4) == 0 ? 50 : read_u16(data + 4);
    return out;
}

std::vector<std::uint8_t> encode_ack_payload(const Ack& message)
{
    if (message.ack_state > 3 || message.error_code > 5) {
        throw std::invalid_argument("ACK 状态或错误码非法");
    }
    std::vector<std::uint8_t> out;
    out.reserve(kAckSize);
    out.push_back(message.acked_msg_type);
    append_u16(out, message.acked_msg_seq);
    out.push_back(message.ack_state);
    append_u16(out, message.error_code);
    out.push_back(0);
    return out;
}

std::optional<Ack> decode_ack_payload(const std::uint8_t* data, std::size_t size)
{
    if (size != kAckSize || data == nullptr || data[3] > 3 ||
        read_u16(data + 4) > 5 || data[6] != 0) {
        return std::nullopt;
    }
    Ack out;
    out.acked_msg_type = data[0];
    out.acked_msg_seq = read_u16(data + 1);
    out.ack_state = data[3];
    out.error_code = read_u16(data + 4);
    return out;
}

std::vector<std::uint8_t> encode_error_payload(const ErrorMessage& message)
{
    if (message.error_code > 5 || message.detail.size() > kMaxErrorDetailSize) {
        throw std::invalid_argument("ERROR 错误码或详情长度非法");
    }
    std::vector<std::uint8_t> out;
    out.reserve(2 + message.detail.size());
    append_u16(out, message.error_code);
    out.insert(out.end(), message.detail.begin(), message.detail.end());
    return out;
}

std::optional<ErrorMessage> decode_error_payload(const std::uint8_t* data, std::size_t size)
{
    if (size < 2 || size > 2 + kMaxErrorDetailSize || data == nullptr ||
        read_u16(data) > 5) {
        return std::nullopt;
    }
    ErrorMessage out;
    out.error_code = read_u16(data);
    out.detail.assign(data + 2, data + size);
    return out;
}

std::vector<std::uint8_t> make_control_snapshot_frame(
    std::uint16_t frame_seq, std::uint32_t timestamp_us,
    std::uint16_t message_seq, const ControlSnapshot& snapshot)
{
    RoboLinkSlot slot;
    slot.type = 0x20;
    slot.flags = 0x02;  // 控制快照属于 LATEST 类。
    slot.sequence = message_seq;
    slot.payload = encode_control_snapshot_payload(snapshot);
    return encode_single_slot_frame(frame_seq, timestamp_us, slot);
}

std::optional<SystemStatus> decode_system_status_frame(
    const std::vector<std::uint8_t>& frame)
{
    const auto slot = decode_single_slot_frame(frame);
    if (!slot.has_value() || slot->type != 0x10 || slot->flags != 0) {
        return std::nullopt;
    }
    return decode_system_status_payload(slot->payload.data(), slot->payload.size());
}

}  // namespace foray_platform
