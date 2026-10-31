#include "foray/platform/link/robolink_host_link.hpp"

#include <system_error>

namespace foray::platform::link {
namespace {

// 候选协议 §5.1：500 ms 无合法帧即断链；心跳默认 20 ms。
constexpr std::uint64_t kLinkTimeoutMs = 500;
constexpr std::uint64_t kHeartbeatMs = 20;
// HELLO 重试和设备重开间隔是主机侧暂定值，电控联调时再验证。
constexpr std::uint64_t kHelloRetryMs = 200;
constexpr std::uint64_t kOpenRetryMs = 200;
// 序号匹配窗口：ACK 可能在本地已重发下一次 HELLO 之后才到，因此接受「最近
// kHelloAckWindow 次已发 HELLO」的 ACK。窗口按 kHelloRetryMs 折算约 800 ms，
// 远大于 USB CDC 往返延迟，又不足以让上一个连接的陈旧 ACK 混进来。
constexpr std::uint16_t kHelloAckWindow = 4;

}  // 匿名命名空间

RoboLinkHostLink::RoboLinkHostLink(UsbCdcPort& port) : port_(port), channel_(port) {}

void RoboLinkHostLink::disconnect(std::uint64_t now_ms)
{
    port_.close();
    channel_.reset();
    state_ = HostLinkState::Disconnected;
    latest_status_.reset();
    latest_attitude_.reset();
    next_open_ms_ = now_ms + kOpenRetryMs;
}

void RoboLinkHostLink::queue_hello(std::uint64_t now_ms)
{
    Hello hello;
    // RoboLink v0.2 §4.1/§4.8：bit 48 声明支持姿态反馈，MCU 才会发送 0x30。
    for (const unsigned type : {1u, 2u, 3u, 4u, 5u, 16u, 32u, 48u}) {
        hello.capabilities |= std::uint64_t{1} << type;
    }
    RoboLinkSlot slot;
    slot.type = 0x01;
    slot.flags = 0x04;
    // 消息序号必须逐次变化：MCU 用 `acked_msg_seq` 回显它应答的是哪一次 HELLO，
    // 如果每次都是 0，「序号匹配」就退化成「有任何 ACK 都算」。
    slot.sequence = hello_seq_;
    slot.payload = encode_hello_payload(hello);
    if (channel_.queue_frame(encode_single_slot_frame(
            frame_seq_, static_cast<std::uint32_t>(now_ms * 1000), slot))) {
        ++frame_seq_;
        ++hello_seq_;
        last_hello_ms_ = now_ms;
        last_tx_ms_ = now_ms;
    }
}

bool RoboLinkHostLink::queue_heartbeat(std::uint64_t now_ms)
{
    if (!channel_.queue_frame(encode_frame(
            frame_seq_, static_cast<std::uint32_t>(now_ms * 1000), {}))) return false;
    ++frame_seq_;
    last_tx_ms_ = now_ms;
    return true;
}

HostLinkState RoboLinkHostLink::step(std::uint64_t now_ms)
{
    if (state_ == HostLinkState::Disconnected) {
        if (now_ms < next_open_ms_) return state_;
        if (!port_.open()) {
            next_open_ms_ = now_ms + kOpenRetryMs;
            return state_;
        }
        channel_.reset();
        state_ = HostLinkState::HelloPending;
        frame_seq_ = 0;
        hello_seq_ = 0;
        control_message_seq_ = 0;
        control_seq_ = 0;
        last_rx_ms_ = now_ms;
        last_tx_ms_ = now_ms;
        queue_hello(now_ms);
    }

    try {
        for (const auto& frame : channel_.receive_once()) {
            const auto slots = decode_frame(frame);
            if (!slots) continue;
            last_rx_ms_ = now_ms;
            for (const auto& slot : *slots) {
                if (state_ == HostLinkState::HelloPending && slot.type == 0x02) {
                    const auto ack = decode_hello_ack_payload(
                        slot.payload.data(), slot.payload.size());
                    // ACK 只能应答我们真的发过、且还没过期的 HELLO。`hello_seq_` 是
                    // 「下一次要用的序号」，所以已发序号落在 [hello_seq_ - 窗口, hello_seq_)。
                    const std::uint16_t ack_age =
                        ack ? static_cast<std::uint16_t>(hello_seq_ - ack->acked_msg_seq)
                            : std::uint16_t{0};
                    if (ack && ack_age >= 1 && ack_age <= kHelloAckWindow && ack->status == 0 &&
                        ack->negotiated_major == 2) {
                        state_ = HostLinkState::Ready;
                        frame_seq_ = 0;  // 候选协议 §5.1：进入 READY 时重置序号。
                        control_message_seq_ = 0;
                        control_seq_ = 0;
                    }
                } else if ((state_ == HostLinkState::Ready ||
                            state_ == HostLinkState::Active) && slot.type == 0x10) {
                    const auto status = decode_system_status_payload(
                        slot.payload.data(), slot.payload.size());
                    if (status) {
                        latest_status_ = status;
                        state_ = HostLinkState::Active;
                    }
                } else if ((state_ == HostLinkState::Ready ||
                            state_ == HostLinkState::Active) && slot.type == 0x30) {
                    const auto attitude = decode_gimbal_attitude_payload(
                        slot.payload.data(), slot.payload.size());
                    if (attitude) {
                        // 帧头偏移 8..11 是 MCU 发帧时刻，区别于载荷的 IMU 采样时刻。
                        const auto frame_stamp = static_cast<std::uint32_t>(frame[8]) |
                            (static_cast<std::uint32_t>(frame[9]) << 8) |
                            (static_cast<std::uint32_t>(frame[10]) << 16) |
                            (static_cast<std::uint32_t>(frame[11]) << 24);
                        latest_attitude_ = ReceivedGimbalAttitude{*attitude, frame_stamp, now_ms};
                    }
                }
            }
        }
        if (now_ms < last_rx_ms_ || now_ms - last_rx_ms_ > kLinkTimeoutMs) {
            disconnect(now_ms);
            return state_;
        }
        if (state_ == HostLinkState::HelloPending &&
            now_ms - last_hello_ms_ >= kHelloRetryMs &&
            !channel_.has_pending_write()) {
            queue_hello(now_ms);
        } else if (state_ != HostLinkState::HelloPending &&
                   now_ms - last_tx_ms_ >= kHeartbeatMs &&
                   !channel_.has_pending_write()) {
            queue_heartbeat(now_ms);
        }
        const ssize_t written = channel_.flush_once();
        if (written < 0) {
            disconnect(now_ms);
        } else if (written > 0) {
            last_tx_ms_ = now_ms;
        }
    } catch (const std::system_error&) {
        disconnect(now_ms);
    }
    return state_;
}

bool RoboLinkHostLink::queue_control(ControlSnapshot snapshot, std::uint64_t now_ms)
{
    if (state_ != HostLinkState::Active || !port_.is_open()) return false;
    if (channel_.has_pending_write()) return false;
    snapshot.control_seq = control_seq_;
    const auto frame = make_control_snapshot_frame(
        frame_seq_, static_cast<std::uint32_t>(now_ms * 1000),
        control_message_seq_, snapshot);
    if (!channel_.queue_frame(frame)) return false;
    ++frame_seq_;
    ++control_message_seq_;
    ++control_seq_;
    return true;
}

}  // namespace foray::platform::link
