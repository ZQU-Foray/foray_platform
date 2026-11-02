#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "foray/platform/link/robolink_byte_channel.hpp"
#include "foray/platform/link/robolink_codec.hpp"

namespace foray::platform::link {

enum class HostLinkState { Disconnected, HelloPending, Ready, Active };

// 接收时保留 MCU 采样/发帧时刻和 PC 本机接收时刻；两端时钟不能直接相减。
struct ReceivedGimbalAttitude {
    GimbalAttitude attitude;
    std::uint32_t frame_timestamp_us = 0;
    std::uint64_t received_at_ms = 0;
};

// 主机侧链路生命周期；时间由调用方提供，便于回放和无硬件测试。
// 不计算瞄准角，不自动产生控制指令，也不替代电控本地失联保护。
class RoboLinkHostLink {
public:
    explicit RoboLinkHostLink(UsbCdcPort& port);

    // 执行一次非阻塞收发和超时检查；断线后按间隔重新尝试打开设备。
    HostLinkState step(std::uint64_t now_ms);

    // 只有收到有效握手和状态回读后才允许排入控制快照；序号由本链路统一分配。
    bool queue_control(ControlSnapshot snapshot, std::uint64_t now_ms);

    HostLinkState state() const { return state_; }
    const std::optional<SystemStatus>& latest_status() const { return latest_status_; }
    const std::optional<ReceivedGimbalAttitude>& latest_attitude() const { return latest_attitude_; }

private:
    void disconnect(std::uint64_t now_ms);
    void queue_hello(std::uint64_t now_ms);
    bool queue_heartbeat(std::uint64_t now_ms);

    UsbCdcPort& port_;
    RoboLinkByteChannel channel_;
    HostLinkState state_ = HostLinkState::Disconnected;
    std::optional<SystemStatus> latest_status_;
    std::optional<ReceivedGimbalAttitude> latest_attitude_;
    std::uint64_t last_rx_ms_ = 0;
    std::uint64_t last_tx_ms_ = 0;
    std::uint64_t last_hello_ms_ = 0;
    std::uint64_t next_open_ms_ = 0;
    std::uint16_t frame_seq_ = 0;
    std::uint16_t hello_seq_ = 0;
    std::uint16_t control_message_seq_ = 0;
    std::uint32_t control_seq_ = 0;
};

}  // namespace foray::platform::link
