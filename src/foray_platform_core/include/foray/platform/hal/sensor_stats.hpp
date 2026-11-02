#pragma once

#include <cstdint>

#include "foray/platform/time/timestamp.hpp"

namespace foray::platform::hal {

/// @brief 传感器运行统计快照。
struct SensorStats {
    std::uint64_t frames_received = 0;
    std::uint64_t frames_delivered = 0;
    std::uint64_t frames_dropped = 0;
    std::uint64_t packets_received = 0;
    /// @brief 本组件自己丢掉的包（解析失败、队列满、并发合并掉的控制帧等）。
    std::uint64_t packets_dropped = 0;
    /**
     * @brief 按协议内的序号连续性推算出的「上游没送到」的包（审计 H23）。
     *
     * 与 `packets_dropped` 不是一回事：那个是到了本进程但被丢掉的，这个是压根没到。
     * 只有自带包序号且序号逐包递增的传感器能给出这个值（Livox 点云包按 `udp_cnt`），
     * 其余传感器保持 0——不要拿 0 当「没丢包」的证据，它只表示「测不出来」。
     */
    std::uint64_t packets_lost = 0;
    std::uint64_t bytes_received = 0;
    std::uint64_t timeouts = 0;
    std::uint64_t sdk_errors = 0;
    std::uint64_t reconnects = 0;
    std::uint64_t last_sequence = 0;
    std::int32_t last_error_vendor_code = 0;
    time::Timestamp last_frame_stamp;
    time::Timestamp last_frame_receive_stamp;
};

} // namespace foray::platform::hal
