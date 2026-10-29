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
    std::uint64_t packets_dropped = 0;
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
