#ifndef FORAY_PLATFORM_ROBOLINK_FRAME_HPP
#define FORAY_PLATFORM_ROBOLINK_FRAME_HPP

#include <array>
#include <cstdint>

namespace foray_platform {

/// 按 RoboLink v0.1 候选帧格式构造零槽保活帧。
std::array<std::uint8_t, 18> make_keepalive_frame(
    std::uint16_t seq, std::uint32_t timestamp_us);

}  // namespace foray_platform

#endif
