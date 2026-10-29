#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace foray::platform::link {

/// 从任意长度的 USB 字节流中提取 CRC 校验通过的完整帧。
/// 仅实现候选帧格式，不解读消息槽位，也不代表 MCU 固件已支持此格式。
class RoboLinkFrameParser {
public:
    std::vector<std::vector<std::uint8_t>> feed(
        const std::uint8_t* data, std::size_t size);

private:
    std::vector<std::uint8_t> pending_;
};

}  // namespace foray::platform::link
