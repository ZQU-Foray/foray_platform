#pragma once

#include <cstdint>
#include <cstddef>

namespace foray::platform::link {
    std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t size);

}  // namespace foray::platform::link
