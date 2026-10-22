#ifndef FORAY_PLATFORM_ROBOLINK_CRC
#define FORAY_PLATFORM_ROBOLINK_CRC

#include <cstdint>
#include <cstddef>

namespace foray_platform
{
    std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t size);

} // namespace foray_platform

#endif
