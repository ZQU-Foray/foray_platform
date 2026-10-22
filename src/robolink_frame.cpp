#include "foray_platform/robolink_frame.hpp"

#include "foray_platform/robolink_crc.hpp"

namespace foray_platform {

std::array<std::uint8_t, 18> make_keepalive_frame(
    std::uint16_t seq, std::uint32_t timestamp_us)
{
    std::array<std::uint8_t, 18> frame{};

    // RoboLink v0.1 候选格式：14 字节帧头 + 4 字节 CRC，均按小端编码。
    frame[0] = 0xD3;
    frame[1] = 0x91;
    frame[2] = 0x02;  // 协议版本。
    frame[3] = 0x02;  // 零槽保活标志。
    frame[4] = 18;    // 整帧长度低字节。
    frame[5] = 0;     // 整帧长度高字节。

    frame[6] = static_cast<std::uint8_t>(seq & 0xFF);
    frame[7] = static_cast<std::uint8_t>((seq >> 8) & 0xFF);

    frame[8] = static_cast<std::uint8_t>(timestamp_us & 0xFF);
    frame[9] = static_cast<std::uint8_t>((timestamp_us >> 8) & 0xFF);
    frame[10] = static_cast<std::uint8_t>((timestamp_us >> 16) & 0xFF);
    frame[11] = static_cast<std::uint8_t>((timestamp_us >> 24) & 0xFF);

    frame[12] = 0;  // 槽位数量。
    frame[13] = 0;  // 保留位。

    const std::uint32_t crc = crc32_iso_hdlc(frame.data(), 14);
    frame[14] = static_cast<std::uint8_t>(crc & 0xFF);
    frame[15] = static_cast<std::uint8_t>((crc >> 8) & 0xFF);
    frame[16] = static_cast<std::uint8_t>((crc >> 16) & 0xFF);
    frame[17] = static_cast<std::uint8_t>((crc >> 24) & 0xFF);

    return frame;
}

}  // namespace foray_platform
