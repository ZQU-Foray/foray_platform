#include <array>
#include <cstdint>
#include <gtest/gtest.h>

#include "foray/platform/link/robolink_frame.hpp"

TEST(RoboLinkFrame, KeepaliveMatchesIndependentGoldenBytes)
{
    // 前 14 字节的 CRC 由 Python zlib.crc32 独立计算。
    constexpr std::array<std::uint8_t, 18> expected{
        0xD3, 0x91, 0x02, 0x02, 0x12, 0x00,
        0x34, 0x12, 0x78, 0x56, 0x34, 0x12,
        0x00, 0x00, 0xEF, 0x29, 0x5A, 0x96};

    EXPECT_EQ(foray::platform::link::make_keepalive_frame(0x1234u, 0x12345678u), expected);
}
