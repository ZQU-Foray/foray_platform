#include <cstdint>
#include <gtest/gtest.h>

#include "foray_platform/robolink_crc.hpp"

TEST(RoboLinkCrc, StandardCheckValue)
{
    constexpr std::uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(foray_platform::crc32_iso_hdlc(data, sizeof(data)), 0xCBF43926u);
}

TEST(RoboLinkCrc, EmptyInput)
{
    EXPECT_EQ(foray_platform::crc32_iso_hdlc(nullptr, 0), 0u);
}
