#include <array>
#include <cstdint>
#include <vector>
#include <gtest/gtest.h>

#include "foray_platform/robolink_frame.hpp"
#include "foray_platform/robolink_frame_parser.hpp"

namespace {

std::vector<std::uint8_t> as_vector(const std::array<std::uint8_t, 18>& frame)
{
    return {frame.begin(), frame.end()};
}

TEST(RoboLinkFrameParser, WaitsForSecondHalf)
{
    foray_platform::RoboLinkFrameParser parser;
    const auto frame = foray_platform::make_keepalive_frame(1, 100);

    EXPECT_TRUE(parser.feed(frame.data(), 5).empty());
    const auto parsed = parser.feed(frame.data() + 5, frame.size() - 5);
    ASSERT_EQ(parsed.size(), 1u);
    EXPECT_EQ(parsed[0], as_vector(frame));
}

TEST(RoboLinkFrameParser, ExtractsTwoFramesFromOneRead)
{
    foray_platform::RoboLinkFrameParser parser;
    const auto first = foray_platform::make_keepalive_frame(1, 100);
    const auto second = foray_platform::make_keepalive_frame(2, 200);
    std::vector<std::uint8_t> bytes(first.begin(), first.end());
    bytes.insert(bytes.end(), second.begin(), second.end());

    const auto parsed = parser.feed(bytes.data(), bytes.size());
    ASSERT_EQ(parsed.size(), 2u);
    EXPECT_EQ(parsed[0], as_vector(first));
    EXPECT_EQ(parsed[1], as_vector(second));
}

TEST(RoboLinkFrameParser, AcceptsVariableLengthFrameWithOneSlot)
{
    foray_platform::RoboLinkFrameParser parser;
    // 候选 HEARTBEAT 槽位（类型 0x03、零载荷）；前 20 字节的 CRC 由 Python 独立计算。
    const std::vector<std::uint8_t> frame{
        0xD3, 0x91, 0x02, 0x00, 0x18, 0x00, 0x08, 0x00,
        0x10, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x03, 0x00, 0x34, 0x12, 0x00, 0x00,
        0x1A, 0xAD, 0x76, 0xC5};

    const auto parsed = parser.feed(frame.data(), frame.size());
    ASSERT_EQ(parsed.size(), 1u);
    EXPECT_EQ(parsed[0], frame);
}

TEST(RoboLinkFrameParser, SkipsGarbageBeforeMagic)
{
    foray_platform::RoboLinkFrameParser parser;
    const auto frame = foray_platform::make_keepalive_frame(3, 300);
    std::vector<std::uint8_t> bytes{0x00, 0x91, 0xD3, 0x00};
    bytes.insert(bytes.end(), frame.begin(), frame.end());

    const auto parsed = parser.feed(bytes.data(), bytes.size());
    ASSERT_EQ(parsed.size(), 1u);
    EXPECT_EQ(parsed[0], as_vector(frame));
}

TEST(RoboLinkFrameParser, KeepsMagicSplitAcrossReads)
{
    foray_platform::RoboLinkFrameParser parser;
    const auto frame = foray_platform::make_keepalive_frame(6, 600);
    const std::uint8_t prefix[] = {0x00, 0xD3};

    EXPECT_TRUE(parser.feed(prefix, sizeof(prefix)).empty());
    const auto parsed = parser.feed(frame.data() + 1, frame.size() - 1);
    ASSERT_EQ(parsed.size(), 1u);
    EXPECT_EQ(parsed[0], as_vector(frame));
}

TEST(RoboLinkFrameParser, RejectsBadHeaderThenResynchronizes)
{
    foray_platform::RoboLinkFrameParser parser;
    const auto frame = foray_platform::make_keepalive_frame(7, 700);
    std::vector<std::uint8_t> bytes{
        0xD3, 0x91, 0x02, 0x02, 0xFF, 0xFF,
        0, 0, 0, 0, 0, 0, 0, 0};  // 长度超过 512，不可能是合法帧。
    bytes.insert(bytes.end(), frame.begin(), frame.end());

    const auto parsed = parser.feed(bytes.data(), bytes.size());
    ASSERT_EQ(parsed.size(), 1u);
    EXPECT_EQ(parsed[0], as_vector(frame));
}

TEST(RoboLinkFrameParser, RejectsBadCrcAndRecovers)
{
    foray_platform::RoboLinkFrameParser parser;
    auto bad = foray_platform::make_keepalive_frame(4, 400);
    bad[8] ^= 0x01;  // 改动时间戳，但不重新计算 CRC。
    const auto good = foray_platform::make_keepalive_frame(5, 500);
    std::vector<std::uint8_t> bytes(bad.begin(), bad.end());
    bytes.insert(bytes.end(), good.begin(), good.end());

    const auto parsed = parser.feed(bytes.data(), bytes.size());
    ASSERT_EQ(parsed.size(), 1u);
    EXPECT_EQ(parsed[0], as_vector(good));
}

}  // namespace
