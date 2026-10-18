#include <gtest/gtest.h>

#include <cstring>

#include "livox/packet_parser.hpp"
#include "packet_builder.hpp"

namespace foray::platform::driver::livox::detail {
namespace {

using test::append;
using test::make_packet;

TEST(PacketParserTest, ParsesHeaderFields) {
    const std::vector<std::uint8_t> payload = test::high_point_payload(1000, -2000, 3000, 77, 3);
    const std::vector<std::uint8_t> packet =
        make_packet(0x01, 1, 1, 9, 4, 123456789ULL, 10, payload);

    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    ASSERT_TRUE(header.ok()) << header.error().message;
    EXPECT_EQ(header.value().version, 1);
    EXPECT_EQ(header.value().length, static_cast<std::uint16_t>(packet.size()));
    EXPECT_EQ(header.value().time_interval, 10);
    EXPECT_EQ(header.value().dot_num, 1);
    EXPECT_EQ(header.value().udp_cnt, 4);
    EXPECT_EQ(header.value().frame_cnt, 9);
    EXPECT_EQ(header.value().data_type, 0x01);
    EXPECT_EQ(header.value().time_type, 1);
    EXPECT_EQ(header.value().timestamp_ns, 123456789ULL);
}

TEST(PacketParserTest, RejectsTruncatedPacket) {
    const std::vector<std::uint8_t> packet(10, 0);
    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    EXPECT_FALSE(header.ok());
    EXPECT_EQ(header.error().code, ErrorCode::kFrameTooLarge);
}

TEST(PacketParserTest, RejectsLengthLargerThanBuffer) {
    std::vector<std::uint8_t> packet(40, 0);
    test::put_u16(packet, 1, 100);
    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    EXPECT_FALSE(header.ok());
    EXPECT_EQ(header.error().code, ErrorCode::kFrameTooLarge);
}

TEST(PacketParserTest, ConvertsHighPrecisionPointsAndOffsets) {
    std::vector<std::uint8_t> payload;
    for (int index = 0; index < 4; ++index) {
        test::HighPointWire point{1000 * (index + 1), -500, 200,
                                  static_cast<std::uint8_t>(10 + index),
                                  static_cast<std::uint8_t>(index)};
        append(payload, &point, sizeof(point));
    }

    const std::vector<std::uint8_t> packet =
        make_packet(0x01, 1, 4, 0, 0, 5'000'000'000ULL, 10, payload);
    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    ASSERT_TRUE(header.ok());

    hal::PointXYZITL points[4];
    std::size_t count = 0;
    const Result<void> parsed = parse_points(header.value(), packet.data(), packet.size(), 4,
                                             5'000'000'000LL, points, 4, count);
    ASSERT_TRUE(parsed.ok()) << parsed.error().message;
    ASSERT_EQ(count, 4U);

    EXPECT_FLOAT_EQ(points[0].x, 1.0F);
    EXPECT_FLOAT_EQ(points[0].y, -0.5F);
    EXPECT_FLOAT_EQ(points[0].z, 0.2F);
    EXPECT_FLOAT_EQ(points[0].intensity, 10.0F);
    EXPECT_EQ(points[0].tag, 0);
    EXPECT_EQ(points[0].line, 0);
    EXPECT_EQ(points[3].line, 3);

    // time_interval = 10 (1 µs)，dot_num = 4：逐点间隔 250 ns。
    EXPECT_EQ(points[0].time_offset_ns, 0);
    EXPECT_EQ(points[1].time_offset_ns, 250);
    EXPECT_EQ(points[2].time_offset_ns, 500);
    EXPECT_EQ(points[3].time_offset_ns, 750);
}

TEST(PacketParserTest, ConvertsLowPrecisionPoints) {
    test::LowPointWire point{100, -200, 300, 42, 2};
    std::vector<std::uint8_t> payload;
    append(payload, &point, sizeof(point));

    const std::vector<std::uint8_t> packet = make_packet(0x02, 0, 1, 0, 0, 0, 0, payload);
    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    ASSERT_TRUE(header.ok());

    hal::PointXYZITL out{};
    std::size_t count = 0;
    ASSERT_TRUE(
        parse_points(header.value(), packet.data(), packet.size(), 1, 0, &out, 1, count).ok());
    EXPECT_FLOAT_EQ(out.x, 1.0F);
    EXPECT_FLOAT_EQ(out.y, -2.0F);
    EXPECT_FLOAT_EQ(out.z, 3.0F);
    EXPECT_FLOAT_EQ(out.intensity, 42.0F);
    EXPECT_EQ(out.tag, 2);
}

TEST(PacketParserTest, ConvertsSphericalPoints) {
    // depth = 1000 mm，theta = 0°，phi = 0° → (1, 0, 0)。
    test::SphericalPointWire first{1000, 0, 0, 5, 0};
    // theta = 9000（90°）→ (0, 1, 0)。
    test::SphericalPointWire second{1000, 9000, 0, 6, 0};
    std::vector<std::uint8_t> payload;
    append(payload, &first, sizeof(first));
    append(payload, &second, sizeof(second));

    const std::vector<std::uint8_t> packet = make_packet(0x03, 1, 2, 0, 0, 0, 0, payload);
    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    ASSERT_TRUE(header.ok());

    hal::PointXYZITL out[2];
    std::size_t count = 0;
    ASSERT_TRUE(
        parse_points(header.value(), packet.data(), packet.size(), 1, 0, out, 2, count).ok());
    EXPECT_NEAR(out[0].x, 1.0F, 1e-5F);
    EXPECT_NEAR(out[0].y, 0.0F, 1e-5F);
    EXPECT_NEAR(out[1].x, 0.0F, 1e-5F);
    EXPECT_NEAR(out[1].y, 1.0F, 1e-5F);
}

TEST(PacketParserTest, RejectsShortPayload) {
    std::vector<std::uint8_t> payload(5, 0);
    const std::vector<std::uint8_t> packet = make_packet(0x01, 1, 1, 0, 0, 0, 0, payload);
    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    ASSERT_TRUE(header.ok());

    hal::PointXYZITL out{};
    std::size_t count = 0;
    const Result<void> parsed =
        parse_points(header.value(), packet.data(), packet.size(), 1, 0, &out, 1, count);
    EXPECT_FALSE(parsed.ok());
    EXPECT_EQ(parsed.error().code, ErrorCode::kInvalidArgument);
}

TEST(PacketParserTest, RejectsDualEcho) {
    std::vector<std::uint8_t> payload(28, 0);
    const std::vector<std::uint8_t> packet = make_packet(0x11, 1, 1, 0, 0, 0, 0, payload);
    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    ASSERT_TRUE(header.ok());

    hal::PointXYZITL out{};
    std::size_t count = 0;
    const Result<void> parsed =
        parse_points(header.value(), packet.data(), packet.size(), 4, 0, &out, 1, count);
    EXPECT_FALSE(parsed.ok());
    EXPECT_EQ(parsed.error().code, ErrorCode::kUnsupportedFormat);
}

TEST(PacketParserTest, ParsesImu) {
    std::vector<std::uint8_t> payload(24, 0);
    test::put_f32(payload, 0, 0.1F);
    test::put_f32(payload, 4, 0.2F);
    test::put_f32(payload, 8, 0.3F);
    test::put_f32(payload, 12, 9.8F);
    test::put_f32(payload, 16, 9.7F);
    test::put_f32(payload, 20, 9.6F);

    const std::vector<std::uint8_t> packet = make_packet(0x00, 1, 0, 0, 0, 42, 0, payload);
    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    ASSERT_TRUE(header.ok());

    hal::ImuSample sample;
    ASSERT_TRUE(parse_imu(header.value(), packet.data(), packet.size(), sample).ok());
    EXPECT_FLOAT_EQ(sample.gyro_x, 0.1F);
    EXPECT_FLOAT_EQ(sample.gyro_y, 0.2F);
    EXPECT_FLOAT_EQ(sample.gyro_z, 0.3F);
    EXPECT_FLOAT_EQ(sample.acc_x, 9.8F);
    EXPECT_FLOAT_EQ(sample.acc_y, 9.7F);
    EXPECT_FLOAT_EQ(sample.acc_z, 9.6F);
}

TEST(PacketParserTest, MapsTimeType) {
    EXPECT_EQ(sync_from_time_type(0), hal::TimestampSync::kNoSync);
    EXPECT_EQ(sync_from_time_type(1), hal::TimestampSync::kPtp);
    EXPECT_EQ(sync_from_time_type(2), hal::TimestampSync::kGpsPps);
    EXPECT_EQ(sync_from_time_type(3), hal::TimestampSync::kPps);
    EXPECT_EQ(sync_from_time_type(4), hal::TimestampSync::kPps);
    EXPECT_EQ(sync_from_time_type(9), hal::TimestampSync::kUnknown);
}

TEST(PacketParserTest, ConvertsIpv4AndModel) {
    const std::uint32_t handle = ipv4_to_handle("192.168.1.100");
    EXPECT_EQ(handle, 0xC0A80164U);
    EXPECT_EQ(handle_to_ipv4(handle), "192.168.1.100");
    EXPECT_EQ(ipv4_to_handle("not-an-ip"), 0U);
    EXPECT_EQ(default_line_num(9), 4);
    EXPECT_EQ(default_line_num(10), 6);
    EXPECT_EQ(default_line_num(1), 1);
    EXPECT_STREQ(model_name(9), "Livox Mid-360");
}

} // namespace
} // namespace foray::platform::driver::livox::detail
