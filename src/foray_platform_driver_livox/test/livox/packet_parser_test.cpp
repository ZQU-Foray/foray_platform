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

/**
 * @brief H19：逐点相对时间存 int32 ns，越界必须拒包而不是静默溢出。
 *
 * 旧实现 `static_cast<std::int32_t>(packet_offset_ns + intra_ns)` 在组帧窗口被配得过大
 * （> 2147 ms）时会把偏移截断成随机值甚至负数，时间域整体错乱且没有任何告警。
 */
TEST(PacketParserTest, RejectsPointOffsetOutsideInt32) {
    std::vector<std::uint8_t> payload;
    test::HighPointWire point{1000, 0, 0, 0, 0};
    test::append(payload, &point, sizeof(point));

    const auto parse_with = [&payload](std::uint64_t timestamp_ns, std::int64_t base_ns) {
        const std::vector<std::uint8_t> packet =
            make_packet(0x01, 1, 1, 0, 0, timestamp_ns, 0, payload);
        const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
        hal::PointXYZITL out{};
        std::size_t count = 0;
        const Result<void> parsed =
            parse_points(header.value(), packet.data(), packet.size(), 1, base_ns, &out, 1, count);
        return std::pair<Result<void>, std::int32_t>(parsed, out.time_offset_ns);
    };

    constexpr std::int64_t kInt32Max = 2'147'483'647;
    constexpr std::int64_t kInt32Min = -2'147'483'648;

    // 边界值本身仍可表达（0 与 ±(2^31-1) / -2^31）。
    EXPECT_TRUE(parse_with(0, 0).first.ok());
    EXPECT_EQ(parse_with(0, 0).second, 0);
    EXPECT_TRUE(parse_with(static_cast<std::uint64_t>(kInt32Max), 0).first.ok());
    EXPECT_EQ(parse_with(static_cast<std::uint64_t>(kInt32Max), 0).second, kInt32Max);
    EXPECT_TRUE(parse_with(std::uint64_t{0}, -kInt32Min).first.ok());
    EXPECT_EQ(parse_with(std::uint64_t{0}, -kInt32Min).second, kInt32Min);

    // 正/负各越界一格：整包拒收，错误码为参数非法。
    const auto above = parse_with(static_cast<std::uint64_t>(kInt32Max + 1), 0);
    EXPECT_FALSE(above.first.ok());
    EXPECT_EQ(above.first.error().code, ErrorCode::kInvalidArgument);
    const auto below = parse_with(std::uint64_t{0}, -kInt32Min + 1);
    EXPECT_FALSE(below.first.ok());
    EXPECT_EQ(below.first.error().code, ErrorCode::kInvalidArgument);
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

// 审计 H21：`rsvd`(12–23) 与 `crc32`(24–27) 这两段本仓不读也不校验——校验由 Livox
// SDK2 在回调前完成。把这两段任意改成垃圾，解析结果必须一字不差；哪天有人给它加校验，
// 这条会红，届时 `docs/driver_livox.md` §4 的偏移表也必须一起改。
TEST(PacketParserTest, HeaderIgnoresReservedAndCrc32Bytes) {
    const std::vector<std::uint8_t> payload = test::high_point_payload(1000, -2000, 3000, 77, 3);
    const std::vector<std::uint8_t> clean =
        make_packet(0x01, 1, 1, 9, 4, 123456789ULL, 10, payload);
    std::vector<std::uint8_t> dirty = clean;
    for (std::size_t offset = 12; offset < 28; ++offset) {
        dirty[offset] = 0xEE;
    }

    const Result<PacketHeader> original = parse_header(clean.data(), clean.size());
    const Result<PacketHeader> tampered = parse_header(dirty.data(), dirty.size());
    ASSERT_TRUE(original.ok()) << original.error().message;
    ASSERT_TRUE(tampered.ok()) << tampered.error().message;
    EXPECT_EQ(tampered.value().length, original.value().length);
    EXPECT_EQ(tampered.value().time_interval, original.value().time_interval);
    EXPECT_EQ(tampered.value().dot_num, original.value().dot_num);
    EXPECT_EQ(tampered.value().udp_cnt, original.value().udp_cnt);
    EXPECT_EQ(tampered.value().frame_cnt, original.value().frame_cnt);
    EXPECT_EQ(tampered.value().data_type, original.value().data_type);
    EXPECT_EQ(tampered.value().time_type, original.value().time_type);
    EXPECT_EQ(tampered.value().timestamp_ns, original.value().timestamp_ns);
}

TEST(PacketParserTest, ConvertsIpv4AndModel) {
    const std::uint32_t handle = ipv4_to_handle("192.168.1.100");
    EXPECT_NE(handle, 0U);
    EXPECT_EQ(handle_to_ipv4(handle), "192.168.1.100");
    EXPECT_EQ(ipv4_to_handle("not-an-ip"), 0U);
    EXPECT_EQ(default_line_num(9), 4);
    EXPECT_EQ(default_line_num(10), 6);
    EXPECT_EQ(default_line_num(1), 1);
    EXPECT_STREQ(model_name(9), "Livox Mid-360");
}

} // namespace
} // namespace foray::platform::driver::livox::detail
