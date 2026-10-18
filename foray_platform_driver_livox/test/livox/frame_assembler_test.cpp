#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "foray/platform/hal/frame_pool.hpp"
#include "livox/frame_assembler.hpp"
#include "packet_builder.hpp"

namespace foray::platform::driver::livox::detail {
namespace {

constexpr std::size_t kPayloadBytes = 256 * sizeof(hal::PointXYZITL);

std::shared_ptr<hal::FramePool<hal::PointCloudFrame>> make_pool(std::size_t slots) {
    return std::make_shared<hal::FramePool<hal::PointCloudFrame>>(slots, kPayloadBytes);
}

std::vector<std::uint8_t> frame_packet(std::uint8_t frame_cnt, std::uint16_t udp_cnt,
                                       std::uint64_t timestamp_ns, std::uint16_t dot_num) {
    std::vector<std::uint8_t> payload;
    for (std::uint16_t index = 0; index < dot_num; ++index) {
        test::HighPointWire point{static_cast<std::int32_t>(1000 + index), 0, 0,
                                  static_cast<std::uint8_t>(index), 0};
        test::append(payload, &point, sizeof(point));
    }
    return test::make_packet(0x01, 1, dot_num, frame_cnt, udp_cnt, timestamp_ns, 10, payload);
}

bool feed(FrameAssembler& assembler, const std::vector<std::uint8_t>& packet,
          std::uint64_t receive_steady_ns) {
    const Result<PacketHeader> header = parse_header(packet.data(), packet.size());
    EXPECT_TRUE(header.ok()) << header.error().message;
    if (!header.ok()) {
        return false;
    }
    return assembler.on_packet(header.value(), packet.data(), packet.size(), receive_steady_ns);
}

TEST(FrameAssemblerTest, PublishesOnFrameChange) {
    auto pool = make_pool(3);
    FrameAssembler assembler(pool, 256, 4, "lidar");

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 1'000'000, 2), 100));
    EXPECT_FALSE(feed(assembler, frame_packet(0, 1, 1'000'500, 2), 200));
    EXPECT_TRUE(feed(assembler, frame_packet(1, 0, 2'000'000, 1), 300));

    hal::FrameLease<hal::PointCloudFrame> lease = pool->try_acquire_latest();
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->count, 4U);
    EXPECT_EQ(lease->sequence, 1U);
    EXPECT_EQ(lease->sync, hal::TimestampSync::kPtp);
    EXPECT_EQ(lease->base_time.nanoseconds_since_epoch(), 1'000'000);
    EXPECT_EQ(lease->device_base_time_ns, 1'000'000);
    EXPECT_EQ(lease->receive_stamp.nanoseconds_since_epoch(), 100);
    EXPECT_EQ(lease->frame_id, "lidar");

    // 每包 2 点、time_interval = 10（1 µs）：包内间隔 500 ns；
    // 第二包相对首包偏移 500 ns。
    EXPECT_EQ(lease->points[0].time_offset_ns, 0);
    EXPECT_EQ(lease->points[1].time_offset_ns, 500);
    EXPECT_EQ(lease->points[2].time_offset_ns, 500);
    EXPECT_EQ(lease->points[3].time_offset_ns, 1000);
    // 与 livox_ros_driver2 一致：line 按包内点序号对线数取模。
    EXPECT_EQ(lease->points[0].line, 0);
    EXPECT_EQ(lease->points[3].line, 1);
}

TEST(FrameAssemblerTest, PublishesAfterIdleTimeout) {
    auto pool = make_pool(2);
    FrameAssembler assembler(pool, 256, 4, "lidar");

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 0, 2), 1000));
    EXPECT_FALSE(assembler.publish_if_idle(1000, 1000));
    EXPECT_TRUE(assembler.publish_if_idle(2000, 1000));
    EXPECT_TRUE(pool->try_acquire_latest());
}

TEST(FrameAssemblerTest, NoSyncKeepsBaseTimeZero) {
    auto pool = make_pool(2);
    FrameAssembler assembler(pool, 256, 4, "lidar");

    std::vector<std::uint8_t> payload;
    test::HighPointWire point{1000, 0, 0, 1, 0};
    test::append(payload, &point, sizeof(point));
    const std::vector<std::uint8_t> packet =
        test::make_packet(0x01, 0, 1, 0, 0, 5'000'000, 0, payload);

    ASSERT_FALSE(feed(assembler, packet, 42));
    ASSERT_TRUE(assembler.flush());

    hal::FrameLease<hal::PointCloudFrame> lease = pool->try_acquire_latest();
    ASSERT_TRUE(lease);
    EXPECT_TRUE(lease->base_time.is_zero());
    EXPECT_EQ(lease->device_base_time_ns, 5'000'000);
    EXPECT_EQ(lease->sync, hal::TimestampSync::kNoSync);
}

TEST(FrameAssemblerTest, OverflowDropsRemainingFrame) {
    auto pool = make_pool(2);
    FrameAssembler assembler(pool, 2, 4, "lidar");

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 0, 4), 100));
    EXPECT_EQ(assembler.point_count(), 0U);
    EXPECT_EQ(assembler.points_dropped(), 4U);

    // 当前帧已溢出，后续包继续丢弃，直到下一帧。
    EXPECT_FALSE(feed(assembler, frame_packet(0, 1, 0, 2), 200));
    EXPECT_EQ(assembler.points_dropped(), 6U);

    // 下一帧数据正常时仍可发布。
    EXPECT_FALSE(feed(assembler, frame_packet(1, 0, 0, 2), 300));
    EXPECT_TRUE(assembler.flush());
    hal::FrameLease<hal::PointCloudFrame> lease = pool->try_acquire_latest();
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->count, 2U);
}

TEST(FrameAssemblerTest, NoFreeSlotDropsFrame) {
    auto pool = make_pool(1);
    FrameAssembler assembler(pool, 256, 4, "lidar");

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 0, 2), 100));
    EXPECT_TRUE(feed(assembler, frame_packet(1, 0, 0, 2), 200));
    // 发布后的槽位尚未被消费者释放，新帧拿不到槽位。
    EXPECT_EQ(assembler.dropped_frames(), 1U);

    hal::FrameLease<hal::PointCloudFrame> lease = pool->try_acquire_latest();
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->count, 2U);
}

} // namespace
} // namespace foray::platform::driver::livox::detail
