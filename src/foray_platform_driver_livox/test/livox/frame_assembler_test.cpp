#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "foray/platform/hal/frame_pool.hpp"
#include "livox/frame_assembler.hpp"
#include "packet_builder.hpp"

namespace foray::platform::driver::livox::detail {
namespace {

constexpr std::size_t kPayloadBytes = 256 * sizeof(hal::PointXYZITL);

// 机型 ID 取自 LivoxLidarDeviceType；见 packet_parser.cpp 的 kDevice* 常量。
constexpr std::uint8_t kMid360DeviceType = 9;
constexpr std::uint8_t kMid360sDeviceType = 35;
constexpr std::uint8_t kHapDeviceType = 15;

// 设备时间单位（纳秒），用于 H18 的乱序/回退用例。
constexpr std::uint64_t kMillisecond = 1'000'000ULL;
constexpr std::uint64_t kSecond = 1'000'000'000ULL;

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
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar");

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

TEST(FrameAssemblerTest, PublishesWhenFrameDurationExpires) {
    auto pool = make_pool(3);
    // 非重复扫描 frame_cnt 无效：用 10 ms 时间窗口切帧。
    FrameAssembler assembler(pool, 256, 4, kMid360DeviceType, "lidar", 10);

    EXPECT_FALSE(feed(assembler, frame_packet(0, 100, 1'000'000, 2), 100));
    EXPECT_FALSE(feed(assembler, frame_packet(0, 101, 5'000'000, 2), 200));
    EXPECT_TRUE(feed(assembler, frame_packet(0, 102, 11'000'000, 2), 300));

    hal::FrameLease<hal::PointCloudFrame> first = pool->try_acquire_latest();
    ASSERT_TRUE(first);
    EXPECT_EQ(first->count, 4U);
    EXPECT_EQ(first->sequence, 1U);
    EXPECT_EQ(first->base_time.nanoseconds_since_epoch(), 1'000'000);

    EXPECT_TRUE(assembler.flush());
    hal::FrameLease<hal::PointCloudFrame> second = pool->try_acquire_latest();
    ASSERT_TRUE(second);
    EXPECT_EQ(second->count, 2U);
    EXPECT_EQ(second->sequence, 2U);
}

TEST(FrameAssemblerTest, PublishesAfterIdleTimeout) {
    auto pool = make_pool(2);
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar");

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 0, 2), 1000));
    EXPECT_FALSE(assembler.publish_if_idle(1000, 1000));
    EXPECT_TRUE(assembler.publish_if_idle(2000, 1000));
    EXPECT_TRUE(pool->try_acquire_latest());
}

TEST(FrameAssemblerTest, NoSyncKeepsBaseTimeZero) {
    auto pool = make_pool(2);
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar");

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
    FrameAssembler assembler(pool, 2, 4, kHapDeviceType, "lidar");

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
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar");

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 0, 2), 100));
    EXPECT_TRUE(feed(assembler, frame_packet(1, 0, 0, 2), 200));
    // 发布后的槽位尚未被消费者释放，新帧拿不到槽位。
    EXPECT_EQ(assembler.dropped_frames(), 1U);

    hal::FrameLease<hal::PointCloudFrame> lease = pool->try_acquire_latest();
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->count, 2U);
}

/**
 * 审计 H23：`udp_cnt` 是设备侧的包序号，同帧内逐包 +1；缺口就是没到的包。
 * 改前代码只在帧边界用到 `udp_cnt`（判 `== 0`），连续性根本不看，丢包完全不可见。
 */
TEST(FrameAssemblerTest, UdpCntGapCountsLostPackets) {
    auto pool = make_pool(3);
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar");

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 1'000'000, 2), 100));
    EXPECT_FALSE(feed(assembler, frame_packet(0, 1, 1'000'500, 2), 200));
    EXPECT_FALSE(feed(assembler, frame_packet(0, 2, 1'001'000, 2), 300));
    // 跳过 udp_cnt = 3、4：这两包没到。
    EXPECT_FALSE(feed(assembler, frame_packet(0, 5, 1'001'500, 2), 400));
    EXPECT_EQ(assembler.udp_cnt_gaps(), 2U);
    EXPECT_EQ(assembler.udp_cnt_anomalies(), 0U);

    // 新帧开始重新数：帧首包的 udp_cnt=0 不能与上一帧末包求差。
    EXPECT_TRUE(feed(assembler, frame_packet(1, 0, 2'000'000, 2), 500));
    EXPECT_EQ(assembler.udp_cnt_gaps(), 2U);
    EXPECT_EQ(assembler.udp_cnt_anomalies(), 0U);
}

/**
 * 乱序/计数重置会让模 65536 的差值接近一整圈，那种情况只记异常次数，不算丢包——
 * 否则一次乱序就能把丢包指标推到几万。
 */
TEST(FrameAssemblerTest, UdpCntRollbackIsAnomalyNotLoss) {
    auto pool = make_pool(3);
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar");

    EXPECT_FALSE(feed(assembler, frame_packet(0, 10, 1'000'000, 2), 100));
    EXPECT_FALSE(feed(assembler, frame_packet(0, 3, 1'000'500, 2), 200));
    EXPECT_EQ(assembler.udp_cnt_gaps(), 0U);
    EXPECT_EQ(assembler.udp_cnt_anomalies(), 1U);
    // 乱序包不改变基准：下一包按「以本包为准」继续算，udp_cnt=4 紧跟 3 不算缺口。
    EXPECT_FALSE(feed(assembler, frame_packet(0, 4, 1'001'000, 2), 300));
    EXPECT_EQ(assembler.udp_cnt_gaps(), 0U);
    EXPECT_EQ(assembler.udp_cnt_anomalies(), 1U);
}

/**
 * Mid-360 系列的 `udp_cnt` 不承担帧内序号语义，按它求差只会得到一堆假缺口。
 */
TEST(FrameAssemblerTest, Mid360DoesNotInferLossFromUdpCnt) {
    auto pool = make_pool(3);
    FrameAssembler assembler(pool, 1024, 4, kMid360DeviceType, "lidar");

    EXPECT_FALSE(feed(assembler, frame_packet(3, 7, 1'000'000, 2), 100));
    EXPECT_FALSE(feed(assembler, frame_packet(9, 41, 1'001'000, 2), 200));
    EXPECT_EQ(assembler.udp_cnt_gaps(), 0U);
    EXPECT_EQ(assembler.udp_cnt_anomalies(), 0U);
}

/**
 * 长跑口径：10000 个序号里每 73 个丢 1 个，缺口必须精确等于丢掉的包数——
 * 这是报告里"改前 0（测不出来）→ 改后 137"那条数值对比的来源。
 */
TEST(FrameAssemblerTest, UdpCntGapsAggregateOverLongRun) {
    auto pool = make_pool(3);
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar");

    constexpr std::uint16_t kTotalPackets = 10000;
    constexpr std::uint16_t kLossStride = 73;
    std::uint64_t expected = 0;
    std::uint16_t emitted = 0;
    for (std::uint16_t index = 0; index < kTotalPackets; ++index) {
        if (index % kLossStride == 0) {
            ++expected;
            continue;
        }
        // frame_cnt 保持 0，只让 udp_cnt 连续递增；每包一个点，避免撑爆帧容量。
        EXPECT_FALSE(feed(assembler, frame_packet(0, index, 1'000'000, 1), index + 1U));
        ++emitted;
    }

    // 帧首包（index=0）也被丢掉了，但它前面没有包可比，无法归因；所以可归因的丢包
    // 是 expected - 1 = 136。这个边界必须写清楚，否则长跑口径会系统性少算第一个包。
    EXPECT_EQ(assembler.udp_cnt_gaps(), expected - 1U);
    // 丢掉的实际包数是 137（index 0, 73, ..., 9929），可归因 136。
    EXPECT_EQ(expected, kTotalPackets / kLossStride + 1U);
    EXPECT_EQ(assembler.udp_cnt_anomalies(), 0U);
    EXPECT_EQ(emitted, kTotalPackets - expected);
}

TEST(FrameAssemblerTest, FrameCountUsabilityFollowsDeviceType) {
    // 非重复扫描机型只认时间窗；其余机型（含未知机型）沿用 frame_cnt 语义。
    EXPECT_FALSE(frame_cnt_represents_frame(kMid360DeviceType));
    EXPECT_FALSE(frame_cnt_represents_frame(kMid360sDeviceType));
    EXPECT_TRUE(frame_cnt_represents_frame(kHapDeviceType));
    EXPECT_TRUE(frame_cnt_represents_frame(10));
    EXPECT_TRUE(frame_cnt_represents_frame(0));
}

/**
 * @brief Mid-360 的 frame_cnt 每包都变时，不能按它切帧。
 *
 * 审计 H17：旧实现无条件用 `frame_cnt` / `udp_cnt == 0` 定帧边界，而 Mid-360 是非重复
 * 扫描，固件不保证这两个字段按帧回零，结果一帧被切成几十个两点的碎帧，下游看到的帧率
 * 远高于 10 Hz、每帧点数远低于预期。
 */
TEST(FrameAssemblerTest, Mid360SplitsByTimeWindowNotFrameCount) {
    auto pool = make_pool(3);
    FrameAssembler assembler(pool, 256, 4, kMid360DeviceType, "lidar", 100);

    // 991 ms 的流量，每包 2 点、每 10 ms 一包，frame_cnt 与 udp_cnt 每包都变。
    std::size_t published_in_loop = 0;
    for (int index = 0; index < 100; ++index) {
        const auto stamp_ns = static_cast<std::uint64_t>(index) * 10'000'000ULL;
        if (feed(assembler,
                 frame_packet(static_cast<std::uint8_t>(index), static_cast<std::uint16_t>(index),
                              stamp_ns, 2),
                 stamp_ns + 1)) {
            ++published_in_loop;
        }
    }
    // 只有时间窗到期才切帧：100/200/.../900 ms 共 9 次，末尾残留由 flush 补 1 帧。
    EXPECT_EQ(published_in_loop, 9U);
    EXPECT_TRUE(assembler.flush());

    // 时间窗 100 ms + 每帧 10 包 → 10 帧（≈10.1 Hz）；按 frame_cnt 切会切出约 100 个碎帧。
    EXPECT_EQ(assembler.frames_published(), 10U);
    EXPECT_GT(assembler.frame_cnt_discontinuities(), 90U);
}

/// @brief Mid-360s（dev_type 35）同属非重复扫描，行为必须与 Mid-360 一致。
TEST(FrameAssemblerTest, Mid360sAlsoIgnoresFrameCountChanges) {
    auto pool = make_pool(3);
    FrameAssembler assembler(pool, 256, 4, kMid360sDeviceType, "lidar", 100);

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 1'000'000, 2), 100));
    // frame_cnt 变了，但还在同一个 100 ms 时间窗内：不许切帧。
    EXPECT_FALSE(feed(assembler, frame_packet(7, 3, 50'000'000, 2), 200));
    EXPECT_EQ(assembler.frames_published(), 0U);
    EXPECT_EQ(assembler.point_count(), 4U);
    EXPECT_EQ(assembler.frame_cnt_discontinuities(), 1U);

    // 越过时间窗才发布，且一帧包含两包共 4 点。
    EXPECT_TRUE(feed(assembler, frame_packet(9, 4, 150'000'000, 2), 300));
    hal::FrameLease<hal::PointCloudFrame> lease = pool->try_acquire_latest();
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->count, 4U);
    EXPECT_EQ(lease->sequence, 1U);
}

/**
 * @brief 边界包时间戳回退时，不能把它当成新帧基准。
 *
 * 审计 H18：旧实现无条件把触发边界的那一包的时间戳当作 `base_device_ns_`。设备时间一旦
 * 回退（乱序包、集线器重排、PTP 阶跃残留），新帧的基准就比真实时间更早，100 ms 时间窗
 * 提前到期，帧被切成短帧、逐点 `time_offset_ns` 也被算歪。
 */
TEST(FrameAssemblerTest, OutOfOrderBoundaryPacketDoesNotResetFrameBase) {
    auto pool = make_pool(4);
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar", 100);

    constexpr std::uint64_t kStreamOrigin = 5 * kSecond; // 从整秒起，确保单调水线已建立。
    std::size_t frames_in_loop = 0;
    for (int index = 0; index <= 100; ++index) {
        const auto stamp_ns =
            kStreamOrigin + static_cast<std::uint64_t>(index) * 10ULL * kMillisecond;
        const auto frame_cnt = static_cast<std::uint8_t>(index / 10);
        const bool attack = index % 20 == 10;
        // 每两帧制造一次边界包时间戳回退：比上一帧基准（stamp - 100 ms）还早 90 ms，
        // 仍落在 1 s 抖动窗口内，属于必须丢掉的乱序包。
        const std::uint64_t packet_stamp = attack ? stamp_ns - 190ULL * kMillisecond : stamp_ns;
        const auto udp_cnt = attack ? std::uint16_t{0} : static_cast<std::uint16_t>(index % 10);
        if (feed(assembler, frame_packet(frame_cnt, udp_cnt, packet_stamp, 2), stamp_ns + 1)) {
            ++frames_in_loop;
        }
    }
    EXPECT_GT(assembler.stale_time_packets(), 0U);
    EXPECT_EQ(assembler.clock_jumps(), 0U);
    EXPECT_TRUE(assembler.flush());

    // 1010 ms 的设备时间、100 ms 时间窗 → 10 帧（≈10 Hz）；基准被回退包带歪时会多切出短帧。
    EXPECT_NEAR(static_cast<double>(assembler.frames_published()), 10.0, 1.0);
}

/// @brief 设备时间整段回退（PTP 阶跃 /
/// 重新授时）必须被当成新纪元接受，否则驱动永远等不到更大的时间戳。
TEST(FrameAssemblerTest, LargeClockRollbackStartsNewEpoch) {
    auto pool = make_pool(4);
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar", 100);

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 5 * kSecond, 2), 100));
    EXPECT_TRUE(feed(assembler, frame_packet(1, 0, 5 * kSecond + 100 * kMillisecond, 2), 200));

    // 回退 4 s：超过抖动窗口，按新纪元接受并计数。
    EXPECT_TRUE(feed(assembler, frame_packet(2, 0, 1 * kSecond, 2), 300));
    EXPECT_EQ(assembler.clock_jumps(), 1U);
    EXPECT_EQ(assembler.stale_time_packets(), 0U);

    // 新纪元下的后续包正常累积：再过 100 ms 才切帧。
    EXPECT_FALSE(feed(assembler, frame_packet(2, 1, 1 * kSecond + 50 * kMillisecond, 2), 400));
    EXPECT_TRUE(feed(assembler, frame_packet(3, 0, 1 * kSecond + 100 * kMillisecond, 2), 500));
    EXPECT_EQ(assembler.frames_published(), 3U);
}

/// @brief 已有设备时间水线后，时间戳为 0 的滞后包不能被当成新帧基准。
TEST(FrameAssemblerTest, ZeroTimestampAfterWaterlineIsDropped) {
    auto pool = make_pool(4);
    FrameAssembler assembler(pool, 256, 4, kHapDeviceType, "lidar", 100);

    EXPECT_FALSE(feed(assembler, frame_packet(0, 0, 5 * kSecond, 2), 100));
    // 边界包时间戳为 0：先按 frame_cnt 发布，再拒绝它成为新基准。
    EXPECT_TRUE(feed(assembler, frame_packet(1, 0, 0, 2), 200));
    EXPECT_EQ(assembler.stale_time_packets(), 1U);
    EXPECT_EQ(assembler.clock_jumps(), 0U);

    // 该包被丢弃，帧里只剩它的点数没有进入任何帧；后续正常包把它补成新帧。
    EXPECT_FALSE(feed(assembler, frame_packet(1, 1, 5 * kSecond + 50 * kMillisecond, 2), 300));
    EXPECT_TRUE(feed(assembler, frame_packet(2, 0, 5 * kSecond + 100 * kMillisecond, 2), 400));
    hal::FrameLease<hal::PointCloudFrame> lease = pool->try_acquire_latest();
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->device_base_time_ns, 5 * kSecond + 50 * kMillisecond);
}

} // namespace
} // namespace foray::platform::driver::livox::detail
