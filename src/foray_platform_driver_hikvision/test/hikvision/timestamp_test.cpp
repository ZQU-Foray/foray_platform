#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "foray/platform/hal/config.hpp"

#include "hikvision/mvs_nodes.hpp"
#include "hikvision/timestamp.hpp"

namespace foray::platform::driver::hikvision::detail {
namespace {

/// @brief 一个固定的主机接收时刻，便于手算期望值。
constexpr std::int64_t kWallNs = 1'700'000'000'000'000'000LL;
constexpr std::int64_t kSteadyNs = 5'000'000'000LL;

TimingSample make_sample(std::uint64_t ticks) {
    TimingSample sample;
    sample.device_ticks = ticks;
    sample.receive_wall_ns = kWallNs;
    sample.receive_steady_ns = kSteadyNs;
    return sample;
}

/// @brief 100 MHz 设备时钟：1e8 tick == 1 秒。
constexpr std::uint32_t kTicksPerSecond = 100'000'000U;

/// @brief 1 GHz 设备时钟：1 tick == 1 ns，便于把设备时间直接写成纳秒手算期望值。
constexpr std::uint32_t kNanosecondsPerSecond = 1'000'000'000U;

/**
 * @brief 源码：`docs/driver_hikvision.md` §7 路径①。
 *
 * PTP 从时钟优先，stamp 直接取设备时间，不掺主机时刻。
 */
TEST(HikvisionTiming, AutoPrefersPtpWhenSlave) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kAuto;
    config.ptp_slave = true;
    config.tick_frequency_hz = kTicksPerSecond;
    config.device_to_host_offset_ns = 12'345; // 应被 PTP 覆盖

    const auto result = compute_timing(make_sample(kTicksPerSecond), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kPtp);
    EXPECT_EQ(result.device_time_ns, 1'000'000'000);
    EXPECT_EQ(result.device_tick_frequency_hz, kTicksPerSecond);
    EXPECT_EQ(result.stamp.nanoseconds_since_epoch(), 1'000'000'000);
}

/**
 * @brief §7 路径②：有设备时间 + 标定偏移，`stamp = 设备时间 + 偏移`。
 *
 * 审计 H12：旧实现是 `receive_wall_ns + offset`，被加数错了、方向也反了。
 */
TEST(HikvisionTiming, CalibratedOffsetAppliesToDeviceTime) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kAuto;
    config.device_to_host_offset_ns = 5'000;
    config.exposure_to_host_latency_ns = 900; // 只有路径③才用
    config.tick_frequency_hz = kNanosecondsPerSecond;

    // 设备时间比墙钟慢 5 µs，加上偏移后正好落在墙钟上。
    const auto result = compute_timing(make_sample(kWallNs - 5'000), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kDeviceCalibrated);
    EXPECT_EQ(result.device_time_ns, kWallNs - 5'000);
    EXPECT_EQ(result.stamp.nanoseconds_since_epoch(), kWallNs);
}

/// @brief 负偏移同样按「设备 → 主机」的加性修正，不能取绝对值或反向相减。
TEST(HikvisionTiming, CalibratedOffsetHonoursNegativeOffset) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kDevice;
    config.device_to_host_offset_ns = -7'500;
    config.tick_frequency_hz = kNanosecondsPerSecond;

    // 设备时间比墙钟快 7.5 µs（PTP 反向偏差）。
    const auto result = compute_timing(make_sample(kWallNs + 7'500), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kDeviceCalibrated);
    EXPECT_EQ(result.stamp.nanoseconds_since_epoch(), kWallNs);
}

/**
 * @brief 标定后的 `stamp` 与主机接收时刻的偏差应当只剩传输延迟（< 1 ms）。
 *
 * 这是审计 H12 的验收口径：有同步时 `stamp` 要落在主机墙钟附近（误差 < 1 ms），
 * 而不是漂在设备计数或两倍偏移上。设设备时钟比主机慢 1.25 µs、帧在路上走 400 µs，
 * 则 `stamp = 主机接收时刻 − 400 µs`，且 `stamp == device_time + offset`。
 */
TEST(HikvisionTiming, CalibratedStampTracksHostClockWithinOneMillisecond) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kAuto;
    config.device_to_host_offset_ns = 1'250;
    config.tick_frequency_hz = kNanosecondsPerSecond;

    const std::int64_t transmission_delay_ns = 400'000;
    const auto result = compute_timing(
        make_sample(static_cast<std::uint64_t>(kWallNs - transmission_delay_ns - 1'250)), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kDeviceCalibrated);
    const std::int64_t stamp_ns = result.stamp.nanoseconds_since_epoch();
    EXPECT_EQ(stamp_ns, result.device_time_ns + config.device_to_host_offset_ns);
    const std::int64_t error_ns = kWallNs - stamp_ns;
    EXPECT_LT(error_ns, 1'000'000);
    EXPECT_EQ(error_ns, transmission_delay_ns);
}

/**
 * @brief 设备时间换算不出来时，标定偏移无从施加，退回主机接收。
 *
 * 审计 H12：旧实现在这条路径上输出 `receive_wall + offset` 并标成
 * `kDeviceCalibrated`，等于凭空造了一个「已标定」的设备时间。
 */
TEST(HikvisionTiming, AutoWithoutDeviceTimeFallsBackToHostReceive) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kAuto;
    config.device_to_host_offset_ns = 5'000;
    config.exposure_to_host_latency_ns = 900; // 只有路径③才用

    const auto result = compute_timing(make_sample(0), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kHostReceive);
    EXPECT_EQ(result.stamp.nanoseconds_since_epoch(), kWallNs - 900);
}

/**
 * @brief §7 路径③：无任何同步，允许兜底时用主机接收时刻倒推曝光时刻。
 */
TEST(HikvisionTiming, AutoFallsBackToHostReceive) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kAuto;
    config.exposure_to_host_latency_ns = 1'000;

    const auto result = compute_timing(make_sample(0), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kHostReceive);
    EXPECT_EQ(result.stamp.nanoseconds_since_epoch(), kWallNs - 1'000);
}

/// @brief 无同步且禁止兜底：宁可为零，也不给一个不可信的时间。
TEST(HikvisionTiming, AutoKeepsZeroStampWhenUnsyncedDisallowed) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kAuto;
    config.allow_unsynced_stamp = false;

    const auto result = compute_timing(make_sample(kTicksPerSecond), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kNoSync);
    EXPECT_TRUE(result.stamp.is_zero());
}

/**
 * @brief `kDevice` 强制设备时间，tick 读不到时降级为 `kNoSync`。
 *
 * 关键：**不回退**主机时刻——把未同步的时间伪装成同步比没有时间更危险。
 */
TEST(HikvisionTiming, DeviceSourceWithoutTicksDoesNotFallBack) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kDevice;
    config.tick_frequency_hz = 0;
    config.allow_unsynced_stamp = true; // 也应被忽略

    const auto result = compute_timing(make_sample(123'456), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kNoSync);
    EXPECT_TRUE(result.stamp.is_zero());
    EXPECT_EQ(result.device_time_ns, 0);
}

/**
 * @brief `kDevice` 只有 tick、既无 PTP 也无标定偏移时，`stamp` 必须为零。
 *
 * `device_time_ns` 仍要照实填（它是设备时钟域的读数，供离线重算），但它属于另一
 * 个时间域，绝不能冒充墙钟写进 `stamp`——旧实现正是这么做的，测试也照抄了错误期望。
 */
TEST(HikvisionTiming, DeviceSourceWithoutCalibrationReportsNoSync) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kDevice;
    config.tick_frequency_hz = kNanosecondsPerSecond;

    const auto result = compute_timing(make_sample(2 * kNanosecondsPerSecond), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kNoSync);
    EXPECT_TRUE(result.stamp.is_zero());
    EXPECT_EQ(result.device_time_ns, 2'000'000'000);
}

/// @brief `kDevice` + PTP 从时钟：设备计数就是墙钟，`stamp` 与 `device_time_ns` 一致。
TEST(HikvisionTiming, DeviceSourceUsesPtpClockWhenSlave) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kDevice;
    config.ptp_slave = true;
    config.tick_frequency_hz = kNanosecondsPerSecond;

    const auto result = compute_timing(make_sample(kWallNs), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kPtp);
    EXPECT_EQ(result.stamp.nanoseconds_since_epoch(), kWallNs);
    EXPECT_EQ(result.device_time_ns, kWallNs);
}

/// @brief 标定偏移把时间推出 int64 时判「没有同步手段」，不能回绕出负时间戳。
TEST(HikvisionTiming, CalibratedOffsetOverflowYieldsNoSync) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kDevice;
    config.allow_unsynced_stamp = false;
    config.device_to_host_offset_ns = 1'000;
    config.tick_frequency_hz = kNanosecondsPerSecond;

    const auto result = compute_timing(
        make_sample(static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() - 10)),
        config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kNoSync);
    EXPECT_TRUE(result.stamp.is_zero());
    EXPECT_GT(result.device_time_ns, 0);
}

/// @brief `kHostReceive` 强制走路径③，即使 PTP 是 Slave。
TEST(HikvisionTiming, HostReceiveIgnoresPtp) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kHostReceive;
    config.ptp_slave = true;
    config.tick_frequency_hz = kTicksPerSecond;
    config.exposure_to_host_latency_ns = 2'500;

    const auto result = compute_timing(make_sample(kTicksPerSecond), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kHostReceive);
    EXPECT_EQ(result.stamp.nanoseconds_since_epoch(), kWallNs - 2'500);
}

/// @brief `receive_stamp` 与来源策略无关，任何分支都必须记录。
TEST(HikvisionTiming, ReceiveStampAlwaysRecorded) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kAuto;

    const auto result = compute_timing(make_sample(0), config);

    EXPECT_EQ(result.receive_stamp.nanoseconds_since_epoch(), kSteadyNs);
}

/// @brief 余数按向下取整换算，不能因为整数除法把小数部分吃掉。
TEST(HikvisionTiming, TickRemainderIsRoundedDown) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kDevice;
    config.tick_frequency_hz = 3;

    // 10 / 3 = 3 秒 + 1/3 秒 → 3'333'333'333 ns
    const auto result = compute_timing(make_sample(10), config);

    EXPECT_EQ(result.device_time_ns, 3'333'333'333);
}

/// @brief tick 溢出 int64 纳秒时视为时钟不可信，降级而非回绕。
TEST(HikvisionTiming, TickOverflowYieldsNoSync) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kDevice;
    config.tick_frequency_hz = 1;

    const auto result = compute_timing(
        make_sample(std::numeric_limits<std::uint64_t>::max()), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kNoSync);
    EXPECT_EQ(result.device_time_ns, 0);
}

/// @brief 1 MHz 的亚秒余数也要保留精度。
TEST(HikvisionTiming, SubSecondTicksKeepPrecision) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kDevice;
    config.tick_frequency_hz = 1'000'000;

    const auto result = compute_timing(make_sample(1'500'000), config);

    EXPECT_EQ(result.device_time_ns, 1'500'000'000);
}

/**
 * @brief `make_timestamp_config()` 只在显式开启 PTP 时采纳 Slave 状态。
 *
 * 理由：设备单方面进入 Slave 不该让驱动打上 `kPtp` 标记，否则上层会误信时基。
 */
TEST(HikvisionTiming, PtpRequiresExplicitEnable) {
    hal::CameraConfig camera_config;
    camera_config.enable_ptp = false;

    const auto disabled = make_timestamp_config(camera_config, true, kTicksPerSecond);
    EXPECT_FALSE(disabled.ptp_slave);

    camera_config.enable_ptp = true;
    const auto enabled = make_timestamp_config(camera_config, true, kTicksPerSecond);
    EXPECT_TRUE(enabled.ptp_slave);
}

/// @brief 配置里的 `double` 延迟换算成整数纳秒，不能出现截断丢失到 0 的意外。
TEST(HikvisionTiming, MakeConfigConvertsLatencyToNanoseconds) {
    hal::CameraConfig camera_config;
    camera_config.exposure_to_host_latency_ns = 1'234.0;
    camera_config.device_to_host_offset_ns = -4'000;
    camera_config.allow_unsynced_stamp = false;

    const auto config = make_timestamp_config(camera_config, false, 0);

    EXPECT_EQ(config.exposure_to_host_latency_ns, 1'234);
    EXPECT_EQ(config.device_to_host_offset_ns, -4'000);
    EXPECT_FALSE(config.allow_unsynced_stamp);
    EXPECT_EQ(config.tick_frequency_hz, 0U);
}

} // namespace
} // namespace foray::platform::driver::hikvision::detail
