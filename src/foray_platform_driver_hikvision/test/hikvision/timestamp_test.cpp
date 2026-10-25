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
 * @brief §7 路径②：无 PTP 但有标定偏移，用主机接收时刻 + 偏移。
 */
TEST(HikvisionTiming, AutoFallsBackToCalibratedOffset) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kAuto;
    config.device_to_host_offset_ns = 5'000;
    config.exposure_to_host_latency_ns = 900; // 只有路径③才用

    const auto result = compute_timing(make_sample(0), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kDeviceCalibrated);
    EXPECT_EQ(result.stamp.nanoseconds_since_epoch(), kWallNs + 5'000);
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

/// @brief `kDevice` 有 tick 时按设备时间出 stamp。
TEST(HikvisionTiming, DeviceSourceUsesDeviceClock) {
    TimestampConfig config;
    config.source = hal::TimestampSource::kDevice;
    config.tick_frequency_hz = kTicksPerSecond;

    const auto result = compute_timing(make_sample(2 * kTicksPerSecond), config);

    EXPECT_EQ(result.sync, hal::TimestampSync::kDeviceCalibrated);
    EXPECT_EQ(result.stamp.nanoseconds_since_epoch(), 2'000'000'000);
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
