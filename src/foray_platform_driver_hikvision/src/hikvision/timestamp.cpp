#include "timestamp.hpp"

#include <limits>

namespace foray::platform::driver::hikvision::detail {

namespace {

constexpr std::uint64_t kNanosecondsPerSecond = 1000000000ULL;

/**
 * @brief 设备 tick 换算为纳秒。
 *
 * 先算整数部分再算余数，避免 `ticks * 1e9` 在 64 位 tick 上溢出；余数上界
 * `freq - 1 < 2^32`，`rem * 1e9 < 4.3e18` 仍在 uint64 内。整秒部分超出 int64 时
 * 认为时钟不可信，返回 0（上层据此降级为 `kNoSync`）。
 */
std::int64_t ticks_to_nanoseconds(std::uint64_t ticks, std::uint32_t frequency_hz) noexcept {
    if (frequency_hz == 0) {
        return 0;
    }
    const auto frequency = static_cast<std::uint64_t>(frequency_hz);
    const std::uint64_t whole_seconds = ticks / frequency;
    const std::uint64_t remainder = ticks % frequency;
    constexpr auto kMaxSeconds =
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) /
        kNanosecondsPerSecond;
    if (whole_seconds > kMaxSeconds) {
        return 0;
    }
    const std::uint64_t nanoseconds =
        whole_seconds * kNanosecondsPerSecond +
        (remainder * kNanosecondsPerSecond) / frequency;
    return static_cast<std::int64_t>(nanoseconds);
}

time::Timestamp host_receive_stamp(const TimingSample& sample,
                                   const TimestampConfig& config) noexcept {
    return time::Timestamp::from_nanoseconds(sample.receive_wall_ns -
                                             config.exposure_to_host_latency_ns);
}

} // namespace

TimingResult compute_timing(const TimingSample& sample,
                            const TimestampConfig& config) noexcept {
    TimingResult result;
    // receive_stamp 用单调时钟，语义是「进程收到这一帧的时刻」，始终可用。
    result.receive_stamp = time::Timestamp::from_nanoseconds(sample.receive_steady_ns);

    if (config.tick_frequency_hz > 0 && sample.device_ticks > 0) {
        result.device_tick_frequency_hz = config.tick_frequency_hz;
        result.device_time_ns =
            ticks_to_nanoseconds(sample.device_ticks, config.tick_frequency_hz);
    }
    const bool device_time_usable = result.device_time_ns > 0;

    if (config.source == hal::TimestampSource::kDevice) {
        if (!device_time_usable) {
            // 明确要求设备时间却拿不到：宁可没有时间戳，也不静默回退主机时刻。
            result.sync = hal::TimestampSync::kNoSync;
            return result;
        }
        result.stamp = time::Timestamp::from_nanoseconds(result.device_time_ns);
        result.sync = config.ptp_slave ? hal::TimestampSync::kPtp
                                       : hal::TimestampSync::kDeviceCalibrated;
        return result;
    }

    if (config.source == hal::TimestampSource::kHostReceive) {
        result.stamp = host_receive_stamp(sample, config);
        result.sync = hal::TimestampSync::kHostReceive;
        return result;
    }

    // kAuto：PTP → 标定偏移 → 主机接收，优先级即文档 §7 的顺序。
    if (config.ptp_slave && device_time_usable) {
        result.stamp = time::Timestamp::from_nanoseconds(result.device_time_ns);
        result.sync = hal::TimestampSync::kPtp;
        return result;
    }

    if (config.device_to_host_offset_ns != 0) {
        result.stamp = time::Timestamp::from_nanoseconds(
            sample.receive_wall_ns + config.device_to_host_offset_ns);
        result.sync = hal::TimestampSync::kDeviceCalibrated;
        return result;
    }

    if (config.allow_unsynced_stamp) {
        result.stamp = host_receive_stamp(sample, config);
        result.sync = hal::TimestampSync::kHostReceive;
        return result;
    }

    // 既没有同步手段，配置也禁止兜底：保留零时间戳，让上层能识别出「不可信」。
    result.sync = hal::TimestampSync::kNoSync;
    return result;
}

} // namespace foray::platform::driver::hikvision::detail
