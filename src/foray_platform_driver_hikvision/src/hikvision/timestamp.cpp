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

/**
 * @brief 把标定偏移加到设备时间上，溢出时报「不可用」而不是回绕。
 *
 * 回绕出来的时间戳比现在差几十年，上层根本看不出来源已经错了，因此宁可判
 * `kNoSync`。手写上下界判断而不依赖编译器内建，保证 g++/clang 行为一致。
 */
bool offset_device_time(std::int64_t device_time_ns, std::int64_t offset_ns,
                        std::int64_t& out) noexcept {
    if (offset_ns > 0 && device_time_ns > std::numeric_limits<std::int64_t>::max() - offset_ns) {
        return false;
    }
    if (offset_ns < 0 && device_time_ns < std::numeric_limits<std::int64_t>::min() - offset_ns) {
        return false;
    }
    out = device_time_ns + offset_ns;
    return true;
}

} // namespace

TimingResult compute_timing(const TimingSample& sample,
                            const TimestampConfig& config) noexcept {
    TimingResult result;
    // receive_stamp 用单调时钟，语义是「进程收到这一帧的时刻」，始终可用。
    result.receive_stamp = time::Timestamp::from_nanoseconds(sample.receive_steady_ns);

    // 设备计数只写 `device_time_ns`：它是设备时钟域的原始换算值，而 `stamp` 必须是
    // 绝对墙钟（`hal/frame.hpp` 契约）。这两个域的值永远不许互相顶替。
    if (config.tick_frequency_hz > 0 && sample.device_ticks > 0) {
        result.device_tick_frequency_hz = config.tick_frequency_hz;
        result.device_time_ns =
            ticks_to_nanoseconds(sample.device_ticks, config.tick_frequency_hz);
    }
    const bool device_time_usable = result.device_time_ns > 0;

    if (config.source == hal::TimestampSource::kHostReceive) {
        result.stamp = host_receive_stamp(sample, config);
        result.sync = hal::TimestampSync::kHostReceive;
        return result;
    }

    // kDevice 与 kAuto 共用同一条解析链，差别只在「最后能不能用主机时刻兜底」。
    if (device_time_usable && config.ptp_slave) {
        // PTP 从时钟在线时，设备计数本身就在墙钟时基上，可以直接当 `stamp`。
        result.stamp = time::Timestamp::from_nanoseconds(result.device_time_ns);
        result.sync = hal::TimestampSync::kPtp;
        return result;
    }

    if (device_time_usable && config.device_to_host_offset_ns != 0) {
        // 偏移的语义是「设备时间 + 偏移 = 主机时间」。旧实现拿 receive_wall 去加它，
        // 既用错了被加数，又把方向反了，等于每帧凭空偏了两倍标定量。
        std::int64_t calibrated_ns = 0;
        if (offset_device_time(result.device_time_ns, config.device_to_host_offset_ns,
                               calibrated_ns)) {
            result.stamp = time::Timestamp::from_nanoseconds(calibrated_ns);
            result.sync = hal::TimestampSync::kDeviceCalibrated;
            return result;
        }
        // 偏移把时间推出 int64：标定值本身不可信，按「没有同步手段」处理。
    }

    if (config.source == hal::TimestampSource::kAuto && config.allow_unsynced_stamp) {
        result.stamp = host_receive_stamp(sample, config);
        result.sync = hal::TimestampSync::kHostReceive;
        return result;
    }

    // `kDevice`：明确要求设备时间却换算不出墙钟，宁可没有时间戳，也不静默回退主机时刻。
    // `kAuto`：既没有 PTP、也没有标定偏移，配置还禁止兜底 —— 保留零时间戳。
    result.sync = hal::TimestampSync::kNoSync;
    return result;
}

} // namespace foray::platform::driver::hikvision::detail
