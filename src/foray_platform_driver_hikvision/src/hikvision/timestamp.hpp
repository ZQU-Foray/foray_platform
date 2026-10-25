#pragma once

#include <cstdint>

#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/types.hpp"
#include "foray/platform/time/timestamp.hpp"

namespace foray::platform::driver::hikvision::detail {

/**
 * @brief 一帧时间戳所需的配置快照。
 *
 * 全部来自 `hal::CameraConfig` 与设备实际能力，构造一次后可反复用于每帧换算。
 */
struct TimestampConfig {
    hal::TimestampSource source = hal::TimestampSource::kAuto;
    /// 设备时间 → 主机时间的标定偏移；0 表示未标定。
    std::int64_t device_to_host_offset_ns = 0;
    /// 曝光到主机收到 imagemem 的延迟；从主机接收时刻倒推曝光时刻用。
    std::int64_t exposure_to_host_latency_ns = 0;
    /// 无任何同步时是否允许用主机接收时刻兜底。
    bool allow_unsynced_stamp = true;
    /// `GevIEEE1588Status == "Slave"`。
    bool ptp_slave = false;
    /// 设备 tick 频率；0 表示读不到，设备时间不可换算。
    std::uint32_t tick_frequency_hz = 0;
};

/**
 * @brief 一帧的原始时间采样。
 *
 * 两个主机时钟各取一次：`receive_wall_ns` 用于写入 `stamp`（需与设备时间同上
 * 一时基），`receive_steady_ns` 用于写入 `receive_stamp`（单调，可算延迟）。
 */
struct TimingSample {
    std::uint64_t device_ticks = 0;
    std::int64_t receive_wall_ns = 0;
    std::int64_t receive_steady_ns = 0;
};

/// @brief `compute_timing()` 的产出，字段与 `hal::ImageFrame` 一一对应。
struct TimingResult {
    time::Timestamp stamp;
    time::Timestamp receive_stamp;
    std::int64_t device_time_ns = 0;
    std::uint32_t device_tick_frequency_hz = 0;
    hal::TimestampSync sync = hal::TimestampSync::kUnknown;
};

/**
 * @brief 按 `docs/driver_hikvision.md` §7 的三条路径换算一帧时间。
 *
 * - `kAuto`：PTP slave → 标定偏移 → 主机接收 的优先级；
 * - `kDevice`：强制设备计数换算，tick 读不到时 `sync = kNoSync` 且 `stamp` 为零，
 *   **不回退主机时刻**（避免把未同步的时间伪装成同步）；
 * - `kHostReceive`：强制主机接收时刻倒推曝光时刻。
 *
 * 纯函数，不读时钟、不碰设备，可独立单测。
 */
TimingResult compute_timing(const TimingSample& sample,
                            const TimestampConfig& config) noexcept;

} // namespace foray::platform::driver::hikvision::detail
