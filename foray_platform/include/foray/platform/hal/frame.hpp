#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "foray/platform/hal/types.hpp"
#include "foray/platform/time/timestamp.hpp"

namespace foray::platform::hal {

/**
 * @brief 一帧图像。
 *
 * `data` 指向 HAL 帧池中的预分配缓冲；生命周期由 `FrameLease` 保证。
 * `frame_id` 指向驱动配置中的字符串，驱动必须保证其生命周期长于帧。
 */
struct ImageFrame {
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t stride = 0;
    PixelFormat pixel_format = PixelFormat::kMono8;

    std::uint64_t sequence = 0;
    std::uint32_t device_frame_number = 0;
    std::uint32_t trigger_index = 0;

    /// @brief 绝对墙钟；无绝对同步时为零。
    time::Timestamp stamp;
    /// @brief 主机接收时刻（SteadyClock）。
    time::Timestamp receive_stamp;
    /// @brief 原始设备计数（若可换算）。
    std::int64_t device_time_ns = 0;
    std::uint32_t device_tick_frequency_hz = 0;
    TimestampSync sync = TimestampSync::kUnknown;

    std::string_view frame_id;

    float exposure_time_us = 0;
    float gain_db = 0;
    std::uint32_t lost_packets = 0;
};

/// @brief 带逐点时间的点（传感器安装系，单位 m）。
struct PointXYZITL {
    float x = 0;
    float y = 0;
    float z = 0;
    float intensity = 0;
    std::int32_t time_offset_ns = 0;
    std::uint8_t line = 0;
    std::uint8_t tag = 0;
    std::uint16_t reserved = 0;
};

/// @brief 一帧点云。
struct PointCloudFrame {
    const PointXYZITL* points = nullptr;
    std::size_t count = 0;
    /// @brief 首点绝对时间；无绝对同步时为零。
    time::Timestamp base_time;
    time::Timestamp receive_stamp;
    std::int64_t device_base_time_ns = 0;
    TimestampSync sync = TimestampSync::kUnknown;
    std::uint64_t sequence = 0;
    std::string_view frame_id;
    std::uint32_t dropped_packets = 0;
};

/// @brief 一帧 IMU 采样。
struct ImuSample {
    float gyro_x = 0;
    float gyro_y = 0;
    float gyro_z = 0;
    float acc_x = 0;
    float acc_y = 0;
    float acc_z = 0;

    time::Timestamp stamp;
    time::Timestamp receive_stamp;
    std::int64_t device_time_ns = 0;
    TimestampSync sync = TimestampSync::kUnknown;
    std::uint64_t sequence = 0;
    std::string_view frame_id;
};

} // namespace foray::platform::hal
