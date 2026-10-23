#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "foray/platform/hal/types.hpp"

namespace foray::platform::hal {

/**
 * @brief 设备选择条件。
 *
 * 匹配优先级：`serial` > `user_defined_name` > `(model + index)` > `ip_address`。
 * 未提供任何条件时驱动应返回 `ErrorCode::kInvalidArgument`，**不得**默认取第 0 个设备。
 */
struct DeviceSelector {
    std::string serial;
    std::string user_defined_name;
    std::string model;
    std::string ip_address;
    Transport transport = Transport::kAny;
    /// @brief 仅在无 serial/name 时作为显式回退。
    std::uint32_t index = 0;
};

/// @brief 触发配置。
struct TriggerConfig {
    TriggerMode mode = TriggerMode::kOff;
    TriggerActivation activation = TriggerActivation::kRisingEdge;
    std::optional<double> debounce_us;
};

/// @brief 相机配置。
struct CameraConfig {
    DeviceSelector selector;

    /// @brief 0 表示使用设备当前值。
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t offset_x = 0;
    std::uint32_t offset_y = 0;

    PixelFormat pixel_format = PixelFormat::kBayerRG8;

    /// @brief 自由运行时使用；触发模式下忽略。
    std::optional<double> frame_rate_hz;
    std::optional<double> exposure_us;
    std::optional<double> gain_db;
    TriggerConfig trigger;

    /// @brief HAL 帧槽数量。
    std::uint32_t buffer_count = 4;
    /// @brief MVS 内部图像节点数（`MV_CC_SetImageNodeNum`）。
    std::uint32_t sdk_node_count = 5;

    /// @brief GigE PTP；首期只保留配置位。
    bool enable_ptp = false;

    /// @brief 设备计数 → 主机时间的标定偏移（纳秒）。
    std::optional<std::int64_t> device_to_host_offset_ns;
    /// @brief 曝光 → 主机接收的标定延迟（纳秒）。
    std::optional<double> exposure_to_host_latency_ns;

    /// @brief 时间戳不可用时是否允许用主机接收时刻填充。
    bool allow_unsynced_stamp = true;

    std::string frame_id = "camera";
};

/// @brief 激光雷达配置。
struct LidarConfig {
    DeviceSelector selector;

    std::string host_ip;
    /// @brief 用户提供的 JSON 配置路径；为空时由驱动生成。
    std::string config_json_path;
    /// @brief 生成 JSON 的运行时目录。
    std::string runtime_dir;

    enum class PointDataType {
        kHigh,
        kLow,
        kSpherical,
        kDualEcho,
    };
    enum class ScanPattern {
        kNonRepetitive,
        kRepetitive,
    };

    PointDataType data_type = PointDataType::kHigh;
    ScanPattern scan_pattern = ScanPattern::kNonRepetitive;
    bool dual_emit = false;
    bool imu_enabled = true;

    /// @brief 覆盖机型默认线数（Mid-360 = 4，HAP = 6）。
    std::optional<std::uint8_t> line_num;

    /// @brief 原始包环槽数。
    std::uint32_t packet_ring_slots = 4096;
    /// @brief 单个原始包最大字节数。
    std::uint32_t max_packet_bytes = 1500;
    /// @brief 单帧最大点数。
    std::uint32_t point_buffer_capacity = 200000;
    /// @brief HAL 点云帧槽数。
    std::uint32_t frame_buffer_count = 3;
    /// @brief 非重复扫描的时间组帧窗口；10 Hz 点云建议 100 ms。
    ///
    /// Mid-360 系列在非重复扫描下 `frame_cnt` 无效，必须按时间切帧。
    std::uint32_t frame_duration_ms = 100;
    /// @brief 组帧空闲超时（数据中断时强制发布当前帧）。
    std::uint32_t frame_timeout_ms = 200;

    std::string frame_id = "lidar";
};

} // namespace foray::platform::hal
