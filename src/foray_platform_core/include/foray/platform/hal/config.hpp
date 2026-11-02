#pragma once

#include <cstddef>
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
    /// @brief 触发线路去抖时间（微秒）；空表示不改设备值。
    std::optional<double> debounce_us;
    /// @brief 显式触发源；空表示由 `mode` 推导（`software` → Software，`lineN` → LineN）。
    std::optional<TriggerMode> source;
};

/// @brief 相机配置。
///
/// 字段与 `docs/camera_config.md` §3 的键空间一一对应；「级」列决定能否热改，
/// 运行期修改走 `Camera::apply(CameraTuning)`。
struct CameraConfig {
    DeviceSelector selector;

    /// @brief 0 表示使用设备当前值。
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t offset_x = 0;
    std::uint32_t offset_y = 0;

    /// @brief 0 表示不改设备值（设备通常为 1）。
    std::uint32_t binning_horizontal = 0;
    std::uint32_t binning_vertical = 0;
    /// @brief 0 表示不改设备值（设备通常为 1）。
    std::uint32_t decimation_horizontal = 0;
    std::uint32_t decimation_vertical = 0;

    PixelFormat pixel_format = PixelFormat::kBayerRG8;

    /// @brief 自由运行时使用；触发模式下忽略。
    std::optional<double> frame_rate_hz;
    /// @brief `AcquisitionFrameRateEnable`；只有显式配置限速时才需要改动。
    bool frame_rate_enable = true;
    std::optional<double> exposure_us;
    /// @brief 曝光自动模式；非 `kOff` 时设备接管曝光，`exposure_us` 被忽略。
    AutoMode exposure_auto = AutoMode::kOff;
    std::optional<double> exposure_auto_lower_us;
    std::optional<double> exposure_auto_upper_us;
    std::optional<double> gain_db;
    AutoMode gain_auto = AutoMode::kOff;
    std::optional<double> gain_auto_lower_db;
    std::optional<double> gain_auto_upper_db;
    TriggerConfig trigger;

    /// @brief HAL 帧槽数量。
    std::uint32_t buffer_count = 4;
    /// @brief MVS 内部图像节点数（`MV_CC_SetImageNodeNum`）。
    std::uint32_t sdk_node_count = 5;
    /// @brief 帧池总内存上限；0 表示不限制。
    std::size_t pool_memory_limit_bytes = 0;

    /// @brief GigE 传输层参数；0 / 空表示不改设备值。
    std::uint32_t packet_size = 0;
    std::optional<std::uint32_t> inter_packet_delay_us;
    std::optional<std::uint32_t> heartbeat_timeout_ms;
    /// @brief 枚举结果中的 `nNetExport`（网卡索引）；空表示自动。
    std::optional<std::int64_t> net_export;

    /// @brief GigE PTP；首期只保留配置位。
    bool enable_ptp = false;

    /// @brief 设备计数 → 主机时间的标定偏移（纳秒）。
    std::optional<std::int64_t> device_to_host_offset_ns;
    /// @brief 曝光 → 主机接收的标定延迟（纳秒）。
    std::optional<double> exposure_to_host_latency_ns;

    /// @brief 时间戳来源策略；`kAuto` 走 §7 的优先级推导。
    TimestampSource timestamp_source = TimestampSource::kAuto;

    /// @brief 时间戳不可用时是否允许用主机接收时刻填充。
    bool allow_unsynced_stamp = true;

    std::string frame_id = "camera";
};

/// @brief 运行期可热改的字段集合，用于 `Camera::apply()`。
///
/// 未设置的字段保持不变；`frame_rate_hz` 用双层 optional 区分
/// 「不改」（外层空）与「清除限速」（外层有值、内层空）。
/// 键与字段的对应关系见 `docs/camera_config.md` §5.2。
struct CameraTuning {
    std::optional<std::optional<double>> frame_rate_hz;
    std::optional<std::optional<double>> exposure_us;
    std::optional<AutoMode> exposure_auto;
    std::optional<std::optional<double>> exposure_auto_lower_us;
    std::optional<std::optional<double>> exposure_auto_upper_us;
    std::optional<std::optional<double>> gain_db;
    std::optional<AutoMode> gain_auto;
    std::optional<std::optional<double>> gain_auto_lower_db;
    std::optional<std::optional<double>> gain_auto_upper_db;
    std::optional<std::optional<double>> trigger_debounce_us;
    std::optional<TimestampSource> timestamp_source;
    std::optional<std::optional<std::int64_t>> device_to_host_offset_ns;
    std::optional<std::optional<double>> exposure_to_host_latency_ns;
    std::optional<bool> allow_unsynced_stamp;
    std::optional<std::string> frame_id;

    /// @brief 是否所有字段都为空（无实际修改）。
    bool empty() const noexcept {
        return !frame_rate_hz.has_value() && !exposure_us.has_value() && !exposure_auto.has_value() &&
               !exposure_auto_lower_us.has_value() && !exposure_auto_upper_us.has_value() &&
               !gain_db.has_value() && !gain_auto.has_value() && !gain_auto_lower_db.has_value() &&
               !gain_auto_upper_db.has_value() && !trigger_debounce_us.has_value() &&
               !timestamp_source.has_value() && !device_to_host_offset_ns.has_value() &&
               !exposure_to_host_latency_ns.has_value() && !allow_unsynced_stamp.has_value() &&
               !frame_id.has_value();
    }
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
