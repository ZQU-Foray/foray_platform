#include "mvs_nodes.hpp"

#include <string>

#include "foray/platform/hal/config.hpp"
#include "foray/platform/result/error.hpp"

namespace foray::platform::driver::hikvision::detail {

namespace {

/// @brief 只在非 0 时下发：`0` 在 `CameraConfig` 里表示「保留设备当前值」。
Result<void> set_int_if_set(MvsApi& api, void* handle, std::string_view node_name,
                            std::int64_t value) {
    if (value == 0) {
        return Result<void>();
    }
    return api.set_int(handle, node_name, value);
}

Result<void> set_optional_float(MvsApi& api, void* handle, std::string_view node_name,
                                const std::optional<double>& value) {
    if (!value.has_value()) {
        return Result<void>();
    }
    return api.set_float(handle, node_name, *value);
}

/// @brief HAL 像素格式 → MVS `PixelFormat` 枚举名；空表示不支持。
std::string_view to_mvs_pixel_format(hal::PixelFormat format) noexcept {
    switch (format) {
    case hal::PixelFormat::kMono8:
        return "Mono8";
    case hal::PixelFormat::kBayerRG8:
        return "BayerRG8";
    case hal::PixelFormat::kBayerGR8:
        return "BayerGR8";
    case hal::PixelFormat::kBayerGB8:
        return "BayerGB8";
    case hal::PixelFormat::kBayerBG8:
        return "BayerBG8";
    default:
        return {};
    }
}

std::string_view to_mvs_auto_mode(hal::AutoMode mode) noexcept {
    switch (mode) {
    case hal::AutoMode::kOff:
        return "Off";
    case hal::AutoMode::kOnce:
        return "Once";
    case hal::AutoMode::kContinuous:
        return "Continuous";
    }
    return "Off";
}

/// @brief 触发源：`kSoftware` → `Software`，`kLineN` → `LineN`。
std::string_view to_mvs_trigger_source(hal::TriggerMode mode) noexcept {
    switch (mode) {
    case hal::TriggerMode::kSoftware:
        return "Software";
    case hal::TriggerMode::kLine0:
        return "Line0";
    case hal::TriggerMode::kLine1:
        return "Line1";
    case hal::TriggerMode::kLine2:
        return "Line2";
    case hal::TriggerMode::kLine3:
        return "Line3";
    case hal::TriggerMode::kOff:
        break;
    }
    return {};
}

std::string_view to_mvs_trigger_activation(hal::TriggerActivation activation) noexcept {
    switch (activation) {
    case hal::TriggerActivation::kRisingEdge:
        return "RisingEdge";
    case hal::TriggerActivation::kFallingEdge:
        return "FallingEdge";
    case hal::TriggerActivation::kLevelHigh:
        return "LevelHigh";
    case hal::TriggerActivation::kLevelLow:
        return "LevelLow";
    }
    return "RisingEdge";
}

Result<void> apply_geometry(MvsApi& api, void* handle, const hal::CameraConfig& config) {
    // 0 表示「保留设备当前分辨率」：裸写 0 会被 MVS 判成 MV_E_GC_RANGE。
    if (auto status = set_int_if_set(api, handle, node::kWidth, config.width); !status.ok()) {
        return status;
    }
    if (auto status = set_int_if_set(api, handle, node::kHeight, config.height); !status.ok()) {
        return status;
    }
    if (auto status = set_int_if_set(api, handle, node::kOffsetX, config.offset_x);
        !status.ok()) {
        return status;
    }
    if (auto status = set_int_if_set(api, handle, node::kOffsetY, config.offset_y);
        !status.ok()) {
        return status;
    }
    if (auto status =
            set_int_if_set(api, handle, node::kBinningHorizontal, config.binning_horizontal);
        !status.ok()) {
        return status;
    }
    if (auto status =
            set_int_if_set(api, handle, node::kBinningVertical, config.binning_vertical);
        !status.ok()) {
        return status;
    }
    if (auto status = set_int_if_set(api, handle, node::kDecimationHorizontal,
                                     config.decimation_horizontal);
        !status.ok()) {
        return status;
    }
    if (auto status = set_int_if_set(api, handle, node::kDecimationVertical,
                                     config.decimation_vertical);
        !status.ok()) {
        return status;
    }

    const std::string_view pixel_format = to_mvs_pixel_format(config.pixel_format);
    if (pixel_format.empty()) {
        return Result<void>(make_error(
            ErrorCode::kUnsupportedFormat,
            std::string("Hikvision 驱动只支持 8 位 Mono/Bayer，当前配置为 ") +
                std::string(hal::to_string(config.pixel_format))));
    }
    return api.set_enum_by_string(handle, node::kPixelFormat, pixel_format);
}

Result<void> apply_frame_rate(MvsApi& api, void* handle, const hal::CameraConfig& config) {
    const auto enable = api.set_bool(handle, node::kFrameRateEnable, config.frame_rate_enable);
    if (!enable.ok()) {
        return enable;
    }
    if (!config.frame_rate_hz.has_value()) {
        return Result<void>();
    }
    return api.set_float(handle, node::kFrameRate, *config.frame_rate_hz);
}

Result<void> apply_exposure(MvsApi& api, void* handle, const hal::CameraConfig& config) {
    if (auto status =
            api.set_enum_by_string(handle, node::kExposureAuto, to_mvs_auto_mode(config.exposure_auto));
        !status.ok()) {
        return status;
    }
    if (config.exposure_auto != hal::AutoMode::kOff) {
        // 自动曝光接管后手动值不生效，但仍下发一次，便于切回手动时沿用。
        if (auto status = set_optional_float(api, handle, node::kExposureTime, config.exposure_us);
            !status.ok()) {
            return status;
        }
        if (auto status = set_optional_float(api, handle, node::kExposureLowerLimit,
                                             config.exposure_auto_lower_us);
            !status.ok()) {
            return status;
        }
        return set_optional_float(api, handle, node::kExposureUpperLimit,
                                  config.exposure_auto_upper_us);
    }
    return set_optional_float(api, handle, node::kExposureTime, config.exposure_us);
}

Result<void> apply_gain(MvsApi& api, void* handle, const hal::CameraConfig& config) {
    if (auto status =
            api.set_enum_by_string(handle, node::kGainAuto, to_mvs_auto_mode(config.gain_auto));
        !status.ok()) {
        return status;
    }
    if (auto status = set_optional_float(api, handle, node::kGain, config.gain_db); !status.ok()) {
        return status;
    }
    if (auto status = set_optional_float(api, handle, node::kGainLowerLimit,
                                         config.gain_auto_lower_db);
        !status.ok()) {
        return status;
    }
    return set_optional_float(api, handle, node::kGainUpperLimit, config.gain_auto_upper_db);
}

Result<void> apply_trigger(MvsApi& api, void* handle, const hal::CameraConfig& config) {
    const hal::TriggerMode mode = config.trigger.source.value_or(config.trigger.mode);
    if (config.trigger.mode == hal::TriggerMode::kOff) {
        return api.set_enum_by_string(handle, node::kTriggerMode, "Off");
    }

    if (auto status = api.set_enum_by_string(handle, node::kTriggerMode, "On"); !status.ok()) {
        return status;
    }

    const std::string_view source = to_mvs_trigger_source(mode);
    if (source.empty()) {
        return Result<void>(make_error(ErrorCode::kInvalidArgument,
                                       "camera.trigger.mode 无法映射到 MVS TriggerSource"));
    }
    if (auto status = api.set_enum_by_string(handle, node::kTriggerSource, source);
        !status.ok()) {
        return status;
    }

    // 软触发没有物理线路，TriggerActivation / LineDebouncerTime 对设备无意义。
    if (mode == hal::TriggerMode::kSoftware) {
        return Result<void>();
    }

    if (auto status = api.set_enum_by_string(handle, node::kTriggerActivation,
                                             to_mvs_trigger_activation(config.trigger.activation));
        !status.ok()) {
        return status;
    }
    return set_optional_float(api, handle, node::kLineDebouncerTime,
                              config.trigger.debounce_us);
}

/// @brief GigE 调优与 PTP：现场网卡差异大，写失败只计数，不阻断 open。
void apply_optional_nodes(MvsApi& api, void* handle, const hal::CameraConfig& config,
                          std::uint32_t& optional_errors) {
    const auto attempt = [&optional_errors](Result<void> status) {
        if (!status.ok()) {
            ++optional_errors;
        }
    };

    // packet_size = 0 表示不改；否则用配置值，配置值由上层用 MV_CC_GetOptimalPacketSize 得来。
    if (config.packet_size != 0) {
        attempt(api.set_int(handle, node::kPacketSize, config.packet_size));
    }
    if (config.inter_packet_delay_us.has_value()) {
        attempt(api.set_int(handle, node::kPacketDelay,
                            static_cast<std::int64_t>(*config.inter_packet_delay_us)));
    }
    if (config.heartbeat_timeout_ms.has_value()) {
        attempt(api.set_int(handle, node::kHeartbeatTimeout,
                            static_cast<std::int64_t>(*config.heartbeat_timeout_ms)));
    }
    if (config.enable_ptp) {
        // USB3 相机没有 GevIEEE1588 节点，这里失败是预期行为。
        attempt(api.set_bool(handle, node::kPtpEnable, true));
    }
}

} // namespace

Result<void> apply_config(MvsApi& api, void* handle, const hal::CameraConfig& config,
                          std::uint32_t& optional_errors) {
    if (auto status = apply_geometry(api, handle, config); !status.ok()) {
        return status;
    }
    if (auto status = apply_frame_rate(api, handle, config); !status.ok()) {
        return status;
    }
    if (auto status = apply_exposure(api, handle, config); !status.ok()) {
        return status;
    }
    if (auto status = apply_gain(api, handle, config); !status.ok()) {
        return status;
    }
    if (auto status = apply_trigger(api, handle, config); !status.ok()) {
        return status;
    }

    apply_optional_nodes(api, handle, config, optional_errors);
    return Result<void>();
}

Result<void> apply_tuning(MvsApi& api, void* handle, const hal::CameraTuning& tuning) {
    if (tuning.frame_rate_hz.has_value()) {
        const auto& frame_rate = *tuning.frame_rate_hz;
        if (frame_rate.has_value()) {
            if (auto status = api.set_bool(handle, node::kFrameRateEnable, true); !status.ok()) {
                return status;
            }
            if (auto status = api.set_float(handle, node::kFrameRate, *frame_rate);
                !status.ok()) {
                return status;
            }
        } else {
            // 内层空 = 清除限速：等价于关掉帧率使能，让设备跑自由运行。
            if (auto status = api.set_bool(handle, node::kFrameRateEnable, false); !status.ok()) {
                return status;
            }
        }
    }

    if (tuning.exposure_auto.has_value()) {
        if (auto status = api.set_enum_by_string(handle, node::kExposureAuto,
                                                 to_mvs_auto_mode(*tuning.exposure_auto));
            !status.ok()) {
            return status;
        }
    }
    if (tuning.exposure_us.has_value()) {
        const auto& exposure = *tuning.exposure_us;
        if (exposure.has_value()) {
            if (auto status = api.set_float(handle, node::kExposureTime, *exposure);
                !status.ok()) {
                return status;
            }
        } else {
            // 内层空 = 交回自动曝光。
            if (auto status = api.set_enum_by_string(handle, node::kExposureAuto, "Continuous");
                !status.ok()) {
                return status;
            }
        }
    }
    if (auto status = set_optional_float(api, handle, node::kExposureLowerLimit,
                                         tuning.exposure_auto_lower_us.value_or(std::nullopt));
        !status.ok()) {
        return status;
    }
    if (auto status = set_optional_float(api, handle, node::kExposureUpperLimit,
                                         tuning.exposure_auto_upper_us.value_or(std::nullopt));
        !status.ok()) {
        return status;
    }

    if (tuning.gain_auto.has_value()) {
        if (auto status = api.set_enum_by_string(handle, node::kGainAuto,
                                                 to_mvs_auto_mode(*tuning.gain_auto));
            !status.ok()) {
            return status;
        }
    }
    if (tuning.gain_db.has_value()) {
        const auto& gain = *tuning.gain_db;
        if (gain.has_value()) {
            if (auto status = api.set_float(handle, node::kGain, *gain); !status.ok()) {
                return status;
            }
        } else {
            if (auto status = api.set_enum_by_string(handle, node::kGainAuto, "Continuous");
                !status.ok()) {
                return status;
            }
        }
    }
    if (auto status = set_optional_float(api, handle, node::kGainLowerLimit,
                                         tuning.gain_auto_lower_db.value_or(std::nullopt));
        !status.ok()) {
        return status;
    }
    if (auto status = set_optional_float(api, handle, node::kGainUpperLimit,
                                         tuning.gain_auto_upper_db.value_or(std::nullopt));
        !status.ok()) {
        return status;
    }

    if (tuning.trigger_debounce_us.has_value()) {
        const auto& debounce = *tuning.trigger_debounce_us;
        if (auto status =
                api.set_float(handle, node::kLineDebouncerTime, debounce.value_or(0.0));
            !status.ok()) {
            return status;
        }
    }

    // 时间戳相关字段只影响驱动侧换算，不写设备；由 CameraImpl 更新自身配置。
    return Result<void>();
}

bool query_ptp_slave(MvsApi& api, void* handle) noexcept {
    auto status = api.get_enum(handle, node::kPtpStatus);
    return status.ok() && status.value() == "Slave";
}

std::uint32_t query_tick_frequency(MvsApi& api, void* handle) noexcept {
    auto status = api.get_int(handle, node::kDeviceTicksFrequency);
    if (!status.ok() || status.value() <= 0) {
        return 0;
    }
    return static_cast<std::uint32_t>(status.value());
}

TimestampConfig make_timestamp_config(const hal::CameraConfig& config, bool ptp_slave,
                                      std::uint32_t tick_frequency_hz) noexcept {
    TimestampConfig result;
    result.source = config.timestamp_source;
    result.device_to_host_offset_ns = config.device_to_host_offset_ns.value_or(0);
    // 曝光延迟用 double 配置，换算到纳秒后按整型参与计算。
    result.exposure_to_host_latency_ns = static_cast<std::int64_t>(
        config.exposure_to_host_latency_ns.value_or(0.0));
    result.allow_unsynced_stamp = config.allow_unsynced_stamp;
    // PTP 只在显式开启时采纳，避免设备单方面进入 Slave 就打上 kPtp 标记。
    result.ptp_slave = config.enable_ptp && ptp_slave;
    result.tick_frequency_hz = tick_frequency_hz;
    return result;
}

} // namespace foray::platform::driver::hikvision::detail
