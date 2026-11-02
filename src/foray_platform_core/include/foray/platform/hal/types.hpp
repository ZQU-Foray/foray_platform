#pragma once

#include <cstdint>

namespace foray::platform::hal {

/// @brief 设备生命周期状态。
enum class DeviceState {
    kClosed,
    kOpening,
    /// @brief 已打开、尚未开始取流。
    kOpen,
    kStreaming,
    kStopping,
    kError,
    kRecovering,
};

/// @brief 相机/设备传输层。
enum class Transport {
    kAny,
    kGigE,
    kUsb3,
};

/// @brief 像素格式。
///
/// 首期驱动只保证 `Mono8` 与 `Bayer*8`；其余格式枚举保留给后续扩展。
enum class PixelFormat {
    kMono8,
    kBayerRG8,
    kBayerGR8,
    kBayerGB8,
    kBayerBG8,
    kMono10,
    kMono12,
    kBayerRG10,
    kBayerRG12,
    kRgb8,
    kBgr8,
    kYuv422,
};

/// @brief 设备时间与主机时间的同步状态。
enum class TimestampSync {
    kUnknown,
    kNoSync,
    kHostReceive,
    kDeviceCalibrated,
    kPps,
    kPtp,
    kGpsPps,
};

/// @brief 触发源。
enum class TriggerMode {
    kOff,
    kSoftware,
    kLine0,
    kLine1,
    kLine2,
    kLine3,
};

/// @brief 触发沿/电平。
enum class TriggerActivation {
    kRisingEdge,
    kFallingEdge,
    kLevelHigh,
    kLevelLow,
};

/// @brief 自动调节模式，对应 GenICam 的 `Off` / `Once` / `Continuous`。
enum class AutoMode {
    kOff,
    /// @brief 下一次采集生效后设备自动回到 `kOff`。
    kOnce,
    kContinuous,
};

/// @brief 时间戳来源策略。
///
/// 与 `docs/driver_hikvision.md` §7 的三条路径对应：
/// - `kAuto`：按 PTP → 标定偏移 → 主机接收 的优先级推导；
/// - `kDevice`：强制使用设备计数换算，时钟不可用时置 `kNoSync`、不回退主机时刻；
/// - `kHostReceive`：强制使用主机接收时刻。
enum class TimestampSource {
    kAuto,
    kDevice,
    kHostReceive,
};

constexpr const char* to_string(DeviceState state) noexcept {
    switch (state) {
    case DeviceState::kClosed:
        return "closed";
    case DeviceState::kOpening:
        return "opening";
    case DeviceState::kOpen:
        return "open";
    case DeviceState::kStreaming:
        return "streaming";
    case DeviceState::kStopping:
        return "stopping";
    case DeviceState::kError:
        return "error";
    case DeviceState::kRecovering:
        return "recovering";
    }
    return "unknown";
}

constexpr const char* to_string(Transport transport) noexcept {
    switch (transport) {
    case Transport::kAny:
        return "any";
    case Transport::kGigE:
        return "gige";
    case Transport::kUsb3:
        return "usb3";
    }
    return "unknown";
}

constexpr const char* to_string(PixelFormat format) noexcept {
    switch (format) {
    case PixelFormat::kMono8:
        return "mono8";
    case PixelFormat::kBayerRG8:
        return "bayer_rgg8";
    case PixelFormat::kBayerGR8:
        return "bayer_grbg8";
    case PixelFormat::kBayerGB8:
        return "bayer_gbrg8";
    case PixelFormat::kBayerBG8:
        return "bayer_bggr8";
    case PixelFormat::kMono10:
        return "mono10";
    case PixelFormat::kMono12:
        return "mono12";
    case PixelFormat::kBayerRG10:
        return "bayer_rgg10";
    case PixelFormat::kBayerRG12:
        return "bayer_rgg12";
    case PixelFormat::kRgb8:
        return "rgb8";
    case PixelFormat::kBgr8:
        return "bgr8";
    case PixelFormat::kYuv422:
        return "yuv422";
    }
    return "unknown";
}

constexpr const char* to_string(TimestampSync sync) noexcept {
    switch (sync) {
    case TimestampSync::kUnknown:
        return "unknown";
    case TimestampSync::kNoSync:
        return "no_sync";
    case TimestampSync::kHostReceive:
        return "host_receive";
    case TimestampSync::kDeviceCalibrated:
        return "device_calibrated";
    case TimestampSync::kPps:
        return "pps";
    case TimestampSync::kPtp:
        return "ptp";
    case TimestampSync::kGpsPps:
        return "gps_pps";
    }
    return "unknown";
}

constexpr const char* to_string(TriggerMode mode) noexcept {
    switch (mode) {
    case TriggerMode::kOff:
        return "off";
    case TriggerMode::kSoftware:
        return "software";
    case TriggerMode::kLine0:
        return "line0";
    case TriggerMode::kLine1:
        return "line1";
    case TriggerMode::kLine2:
        return "line2";
    case TriggerMode::kLine3:
        return "line3";
    }
    return "unknown";
}

constexpr const char* to_string(TriggerActivation activation) noexcept {
    switch (activation) {
    case TriggerActivation::kRisingEdge:
        return "rising_edge";
    case TriggerActivation::kFallingEdge:
        return "falling_edge";
    case TriggerActivation::kLevelHigh:
        return "level_high";
    case TriggerActivation::kLevelLow:
        return "level_low";
    }
    return "unknown";
}

constexpr const char* to_string(AutoMode mode) noexcept {
    switch (mode) {
    case AutoMode::kOff:
        return "off";
    case AutoMode::kOnce:
        return "once";
    case AutoMode::kContinuous:
        return "continuous";
    }
    return "unknown";
}

constexpr const char* to_string(TimestampSource source) noexcept {
    switch (source) {
    case TimestampSource::kAuto:
        return "auto";
    case TimestampSource::kDevice:
        return "device";
    case TimestampSource::kHostReceive:
        return "host_receive";
    }
    return "unknown";
}

/// @brief 每像素字节数；未知/打包格式返回 0。
constexpr std::uint32_t bytes_per_pixel(PixelFormat format) noexcept {
    switch (format) {
    case PixelFormat::kMono8:
    case PixelFormat::kBayerRG8:
    case PixelFormat::kBayerGR8:
    case PixelFormat::kBayerGB8:
    case PixelFormat::kBayerBG8:
        return 1;
    case PixelFormat::kMono10:
    case PixelFormat::kMono12:
    case PixelFormat::kBayerRG10:
    case PixelFormat::kBayerRG12:
    case PixelFormat::kYuv422:
        return 2;
    case PixelFormat::kRgb8:
    case PixelFormat::kBgr8:
        return 3;
    }
    return 0;
}

/// @brief 首期驱动支持的格式：8 位 Mono/Bayer。
constexpr bool is_supported_pixel_format(PixelFormat format) noexcept {
    switch (format) {
    case PixelFormat::kMono8:
    case PixelFormat::kBayerRG8:
    case PixelFormat::kBayerGR8:
    case PixelFormat::kBayerGB8:
    case PixelFormat::kBayerBG8:
        return true;
    default:
        return false;
    }
}

} // namespace foray::platform::hal
