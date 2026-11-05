#pragma once

#include <cstdint>
#include <string>

namespace foray::platform {

/**
 * @brief 平台层通用错误码。
 *
 * 驱动负责把 MVS / Livox SDK 的原始返回码映射到这些错误码，
 * 原始数值保留在 `Error::vendor_code` 中用于诊断。
 */
enum class ErrorCode {
    kOk = 0,
    /// @brief 参数非法（空指针、空序列号、越界配置等）。
    kInvalidArgument,
    /// @brief 当前实现不支持该操作或格式。
    kNotSupported,
    /// @brief 编译期未启用对应 SDK（驱动 stub）。
    kSdkUnavailable,
    /// @brief SDK 返回错误。
    kSdkError,
    /// @brief 找不到匹配设备。
    kDeviceNotFound,
    /// @brief 匹配到多个设备，选择条件不唯一。
    kDeviceAmbiguous,
    /// @brief 无权限打开设备（udev / 占用等）。
    kPermissionDenied,
    /// @brief 设备未打开或未启动。
    kNotOpen,
    /// @brief 设备已经打开。
    kAlreadyOpen,
    /// @brief 设备忙。
    kBusy,
    /// @brief 操作超时。
    kTimeout,
    /// @brief 暂时没有数据。
    kNoData,
    /// @brief 设备断开。
    kDisconnected,
    /// @brief 队列已满。
    kQueueFull,
    /// @brief 帧或包超过缓冲区大小。
    kFrameTooLarge,
    /// @brief 像素格式或点数据类型不受支持。
    kUnsupportedFormat,
    /// @brief 内部错误。
    kInternal,
};

/// @brief 返回错误码名称，用于日志。
const char* to_string(ErrorCode code) noexcept;

/**
 * @brief 错误对象。
 *
 * `message` 用于人类阅读，不进入实时路径；
 * `vendor_code` 保存 SDK 原始返回码（MVS 为十六进制错误码，Livox 为 livox_status）。
 */
struct Error {
    ErrorCode code = ErrorCode::kInternal;
    std::string message;
    std::int32_t vendor_code = 0;

    bool ok() const noexcept {
        return code == ErrorCode::kOk;
    }
};

/// @brief 构造错误对象。
Error make_error(ErrorCode code, std::string message, std::int32_t vendor_code = 0);

} // namespace foray::platform
