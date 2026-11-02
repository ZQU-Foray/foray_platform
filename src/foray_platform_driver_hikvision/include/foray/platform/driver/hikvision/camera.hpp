#pragma once

#include <memory>

#include "foray/platform/hal/camera.hpp"
#include "foray/platform/hal/config.hpp"
#include "foray/platform/result/result.hpp"

namespace foray::platform::driver::hikvision {

/**
 * @brief 创建一个 Hikvision MVS 相机。
 *
 * 返回的 `hal::Camera` 在 `open()` 时才枚举、建句柄、打开设备并下发配置，构造本身
 * 不做任何 I/O。编译期未启用 MVS（`FORAY_BUILD_DRIVER_HIKVISION=OFF`）时，所有
 * 操作返回 `ErrorCode::kSdkUnavailable`。
 *
 * 设备选择遵循 HAL 约定，**不会**默认取第一个设备：
 * - `selector.serial` 精确匹配；
 * - `selector.user_defined_name` 要求唯一；
 * - `selector.model` 命中多个时必须再给 `serial`，或用 `index` 显式指定；
 * - `selector.ip_address` 只适用于 GigE；
 * - 全空返回 `kInvalidArgument`。
 *
 * @param config HAL 相机配置；`width` / `height` 为 0 表示沿用设备当前值。
 */
Result<std::unique_ptr<hal::Camera>> make_camera(const hal::CameraConfig& config);

} // namespace foray::platform::driver::hikvision
