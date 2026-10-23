#pragma once

#include <memory>

#include "foray/platform/hal/config.hpp"
#include "foray/platform/hal/lidar.hpp"
#include "foray/platform/result/result.hpp"

namespace foray::platform::driver::livox {

/**
 * @brief 创建一个 Livox SDK2 雷达。
 *
 * 返回的 `hal::Lidar` 在 `open()` 时才初始化进程级 SDK；多雷达共享同一运行时。
 * 编译期未启用 SDK 时，`open()` 返回 `ErrorCode::kSdkUnavailable`。
 *
 * @param config HAL 雷达配置；必须提供 `selector.serial` 或 `selector.ip_address`，
 *               以及 `host_ip`（除非提供了 `config_json_path`）。
 */
Result<std::unique_ptr<hal::Lidar>> make_lidar(const hal::LidarConfig& config);

} // namespace foray::platform::driver::livox
