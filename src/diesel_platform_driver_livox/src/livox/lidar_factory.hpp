#pragma once

#include <memory>

#include "diesel/platform/hal/config.hpp"
#include "diesel/platform/hal/lidar.hpp"
#include "diesel/platform/result/result.hpp"
#include "livox/livox_api.hpp"

namespace diesel::platform::driver::livox::detail {

/// @brief 测试用工厂：注入自定义 SDK 函数表（mock / 真实实现）。
Result<std::unique_ptr<hal::Lidar>> make_lidar(const hal::LidarConfig& config,
                                               std::shared_ptr<LivoxApi> api);

} // namespace diesel::platform::driver::livox::detail
