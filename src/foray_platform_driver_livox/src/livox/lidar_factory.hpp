#pragma once

#include <memory>

#include "foray/platform/hal/config.hpp"
#include "foray/platform/hal/lidar.hpp"
#include "foray/platform/result/result.hpp"
#include "livox/livox_api.hpp"

namespace foray::platform::driver::livox::detail {

/// @brief 测试用工厂：注入自定义 SDK 函数表（mock / 真实实现）。
Result<std::unique_ptr<hal::Lidar>> make_lidar(const hal::LidarConfig& config,
                                               std::shared_ptr<LivoxApi> api);

} // namespace foray::platform::driver::livox::detail
