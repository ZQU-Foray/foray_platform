#pragma once

#include <memory>

#include "foray/platform/driver/hikvision/camera.hpp"
#include "mvs_api.hpp"

namespace foray::platform::driver::hikvision::detail {

/**
 * @brief 注入式工厂：测试用 mock `MvsApi` 替换真实后端。
 *
 * 公共 `make_camera()` 只是它的 `make_mvs_api()` 包装，便于单测完全绕过 SDK。
 */
Result<std::unique_ptr<hal::Camera>> make_camera(const hal::CameraConfig& config,
                                                std::shared_ptr<MvsApi> api);

} // namespace foray::platform::driver::hikvision::detail
