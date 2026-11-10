#include "foray/platform/driver/hikvision/camera.hpp"

#include "camera_factory.hpp"

namespace foray::platform::driver::hikvision {

Result<std::unique_ptr<hal::Camera>> make_camera(const hal::CameraConfig& config) {
    return detail::make_camera(config, detail::make_mvs_api());
}

} // namespace foray::platform::driver::hikvision
