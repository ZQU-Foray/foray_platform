#include "camera_factory.hpp"

#include "camera_impl.hpp"

namespace foray::platform::driver::hikvision::detail {

Result<std::unique_ptr<hal::Camera>> make_camera(const hal::CameraConfig& config,
                                                std::shared_ptr<MvsApi> api) {
    if (api == nullptr) {
        return Result<std::unique_ptr<hal::Camera>>(make_error(
            ErrorCode::kInternal, "make_camera() 需要非空的 MvsApi 实现"));
    }
    std::unique_ptr<hal::Camera> camera =
        std::make_unique<CameraImpl>(config, std::move(api));
    return Result<std::unique_ptr<hal::Camera>>(std::move(camera));
}

} // namespace foray::platform::driver::hikvision::detail
