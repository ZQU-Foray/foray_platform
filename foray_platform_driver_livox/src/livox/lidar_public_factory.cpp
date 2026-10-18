#include "foray/platform/driver/livox/lidar.hpp"

#include <memory>

#include "livox/lidar_factory.hpp"

namespace foray::platform::driver::livox {

Result<std::unique_ptr<hal::Lidar>> make_lidar(const hal::LidarConfig& config) {
    return detail::make_lidar(config, detail::make_livox_api());
}

} // namespace foray::platform::driver::livox
