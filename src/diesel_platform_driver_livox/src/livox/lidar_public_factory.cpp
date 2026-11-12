#include "diesel/platform/driver/livox/lidar.hpp"

#include <memory>

#include "livox/lidar_factory.hpp"

namespace diesel::platform::driver::livox {

Result<std::unique_ptr<hal::Lidar>> make_lidar(const hal::LidarConfig& config) {
    return detail::make_lidar(config, detail::make_livox_api());
}

} // namespace diesel::platform::driver::livox
