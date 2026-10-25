#pragma once

#include <string>

#include "foray/platform/hal/types.hpp"

namespace foray::platform::hal {

/// @brief 设备静态信息。
struct SensorInfo {
    std::string vendor;
    std::string model;
    std::string serial;
    std::string user_defined_name;
    std::string firmware_version;
    Transport transport = Transport::kAny;
    std::string ip_address;
    std::string frame_id;
};

} // namespace foray::platform::hal
