#include "livox/livox_control.hpp"

#include <thread>
#include <utility>

namespace foray::platform::driver::livox::detail {
namespace {

/// @brief 这些错误重试没有意义：环境/参数本身有问题。
bool retryable(const Error& error) {
    switch (error.code) {
        case ErrorCode::kTimeout:
        case ErrorCode::kSdkError:
        case ErrorCode::kBusy:
        case ErrorCode::kNoData:
            return true;
        default:
            return false;
    }
}

} // namespace

LivoxControl::LivoxControl(LivoxApi& api, ControlPolicy policy)
    : api_(api), policy_(policy) {
    if (policy_.max_attempts <= 0) {
        policy_.max_attempts = 1;
    }
}

Result<void> LivoxControl::retry(const char* operation,
                                 const std::function<Result<void>()>& attempt) const {
    Result<void> last = make_error(ErrorCode::kInternal, std::string(operation) + " 未执行");
    for (int i = 0; i < policy_.max_attempts; ++i) {
        last = attempt();
        if (last.ok()) {
            return last;
        }
        if (!retryable(last.error())) {
            break;
        }
        if (i + 1 < policy_.max_attempts) {
            std::this_thread::sleep_for(policy_.retry_delay);
        }
    }
    return last;
}

Result<void> LivoxControl::set_lidar_ip(std::uint32_t handle, const std::string& ip,
                                        const std::string& net_mask,
                                        const std::string& gateway) {
    return retry("set_lidar_ip", [&] { return api_.set_lidar_ip(handle, ip, net_mask, gateway); });
}

Result<void> LivoxControl::set_install_attitude(std::uint32_t handle, float roll_deg,
                                                float pitch_deg, float yaw_deg, std::int32_t x_mm,
                                                std::int32_t y_mm, std::int32_t z_mm) {
    return retry("set_install_attitude", [&] {
        return api_.set_install_attitude(handle, roll_deg, pitch_deg, yaw_deg, x_mm, y_mm, z_mm);
    });
}

Result<void> LivoxControl::set_pps_sync_mode(std::uint32_t handle, std::uint8_t mode) {
    return retry("set_pps_sync_mode", [&] { return api_.set_pps_sync_mode(handle, mode); });
}

Result<void> LivoxControl::set_host_port_cfg(std::uint32_t handle, const std::string& host_ip,
                                             std::uint16_t host_port, std::uint16_t lidar_port) {
    return retry("set_host_port_cfg",
                 [&] { return api_.set_host_port_cfg(handle, host_ip, host_port, lidar_port); });
}

Result<void> LivoxControl::verify_lidar_ip(std::uint32_t handle, const std::string& expected_ip) {
    const auto info = api_.query_internal_info(handle);
    if (!info.ok()) {
        return Result<void>(info.error());
    }
    if (info.value().ip_address.empty()) {
        return make_error(ErrorCode::kNoData, "雷达没有回报 IP 配置");
    }
    if (info.value().ip_address != expected_ip) {
        return make_error(ErrorCode::kSdkError,
                          "雷达 IP 与期望不一致: " + info.value().ip_address + " != " + expected_ip);
    }
    return Result<void>();
}

} // namespace foray::platform::driver::livox::detail
