#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "livox/livox_api.hpp"

namespace foray::platform::driver::livox::detail {

/// @brief 控制面的重试策略；单次 SDK 调用的等待上限在真实实现里（`kControlTimeout`）。
struct ControlPolicy {
    /// @brief 总尝试次数（含首次）；<=0 视为 1。
    int max_attempts = 3;
    /// @brief 两次尝试之间的退避。
    std::chrono::milliseconds retry_delay{100};
};

/**
 * @brief 雷达参数下发的门面：统一重试 + 写后读回校验。
 *
 * 只依赖 `LivoxApi`，因此可用 mock 覆盖「超时 → 重试 → 成功」与「读回校验」。
 * 下发失败时按 `ControlPolicy` 重试；`kSdkUnavailable`/`kInvalidArgument` 不重试。
 */
class LivoxControl {
  public:
    explicit LivoxControl(LivoxApi& api, ControlPolicy policy = {});

    Result<void> set_lidar_ip(std::uint32_t handle, const std::string& ip,
                              const std::string& net_mask, const std::string& gateway);
    Result<void> set_install_attitude(std::uint32_t handle, float roll_deg, float pitch_deg,
                                      float yaw_deg, std::int32_t x_mm, std::int32_t y_mm,
                                      std::int32_t z_mm);
    Result<void> set_pps_sync_mode(std::uint32_t handle, std::uint8_t mode);
    Result<void> set_host_port_cfg(std::uint32_t handle, const std::string& host_ip,
                                   std::uint16_t host_port, std::uint16_t lidar_port);

    /// @brief 读回 IP 并与期望值比对；读回为空时返回 `kNoData`。
    Result<void> verify_lidar_ip(std::uint32_t handle, const std::string& expected_ip);

  private:
    Result<void> retry(const char* operation, const std::function<Result<void>()>& attempt) const;

    LivoxApi& api_;
    ControlPolicy policy_;
};

} // namespace foray::platform::driver::livox::detail
