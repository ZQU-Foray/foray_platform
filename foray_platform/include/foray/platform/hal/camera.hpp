#pragma once

#include <optional>

#include "foray/platform/hal/device.hpp"
#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/frame_lease.hpp"
#include "foray/platform/time/duration.hpp"

namespace foray::platform::hal {

/**
 * @brief 相机抽象。
 *
 * `read()` 返回的租约指向 HAL 帧池；调用方持有期间驱动不会覆盖该帧。
 * 每个相机只允许**一个消费者线程**调用 `read()`。
 */
class Camera : public Device {
  public:
    /**
     * @brief 读取一帧。
     * @param out 成功时写入帧租约。
     * @param timeout 超时；超时返回 `ErrorCode::kTimeout`。
     */
    virtual Result<void> read(FrameLease<ImageFrame>& out, time::Duration timeout) = 0;

    /// @brief 设置曝光；`std::nullopt` 表示恢复相机自动/当前值。
    virtual Result<void> set_exposure(std::optional<double> microseconds) = 0;

    /// @brief 设置增益；`std::nullopt` 表示恢复相机自动/当前值。
    virtual Result<void> set_gain(std::optional<double> db) = 0;

    /// @brief 设置自由运行帧率；触发模式下忽略。
    virtual Result<void> set_frame_rate(std::optional<double> hz) = 0;
};

} // namespace foray::platform::hal
