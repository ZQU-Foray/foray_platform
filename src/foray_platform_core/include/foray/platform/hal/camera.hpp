#pragma once

#include <optional>
#include <string>
#include <vector>

#include "foray/platform/hal/config.hpp"
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
 *
 * 配置分三层入口，对应 `docs/camera_config.md` §4 的分级：
 * - `static` 项只能通过 `open()` 前的 `CameraConfig` 设置；
 * - `streaming` 项需要在 `stop()` / `start()` 之间重新应用；
 * - `live` 项通过 `apply(CameraTuning)` 在运行期修改。
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

    /**
     * @brief 运行期热改（`live` 级配置）。
     *
     * 未设置的字段保持不变；部分字段失败时返回首个错误，已成功的字段不回滚，
     * 但驱动必须记录失败项。默认实现返回 `kNotSupported`，由驱动覆写。
     *
     * 实现者需要自行串行化多次 `apply()` 调用（取流线程与写入可并发）。
     */
    virtual Result<void> apply(const CameraTuning& tuning) {
        static_cast<void>(tuning);
        return Result<void>::failure(
            make_error(ErrorCode::kNotSupported, "camera driver does not support runtime tuning"));
    }

    /**
     * @brief 软件触发一次。
     *
     * 仅在触发模式为 `TriggerMode::kSoftware` 时有效；其余情况返回 `kNotSupported`。
     * 这是动作而非配置，conf 中没有对应键。
     */
    virtual Result<void> trigger() {
        return Result<void>::failure(
            make_error(ErrorCode::kNotSupported, "camera driver does not support software trigger"));
    }

    /**
     * @brief 该相机实际支持的配置键集合。
     *
     * 由驱动在 `open()` 成功后填充，供配置期自检使用（`docs/camera_config.md` §5.2）。
     * 返回空表示驱动未声明，配置期不做逐键校验。
     */
    virtual std::vector<std::string> supported_keys() const {
        return {};
    }
};

} // namespace foray::platform::hal
