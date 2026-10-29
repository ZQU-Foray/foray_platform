#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "foray/platform/hal/camera.hpp"
#include "foray/platform/hal/config.hpp"
#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/frame_pool.hpp"
#include "foray/platform/hal/sensor_info.hpp"
#include "foray/platform/result/result.hpp"
#include "mvs_api.hpp"
#include "timestamp.hpp"

namespace foray::platform::driver::hikvision::detail {

/**
 * @brief `hal::Camera` 的 MVS 实现。
 *
 * 线程模型：
 * - `open/start/stop/close/recover/apply` 由控制线程调用，用 `control_mutex_` 串行化；
 * - 取流线程只由 `start()` 创建，只碰设备句柄与帧池，不持 `control_mutex_`；
 * - `read()` 由单个消费线程调用，只碰帧池；
 * - SDK 异常回调运行在 SDK 线程上，**只**置状态与计数。
 */
class CameraImpl final : public hal::Camera {
  public:
    CameraImpl(hal::CameraConfig config, std::shared_ptr<MvsApi> api);

    ~CameraImpl() override;

    CameraImpl(const CameraImpl&) = delete;
    CameraImpl& operator=(const CameraImpl&) = delete;

    Result<void> open() override;

    Result<void> start() override;

    Result<void> stop() override;

    Result<void> close() override;

    Result<void> recover() override;

    hal::DeviceState state() const noexcept override;

    hal::SensorInfo info() const override;

    hal::SensorStats stats() const noexcept override;

    Result<void> read(hal::FrameLease<hal::ImageFrame>& out, time::Duration timeout) override;

    Result<void> set_exposure(std::optional<double> microseconds) override;

    Result<void> set_gain(std::optional<double> db) override;

    Result<void> set_frame_rate(std::optional<double> hz) override;

    Result<void> apply(const hal::CameraTuning& tuning) override;

    Result<void> trigger() override;

    std::vector<std::string> supported_keys() const override;

  private:
    /// @brief SDK 异常回调；运行在 SDK 线程。
    static void on_exception(void* user, std::uint32_t code);

    Result<void> open_locked();

    Result<void> start_locked();

    Result<void> stop_locked();

    Result<void> close_locked();

    Result<void> recover_locked();

    /// @brief `open_locked()` 的设备配置阶段，成功后设备可开始取流。
    Result<void> configure_locked(const DeviceDescriptor& device);

    void release_resources_locked();

    /// @brief 取流线程主体。
    void grab_loop();

    /// @brief 把一帧拷进帧池并发布。
    void publish_frame(const FrameView& frame);

    std::shared_ptr<MvsApi> api_;
    hal::CameraConfig config_;

    void* handle_ = nullptr;

    std::shared_ptr<hal::FramePool<hal::ImageFrame>> pool_;

    std::thread grab_thread_;

    std::atomic<bool> stop_requested_{false};

    std::atomic<hal::DeviceState> state_{hal::DeviceState::kClosed};

    /// @brief 串行化控制接口与 `apply()`；取流线程不持有它。
    std::mutex control_mutex_;

    /// @brief 保护 `timestamp_config_` / `frame_id_` / `exposure_us_` 等会被热改的字段。
    std::mutex tuning_mutex_;

    TimestampConfig timestamp_config_;

    std::string frame_id_;

    /// @brief 设备当前曝光/增益的最近一次已知值（µs / dB）。
    ///
    /// `MV_FRAME_OUT_INFO_EX::fExposureTime`/`fGain` 只在开启 chunk 时才有内容，
    /// 默认恒为 0；取流线程用这两个字段兜底，让上层拿到「设备当前配置」而非 0。
    float exposure_hint_us_ = 0.0F;

    float gain_hint_db_ = 0.0F;

    hal::SensorInfo info_;

    std::atomic<std::uint32_t> consecutive_errors_{0};

    std::atomic<std::uint64_t> frames_received_{0};

    std::atomic<std::uint64_t> frames_delivered_{0};

    std::atomic<std::uint64_t> frames_dropped_{0};

    std::atomic<std::uint64_t> bytes_received_{0};

    std::atomic<std::uint64_t> timeouts_{0};

    std::atomic<std::uint64_t> sdk_errors_{0};

    std::atomic<std::uint64_t> reconnects_{0};

    std::atomic<std::uint64_t> last_sequence_{0};

    std::atomic<std::int32_t> last_vendor_code_{0};

    std::atomic<std::int64_t> last_frame_stamp_ns_{0};

    std::atomic<std::int64_t> last_receive_stamp_ns_{0};

    /// @brief 发布序号与设备帧号；只在取流线程内递增。
    std::uint64_t sequence_ = 0;

    std::uint32_t last_device_frame_number_ = 0;
};

} // namespace foray::platform::driver::hikvision::detail
