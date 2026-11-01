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
 *
 * 帧池句柄规则（审计 H8）：`pool_` 由控制线程创建/释放，而 `read()` 在消费线程上
 * 取值，两者**只能**通过 `frame_pool_snapshot()` / `set_frame_pool()` 访问，
 * 由 `pool_mutex_` 串行化。取流线程在 start/stop 之间独占访问 `pool_`
 * （`stop_locked()` 先 join 取流线程再释放池），因此不必加锁。
 *
 * 帧标识规则（审计 H9）：`frame_id_` 只在 `open_locked()` 里赋值一次，之后不可变；
 * `apply()` 收到 `frame_id` 直接拒绝。已交付租约里的 `frame_id` 是它的
 * `std::string_view`，这样做才能保证视图在整个租约期间有效。`info_` 由
 * `info_mutex_` 保护，因为它可能被控制线程之外的线程读取。
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

    /// @brief 锁定并拷出帧池句柄；`read()` 用它把池的生命周期延长到本次调用结束。
    std::shared_ptr<hal::FramePool<hal::ImageFrame>> frame_pool_snapshot() const;

    /// @brief 换出帧池句柄（创建或释放）；与 `frame_pool_snapshot()` 共用 `pool_mutex_`。
    void set_frame_pool(std::shared_ptr<hal::FramePool<hal::ImageFrame>> pool);

    /// @brief 取流线程主体。
    void grab_loop();

    /// @brief 把一帧拷进帧池并发布。
    void publish_frame(const FrameView& frame);

    std::shared_ptr<MvsApi> api_;
    hal::CameraConfig config_;

    void* handle_ = nullptr;

    std::shared_ptr<hal::FramePool<hal::ImageFrame>> pool_;

    /// @brief 保护 `pool_` 句柄本身；不保护帧池内部状态（那是无锁单写单读）。
    mutable std::mutex pool_mutex_;

    /// @brief 帧池槽位按什么分辨率分配的（像素）；0 表示当前没有池。
    ///
    /// 只在 `open_locked()` 建池时写、`release_resources_locked()` 清空，二者都发生在
    /// 取流线程不存在的时候；取流线程只读。`publish_frame()` 用它校验帧元数据里的
    /// width/height——不一致说明设备换了几何，整帧必须拒绝（审计 H14）。
    std::uint32_t pool_width_ = 0;

    std::uint32_t pool_height_ = 0;

    std::thread grab_thread_;

    std::atomic<bool> stop_requested_{false};

    std::atomic<hal::DeviceState> state_{hal::DeviceState::kClosed};

    /// @brief 串行化控制接口与 `apply()`；取流线程不持有它。
    std::mutex control_mutex_;

    /// @brief 保护 `timestamp_config_` 与曝光/增益 hint 等会被 `apply()` 热改的字段。
    std::mutex tuning_mutex_;

    TimestampConfig timestamp_config_;

    /// @brief 帧标识；打开期由 `config_.frame_id` 赋值，之后**不再改变**（审计 H9）。
    ///
    /// 已交付的 `ImageFrame::frame_id` 是指向这份存储的 `std::string_view`，
    /// 存储生命周期短于租约就是读已释放内存。因此 `apply()` 拒绝 `frame_id`，
    /// 该成员只在 `open_locked()` 里写一次（此时取流线程尚未创建）。
    std::string frame_id_;

    /// @brief 设备当前曝光/增益的最近一次已知值（µs / dB）。
    ///
    /// `MV_FRAME_OUT_INFO_EX::fExposureTime`/`fGain` 只在开启 chunk 时才有内容，
    /// 默认恒为 0；取流线程用这两个字段兜底，让上层拿到「设备当前配置」而非 0。
    float exposure_hint_us_ = 0.0F;

    float gain_hint_db_ = 0.0F;

    hal::SensorInfo info_;

    /// @brief 保护 `info_`；`info()` 可能在任何线程被调用（审计 H9）。
    mutable std::mutex info_mutex_;

    std::atomic<std::uint32_t> consecutive_errors_{0};

    /// @brief 「没数据」开始的时刻（steady ns）；0 表示当前有数据在流。
    ///
    /// 只由取流线程读写，不需要原子；用于判定自由运行下的取流停摆（审计 H13）。
    std::int64_t starve_since_ns_ = 0;

    /// @brief 当前配置是否在等外部触发；触发模式下「没数据」是正常状态。
    ///
    /// 由控制线程在 `start_locked()` 里按 `config_.trigger.mode` 写入，
    /// 取流线程只读——触发方式是 static 级配置，运行期不会热改。
    std::atomic<bool> wait_for_trigger_{false};

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
