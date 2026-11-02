#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "foray/platform/container/spsc_queue.hpp"
#include "foray/platform/hal/config.hpp"
#include "foray/platform/hal/frame_pool.hpp"
#include "foray/platform/hal/lidar.hpp"
#include "foray/platform/hal/sensor_info.hpp"
#include "foray/platform/hal/sensor_stats.hpp"
#include "livox/frame_assembler.hpp"
#include "livox/livox_api.hpp"
#include "livox/livox_runtime.hpp"
#include "livox/packet_parser.hpp"

namespace foray::platform::driver::livox::detail {

/**
 * @brief `hal::Lidar` 的 Livox SDK2 实现。
 *
 * 线程布局：
 * - SDK 回调线程：`CallbackSink::push()`，只做长度校验、时间戳与一次拷贝；
 * - 解析线程：`parse_loop()`，解析包、组帧并写入 `FramePool`；
 * - 消费线程：`read()` / `read_imu()`，每实例只允许一个消费者线程；
 * - 控制线程：`open/start/stop/close/recover`。
 */
class LidarImpl final : public hal::Lidar {
  public:
    LidarImpl(const hal::LidarConfig& config, std::shared_ptr<LivoxApi> api);
    ~LidarImpl() override;

    LidarImpl(const LidarImpl&) = delete;
    LidarImpl& operator=(const LidarImpl&) = delete;

    Result<void> open() override;
    Result<void> start() override;
    Result<void> stop() override;
    Result<void> close() override;
    Result<void> recover() override;

    hal::DeviceState state() const noexcept override;
    hal::SensorInfo info() const override;
    hal::SensorStats stats() const noexcept override;

    Result<void> read(hal::FrameLease<hal::PointCloudFrame>& out, time::Duration timeout) override;
    Result<void> read_imu(hal::ImuSample& out, time::Duration timeout) override;

  private:
    Result<void> open_locked();
    Result<void> stop_locked();
    Result<void> close_locked();
    Result<void> prepare_recover_locked();

    Result<void> configure_locked();
    /// @brief 启动准备：校验状态与资源，取一份 `sink_` 快照留给锁外的设备上线等待。
    ///
    /// 已在 `kStreaming` 时返回成功且 @p sink 为空，调用方直接收工。
    Result<void> prepare_start_locked(std::shared_ptr<CallbackSink>& sink);
    /// @brief 启动收尾：设备已上线，绑定 handle、下发配置并拉起解析线程。
    ///
    /// 由调用方持 `control_mutex_` 调用。@p sink 必须仍是当前的 `sink_`，
    /// 否则说明等待期间 `close()` 已经把资源换掉了（审计 H22）。
    Result<void> finish_start_locked(const std::shared_ptr<CallbackSink>& sink);
    /// @brief 等设备上报信息；**不持 `control_mutex_`**，因此最长 5 s 的等待不会卡住控制调用。
    bool wait_for_info(const std::shared_ptr<CallbackSink>& sink);
    void update_sensor_info_locked();

    void parse_loop();
    void handle_packet(const PacketSlot& slot, FrameAssembler& assembler);
    void drain_queues_locked() noexcept;
    void release_resources_locked() noexcept;

    std::uint8_t data_type_value() const noexcept;
    std::uint8_t scan_pattern_value() const noexcept;

    hal::LidarConfig config_;
    std::shared_ptr<LivoxApi> api_;
    LivoxRuntime* runtime_ = &LivoxRuntime::instance();

    std::shared_ptr<CallbackSink> sink_;
    std::shared_ptr<container::SpscQueue<PacketSlot>> queue_;
    std::shared_ptr<container::SpscQueue<hal::ImuSample>> imu_queue_;
    std::shared_ptr<hal::FramePool<hal::PointCloudFrame>> pool_;

    std::thread parse_thread_;
    std::atomic<bool> stop_{false};
    std::atomic<hal::DeviceState> state_{hal::DeviceState::kClosed};
    std::atomic<std::uint32_t> handle_{0};
    std::uint8_t line_num_ = 1;
    bool runtime_acquired_ = false;

    mutable std::mutex control_mutex_;
    /// @brief 控制指令代次。每次公共控制调用（open/start/stop/close/recover）自增。
    ///
    /// `recover()` 把退避与设备上线等待搬到锁外，中途要有办法知道"别人动过状态机"，
    /// 否则用户在这段时间 `close()` 之后，恢复线程还会自作主张把设备重新打开
    /// （审计 H22）。只由持锁的控制调用修改，读时也在锁内。
    std::uint64_t control_generation_ = 0;
    mutable std::mutex info_mutex_;
    hal::SensorInfo info_;

    std::atomic<std::uint64_t> frames_received_{0};
    std::atomic<std::uint64_t> frames_delivered_{0};
    std::atomic<std::uint64_t> frames_dropped_{0};
    std::atomic<std::uint64_t> imu_dropped_{0};
    /// @brief 由解析线程按 `udp_cnt` 缺口镜像过来的网络丢包数（审计 H23）。
    std::atomic<std::uint64_t> udp_cnt_gaps_{0};
    /// @brief 同上，`udp_cnt` 回退/大跳的次数（不算丢包）。
    std::atomic<std::uint64_t> udp_cnt_anomalies_{0};
    std::atomic<std::uint64_t> timeouts_{0};
    std::atomic<std::uint64_t> sdk_errors_{0};
    std::atomic<std::uint64_t> reconnects_{0};
    std::atomic<std::uint64_t> last_sequence_{0};
    std::atomic<std::int64_t> last_frame_stamp_ns_{0};
    std::atomic<std::int64_t> last_receive_stamp_ns_{0};
    std::atomic<std::int32_t> last_vendor_code_{0};

    std::uint64_t imu_sequence_ = 0;
};

} // namespace foray::platform::driver::livox::detail
