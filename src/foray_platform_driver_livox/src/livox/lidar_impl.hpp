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
    Result<void> start_locked();
    Result<void> stop_locked();
    Result<void> close_locked();
    Result<void> recover_locked();

    Result<void> configure_locked();
    bool wait_for_info_locked();
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
    mutable std::mutex info_mutex_;
    hal::SensorInfo info_;

    std::atomic<std::uint64_t> frames_received_{0};
    std::atomic<std::uint64_t> frames_delivered_{0};
    std::atomic<std::uint64_t> frames_dropped_{0};
    std::atomic<std::uint64_t> imu_dropped_{0};
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
