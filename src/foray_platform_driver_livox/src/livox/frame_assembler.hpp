#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>

#include "foray/platform/hal/frame_pool.hpp"
#include "foray/platform/time/timestamp.hpp"
#include "livox/packet_parser.hpp"

namespace foray::platform::driver::livox::detail {

/**
 * @brief 把同一帧的多个 UDP 包组装成 `PointCloudFrame`。
 *
 * 线程模型：只允许解析线程调用 `on_packet()` / `publish_if_idle()` / `flush()`；
 * 统计字段为原子量，可被其他线程读取。
 */
class FrameAssembler {
  public:
    FrameAssembler(std::shared_ptr<hal::FramePool<hal::PointCloudFrame>> pool,
                   std::uint32_t point_capacity, std::uint8_t line_num, std::string_view frame_id,
                   std::uint32_t frame_duration_ms = 100);

    FrameAssembler(const FrameAssembler&) = delete;
    FrameAssembler& operator=(const FrameAssembler&) = delete;

    /**
     * @brief 处理一个点云包。
     * @return 是否因 `frame_cnt` 变化而发布了一帧。
     */
    bool on_packet(const PacketHeader& header, const std::uint8_t* packet, std::size_t size,
                   std::uint64_t receive_steady_ns);

    /// @brief 超过组帧超时且当前帧非空时发布；返回是否发布。
    bool publish_if_idle(std::uint64_t now_steady_ns, std::int64_t timeout_ns);

    /// @brief 强制发布当前帧（停机/析构路径）；返回是否发布。
    bool flush();

    bool has_frame() const noexcept {
        return writer_.has_value();
    }

    std::size_t point_count() const noexcept {
        return count_;
    }

    std::uint64_t frames_published() const noexcept {
        return frames_published_.load(std::memory_order_relaxed);
    }

    std::uint64_t dropped_frames() const noexcept {
        return dropped_frames_.load(std::memory_order_relaxed);
    }

    std::uint64_t points_dropped() const noexcept {
        return points_dropped_.load(std::memory_order_relaxed);
    }

  private:
    bool begin_frame(const PacketHeader& header, std::uint64_t receive_steady_ns);
    bool frame_expired(const PacketHeader& header, std::uint64_t receive_steady_ns) const noexcept;
    bool publish();
    void discard();

    hal::PointXYZITL* points() noexcept;

    std::shared_ptr<hal::FramePool<hal::PointCloudFrame>> pool_;
    std::size_t capacity_ = 0;
    std::uint8_t line_num_ = 1;
    std::string_view frame_id_;
    /// @brief 时间组帧窗口；非重复扫描下 frame_cnt 无效时必须按时间切帧。
    std::int64_t frame_duration_ns_ = 100'000'000;

    std::optional<hal::FramePool<hal::PointCloudFrame>::Writer> writer_;
    std::uint8_t current_frame_cnt_ = 0;
    std::int64_t base_device_ns_ = 0;
    std::uint64_t base_receive_steady_ns_ = 0;
    hal::TimestampSync sync_ = hal::TimestampSync::kUnknown;
    std::size_t count_ = 0;
    std::uint64_t last_packet_steady_ns_ = 0;
    std::uint32_t frame_dropped_packets_ = 0;
    bool frame_overflow_ = false;
    bool dropping_ = false;
    std::uint64_t sequence_ = 0;

    std::atomic<std::uint64_t> frames_published_{0};
    std::atomic<std::uint64_t> dropped_frames_{0};
    std::atomic<std::uint64_t> points_dropped_{0};
};

} // namespace foray::platform::driver::livox::detail
