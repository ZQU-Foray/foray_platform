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
 * 帧边界按机型分派（`frame_cnt_represents_frame()`）：
 * - 重复扫描机型：`frame_cnt` 变化 / `udp_cnt` 回零 / 时间窗到期都切帧；
 * - Mid-360 系列（非重复扫描）：只按时间窗切帧，`frame_cnt` 与 `udp_cnt` 变化只计入
 *   `frame_cnt_discontinuities()` 作一致性观测，绝不切帧。
 *
 * 线程模型：只允许解析线程调用 `on_packet()` / `publish_if_idle()` / `flush()`；
 * 统计字段为原子量，可被其他线程读取。
 */
class FrameAssembler {
  public:
    FrameAssembler(std::shared_ptr<hal::FramePool<hal::PointCloudFrame>> pool,
                   std::uint32_t point_capacity, std::uint8_t line_num, std::uint8_t dev_type,
                   std::string_view frame_id, std::uint32_t frame_duration_ms = 100);

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

    /// @brief 非重复扫描机型下观察到的 `frame_cnt` / `udp_cnt` 异常次数（只观测，不切帧）。
    std::uint64_t frame_cnt_discontinuities() const noexcept {
        return frame_cnt_discontinuities_.load(std::memory_order_relaxed);
    }

    /// @brief 因设备时间戳回退（乱序/抖动）被丢弃的包数。
    std::uint64_t stale_time_packets() const noexcept {
        return stale_time_packets_.load(std::memory_order_relaxed);
    }

    /// @brief 设备时间整段回退（PTP 阶跃/重新授时）的次数，每次按新纪元接受。
    std::uint64_t clock_jumps() const noexcept {
        return clock_jumps_.load(std::memory_order_relaxed);
    }

  private:
    bool begin_frame(const PacketHeader& header, std::uint64_t receive_steady_ns);
    bool is_frame_boundary(const PacketHeader& header, std::uint64_t receive_steady_ns);
    bool accept_frame_base(const PacketHeader& header) noexcept;
    bool frame_expired(const PacketHeader& header, std::uint64_t receive_steady_ns) const noexcept;
    bool publish();
    void discard();

    hal::PointXYZITL* points() noexcept;

    std::shared_ptr<hal::FramePool<hal::PointCloudFrame>> pool_;
    std::size_t capacity_ = 0;
    std::uint8_t line_num_ = 1;
    /// @brief 是否允许用 `frame_cnt` / `udp_cnt` 判定帧边界；由机型决定。
    bool frame_cnt_represents_frame_ = true;
    std::string_view frame_id_;
    /// @brief 时间组帧窗口；非重复扫描下 frame_cnt 无效时必须按时间切帧。
    std::int64_t frame_duration_ns_ = 100'000'000;

    std::optional<hal::FramePool<hal::PointCloudFrame>::Writer> writer_;
    std::uint8_t current_frame_cnt_ = 0;
    std::int64_t base_device_ns_ = 0;
    /// @brief 已接受帧基准的单调水线，用于挡住乱序/回退包。
    std::int64_t last_base_device_ns_ = 0;
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
    std::atomic<std::uint64_t> frame_cnt_discontinuities_{0};
    std::atomic<std::uint64_t> stale_time_packets_{0};
    std::atomic<std::uint64_t> clock_jumps_{0};
};

} // namespace foray::platform::driver::livox::detail
