#include "livox/frame_assembler.hpp"

#include <utility>

namespace foray::platform::driver::livox::detail {

FrameAssembler::FrameAssembler(std::shared_ptr<hal::FramePool<hal::PointCloudFrame>> pool,
                               std::uint32_t point_capacity, std::uint8_t line_num,
                               std::string_view frame_id, std::uint32_t frame_duration_ms)
    : pool_(std::move(pool)), capacity_(point_capacity), line_num_(line_num), frame_id_(frame_id),
      frame_duration_ns_(static_cast<std::int64_t>(frame_duration_ms) * 1'000'000) {
}

bool FrameAssembler::on_packet(const PacketHeader& header, const std::uint8_t* packet,
                               std::size_t size, std::uint64_t receive_steady_ns) {
    last_packet_steady_ns_ = receive_steady_ns;

    bool published = false;
    const bool frame_boundary = writer_.has_value() && (header.frame_cnt != current_frame_cnt_ ||
                                                        (header.udp_cnt == 0 && count_ > 0) ||
                                                        frame_expired(header, receive_steady_ns));

    if (!writer_.has_value() || frame_boundary) {
        if (writer_.has_value()) {
            published = publish();
        }
        if (!begin_frame(header, receive_steady_ns)) {
            return published;
        }
    }

    if (dropping_ || frame_overflow_ || header.dot_num == 0) {
        if (frame_overflow_ || dropping_) {
            points_dropped_.fetch_add(header.dot_num, std::memory_order_relaxed);
        }
        return published;
    }

    const std::size_t remaining = capacity_ - count_;
    std::size_t parsed = 0;
    const Result<void> result = parse_points(header, packet, size, line_num_, base_device_ns_,
                                             points() + count_, remaining, parsed);

    if (!result.ok()) {
        ++frame_dropped_packets_;
        points_dropped_.fetch_add(header.dot_num, std::memory_order_relaxed);
        if (result.error().code == ErrorCode::kQueueFull) {
            // 当前帧已放不下，后续包全部丢弃，等下一帧。
            frame_overflow_ = true;
        }
        return published;
    }

    count_ += parsed;
    return published;
}

bool FrameAssembler::publish_if_idle(std::uint64_t now_steady_ns, std::int64_t timeout_ns) {
    if (!writer_.has_value() || count_ == 0) {
        return false;
    }
    if (timeout_ns < 0) {
        timeout_ns = 0;
    }
    if (now_steady_ns - last_packet_steady_ns_ < static_cast<std::uint64_t>(timeout_ns)) {
        return false;
    }
    return publish();
}

bool FrameAssembler::flush() {
    if (!writer_.has_value()) {
        return false;
    }
    if (count_ == 0) {
        discard();
        return false;
    }
    return publish();
}

bool FrameAssembler::frame_expired(const PacketHeader& header,
                                   std::uint64_t receive_steady_ns) const noexcept {
    if (!writer_.has_value() || count_ == 0 || frame_duration_ns_ <= 0) {
        return false;
    }

    const std::uint64_t duration_ns = static_cast<std::uint64_t>(frame_duration_ns_);
    if (header.timestamp_ns > 0 && base_device_ns_ > 0) {
        return header.timestamp_ns >= static_cast<std::uint64_t>(base_device_ns_) + duration_ns;
    }
    return receive_steady_ns >= base_receive_steady_ns_ + duration_ns;
}

bool FrameAssembler::begin_frame(const PacketHeader& header, std::uint64_t receive_steady_ns) {
    std::optional<hal::FramePool<hal::PointCloudFrame>::Writer> writer =
        pool_->try_acquire_writer();
    if (!writer.has_value()) {
        dropping_ = true;
        dropped_frames_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    writer_ = std::move(*writer);
    current_frame_cnt_ = header.frame_cnt;
    base_device_ns_ = static_cast<std::int64_t>(header.timestamp_ns);
    base_receive_steady_ns_ = receive_steady_ns;
    sync_ = sync_from_time_type(header.time_type);
    count_ = 0;
    frame_dropped_packets_ = 0;
    frame_overflow_ = false;
    dropping_ = false;
    return true;
}

bool FrameAssembler::publish() {
    if (!writer_.has_value()) {
        return false;
    }
    if (count_ == 0) {
        discard();
        return false;
    }

    hal::PointCloudFrame& frame = writer_->frame();
    frame.points = reinterpret_cast<const hal::PointXYZITL*>(writer_->data());
    frame.count = count_;
    frame.device_base_time_ns = base_device_ns_;
    frame.receive_stamp =
        time::Timestamp::from_nanoseconds(static_cast<std::int64_t>(base_receive_steady_ns_));
    frame.sync = sync_;

    // 只有 PTP / GPS PPS 的设备时间才是绝对时间；PPS / NoSync / Unknown
    // 一律把绝对时间置零，避免静默混用时间域。
    if (sync_ == hal::TimestampSync::kPtp || sync_ == hal::TimestampSync::kGpsPps) {
        frame.base_time = time::Timestamp::from_nanoseconds(base_device_ns_);
    } else {
        frame.base_time = time::Timestamp();
    }

    frame.sequence = ++sequence_;
    frame.frame_id = frame_id_;
    frame.dropped_packets = frame_dropped_packets_;

    writer_->publish();
    writer_.reset();
    frames_published_.fetch_add(1, std::memory_order_relaxed);
    count_ = 0;
    return true;
}

void FrameAssembler::discard() {
    writer_.reset();
    count_ = 0;
}

hal::PointXYZITL* FrameAssembler::points() noexcept {
    // `unique_ptr<std::uint8_t[]>` 的缓冲由 `new[]` 分配，满足 PointXYZITL 的对齐要求。
    return reinterpret_cast<hal::PointXYZITL*>(writer_->data());
}

} // namespace foray::platform::driver::livox::detail
