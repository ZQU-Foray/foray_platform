#include "foray_platform/robolink_byte_channel.hpp"

#include <array>
#include <cerrno>
#include <system_error>

#include "foray_platform/robolink_codec.hpp"

namespace foray_platform {

RoboLinkByteChannel::RoboLinkByteChannel(UsbCdcPort& port) : port_(port) {}

bool RoboLinkByteChannel::queue_frame(const std::vector<std::uint8_t>& frame)
{
    if (has_pending_write()) {
        return false;
    }

    // 先确认是一个完整有效的候选帧，防止把半帧或损坏帧交给设备。
    RoboLinkFrameParser verifier;
    const auto parsed = verifier.feed(frame.data(), frame.size());
    if (parsed.size() != 1 || parsed.front() != frame ||
        !decode_frame(frame).has_value()) {
        return false;
    }
    tx_pending_ = frame;
    tx_offset_ = 0;
    return true;
}

bool RoboLinkByteChannel::has_pending_write() const
{
    return tx_offset_ < tx_pending_.size();
}

ssize_t RoboLinkByteChannel::flush_once()
{
    if (!has_pending_write()) {
        return 0;
    }
    const ssize_t count = port_.write_bytes(
        tx_pending_.data() + tx_offset_, tx_pending_.size() - tx_offset_);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return 0;
    }
    if (count > 0) {
        tx_offset_ += static_cast<std::size_t>(count);
        if (!has_pending_write()) {
            tx_pending_.clear();
            tx_offset_ = 0;
        }
    }
    return count;
}

std::vector<std::vector<std::uint8_t>> RoboLinkByteChannel::receive_once()
{
    std::array<std::uint8_t, 512> bytes{};
    const ssize_t count = port_.read_bytes(bytes.data(), bytes.size());
    if (count < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return {};
        }
        throw std::system_error(errno, std::generic_category(), "USB CDC 读取失败");
    }
    if (count == 0) {
        return {};
    }
    return parser_.feed(bytes.data(), static_cast<std::size_t>(count));
}

void RoboLinkByteChannel::reset()
{
    parser_ = RoboLinkFrameParser{};
    tx_pending_.clear();
    tx_offset_ = 0;
}

}  // namespace foray_platform
