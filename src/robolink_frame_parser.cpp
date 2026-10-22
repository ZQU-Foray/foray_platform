#include "foray_platform/robolink_frame_parser.hpp"

#include <algorithm>
#include <stdexcept>

#include "foray_platform/robolink_crc.hpp"

namespace foray_platform {
namespace {

// 常量来自 RoboLink v0.1 候选文档的帧层定义。
constexpr std::uint8_t kMagic[] = {0xD3, 0x91};
constexpr std::uint8_t kVersion = 0x02;
constexpr std::uint8_t kKnownFlags = 0x03;
constexpr std::uint8_t kKeepaliveFlag = 0x02;
constexpr std::size_t kHeaderSize = 14;
constexpr std::size_t kCrcSize = 4;
constexpr std::size_t kMinFrameSize = 18;
constexpr std::size_t kMaxFrameSize = 512;

std::uint32_t read_u32_le(const std::uint8_t* bytes)
{
    return static_cast<std::uint32_t>(bytes[0]) |
           (static_cast<std::uint32_t>(bytes[1]) << 8) |
           (static_cast<std::uint32_t>(bytes[2]) << 16) |
           (static_cast<std::uint32_t>(bytes[3]) << 24);
}

}  // namespace

std::vector<std::vector<std::uint8_t>> RoboLinkFrameParser::feed(
    const std::uint8_t* data, std::size_t size)
{
    if (size != 0 && data == nullptr) {
        throw std::invalid_argument("nonempty byte chunk requires data");
    }

    if (size != 0) {
        pending_.insert(pending_.end(), data, data + size);
    }
    std::vector<std::vector<std::uint8_t>> frames;

    while (true) {
        const auto magic = std::search(
            pending_.begin(), pending_.end(), std::begin(kMagic), std::end(kMagic));
        if (magic == pending_.end()) {
            // 保留末尾的 0xD3：下一次读取可能以 0x91 开头。
            const bool starts_magic = !pending_.empty() && pending_.back() == kMagic[0];
            pending_.clear();
            if (starts_magic) {
                pending_.push_back(kMagic[0]);
            }
            break;
        }
        pending_.erase(pending_.begin(), magic);

        if (pending_.size() < kHeaderSize) {
            break;
        }

        const std::size_t frame_size =
            static_cast<std::size_t>(pending_[4]) |
            (static_cast<std::size_t>(pending_[5]) << 8);
        const std::uint8_t flags = pending_[3];
        const bool keepalive = (flags & kKeepaliveFlag) != 0;
        const bool bad_header =
            pending_[2] != kVersion || (flags & ~kKnownFlags) != 0 ||
            frame_size < kMinFrameSize || frame_size > kMaxFrameSize ||
            pending_[13] != 0 ||
            (pending_[12] == 0) != keepalive;
        if (bad_header) {
            // 假帧头可能和真帧头重叠，从第二字节重新扫描。
            pending_.erase(pending_.begin());
            continue;
        }

        if (pending_.size() < frame_size) {
            break;
        }

        const std::uint32_t expected_crc =
            read_u32_le(pending_.data() + frame_size - kCrcSize);
        const std::uint32_t actual_crc =
            crc32_iso_hdlc(pending_.data(), frame_size - kCrcSize);
        if (actual_crc == expected_crc) {
            frames.emplace_back(pending_.begin(), pending_.begin() + frame_size);
        }
        // CRC 失败时丢弃本候选帧，继续扫描帧尾之后的数据。
        pending_.erase(pending_.begin(), pending_.begin() + frame_size);
    }

    return frames;
}

}  // namespace foray_platform
