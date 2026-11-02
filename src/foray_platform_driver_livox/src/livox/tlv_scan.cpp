#include "livox/tlv_scan.hpp"

#include <algorithm>
#include <cstring>

namespace foray::platform::driver::livox::detail {

namespace {

constexpr std::size_t kTlvHeaderBytes = 2 * sizeof(std::uint16_t);

/// @brief 4 字节对齐分条时的步长。
constexpr std::size_t kAlignedStride = 4;

/// @brief 从 `value` 取字符串：遇 `'\0'` 提前结束，最多 `limit` 字节。
std::string read_bounded_c_string(const std::uint8_t* value, std::size_t limit) {
    const auto* bytes = reinterpret_cast<const char*>(value);
    const auto* terminator = static_cast<const char*>(std::memchr(bytes, '\0', limit));
    const std::size_t length =
        terminator == nullptr ? limit : static_cast<std::size_t>(terminator - bytes);
    return std::string(bytes, length);
}

/**
 * @brief 单轮扫描：`stride == 0` 表示紧凑分条，否则按该值向上对齐。
 *
 * 任何一步越界都立即结束本轮并返回 `std::nullopt`——偏移一旦不可信，继续往后走只会
 * 读到别的字段甚至越界（审计 H25 点名的无界推进）。
 */
std::optional<std::string> scan_pass(const std::uint8_t* data, std::size_t capacity,
                                     std::uint16_t param_num, std::uint16_t wanted_key,
                                     std::size_t max_string, std::size_t stride) {
    std::size_t offset = 0;
    for (std::uint16_t index = 0; index < param_num; ++index) {
        if (offset > capacity || capacity - offset < kTlvHeaderBytes) {
            return std::nullopt;
        }
        std::uint16_t key = 0;
        std::uint16_t length = 0;
        std::memcpy(&key, data + offset, sizeof(key));
        std::memcpy(&length, data + offset + sizeof(key), sizeof(length));

        const std::size_t value_offset = offset + kTlvHeaderBytes;
        const std::size_t remaining = capacity - value_offset;
        if (length > remaining) {
            return std::nullopt;
        }
        if (key == wanted_key && length > 0) {
            return read_bounded_c_string(data + value_offset,
                                         std::min<std::size_t>(length, max_string));
        }

        const std::size_t consumed = kTlvHeaderBytes + length;
        if (stride == 0) {
            offset += consumed;
        } else {
            offset = ((offset + consumed + stride - 1) / stride) * stride;
        }
    }
    return std::nullopt;
}

} // namespace

std::optional<std::string> tlv_find_string(const std::uint8_t* data, std::size_t capacity,
                                           std::uint16_t param_num, std::uint16_t wanted_key,
                                           std::size_t max_string) {
    if (data == nullptr || capacity == 0 || param_num == 0) {
        return std::nullopt;
    }
    if (auto aligned = scan_pass(data, capacity, param_num, wanted_key, max_string, kAlignedStride);
        aligned.has_value()) {
        return aligned;
    }
    return scan_pass(data, capacity, param_num, wanted_key, max_string, 0);
}

} // namespace foray::platform::driver::livox::detail
