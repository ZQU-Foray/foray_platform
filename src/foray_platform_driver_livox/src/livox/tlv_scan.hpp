#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace foray::platform::driver::livox::detail {

/// @brief TLV 扫描的命中结果。
struct TlvStringField {
    std::size_t offset = 0; ///< 值在负载中的字节偏移。
    std::size_t length = 0; ///< 实际可安全读取的长度（已按剩余容量与上限截断）。
};

/**
 * @brief 在 `QueryLivoxLidarInternalInfo` 的 TLV 负载里查找字符串字段（审计 H25）。
 *
 * SDK 只给出 `param_num`，没有总长度，所以每一步都必须拿 `capacity` 兜底：
 * - 读不够「key(2B)+length(2B)」就结束这一轮；
 * - `length` 超出剩余容量就结束这一轮（**不截断继续往后走**，因为后面的偏移已经不可信）；
 * - 值按 `max_string` 截断，遇到 `'\0'` 提前结束（SDK 的定长字符数组不一定带结尾 NUL）。
 *
 * 两种序列化方式都要试：先按 4 字节对齐分条扫描（SDK 常见），再按紧凑分条扫描。
 * 返回 `std::nullopt` 表示两种分条方式都没找到，调用方必须把它当成「没拿到」而不是空字符串。
 */
std::optional<std::string> tlv_find_string(const std::uint8_t* data, std::size_t capacity,
                                           std::uint16_t param_num, std::uint16_t wanted_key,
                                           std::size_t max_string);

} // namespace foray::platform::driver::livox::detail
