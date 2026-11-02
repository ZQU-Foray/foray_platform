#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "livox/tlv_scan.hpp"

namespace foray::platform::driver::livox::detail {
namespace {

constexpr std::uint16_t kKeyIp = 0x0100;
constexpr std::uint16_t kKeyPort = 0x0101;

/// @brief 追加一个 TLV：key(2) + length(2) + value，`align` 为 0 时紧凑排列。
void append_tlv(std::vector<std::uint8_t>& out, std::uint16_t key, const std::string& value,
                std::size_t align) {
    const auto length = static_cast<std::uint16_t>(value.size());
    out.push_back(static_cast<std::uint8_t>(key & 0xFFU));
    out.push_back(static_cast<std::uint8_t>(key >> 8));
    out.push_back(static_cast<std::uint8_t>(length & 0xFFU));
    out.push_back(static_cast<std::uint8_t>(length >> 8));
    out.insert(out.end(), value.begin(), value.end());
    if (align != 0) {
        while (out.size() % align != 0) {
            out.push_back(0);
        }
    }
}

} // namespace

/**
 * 审计 H25：`QueryLivoxLidarInternalInfo` 的负载只给了 `param_num`、没有总长度，
 * 原实现直接 `memcpy(cursor, 4)` 再 `cursor += 4 + length` 无限往后推——设备（或
 * 传输错误）给一个超大的 `length` 就是越界读。
 */
TEST(TlvScanTest, FindsIpInAlignedPayload) {
    std::vector<std::uint8_t> payload;
    append_tlv(payload, kKeyPort, std::string("\x01\x02\x03\x04", 4), 4);
    append_tlv(payload, kKeyIp, std::string("192.168.1.195", 13), 4);

    const auto ip = tlv_find_string(payload.data(), payload.size(), 2, kKeyIp, 16);
    ASSERT_TRUE(ip.has_value());
    EXPECT_EQ(*ip, "192.168.1.195");
}

TEST(TlvScanTest, FallsBackToCompactPayload) {
    std::vector<std::uint8_t> payload;
    append_tlv(payload, kKeyPort, std::string("\x01\x02\x03\x04", 4), 0);
    append_tlv(payload, kKeyIp, std::string("10.0.0.7", 8), 0);

    const auto ip = tlv_find_string(payload.data(), payload.size(), 2, kKeyIp, 16);
    ASSERT_TRUE(ip.has_value());
    EXPECT_EQ(*ip, "10.0.0.7");
}

TEST(TlvScanTest, StopsAtNulTerminator) {
    // 定长字符数组不带结尾 NUL 也可能带；带了就必须在 NUL 处收尾。
    const std::string value("172.16.0.1\0garbage", 19);
    std::vector<std::uint8_t> payload;
    append_tlv(payload, kKeyIp, value, 0);

    const auto ip = tlv_find_string(payload.data(), payload.size(), 1, kKeyIp, 16);
    ASSERT_TRUE(ip.has_value());
    EXPECT_EQ(*ip, "172.16.0.1");
}

TEST(TlvScanTest, TruncatedHeaderStopsScan) {
    // 只剩 2 字节，连 length 都读不到：必须停，不许 memcpy 出去。
    const std::vector<std::uint8_t> payload{0x00, 0x01};
    EXPECT_FALSE(tlv_find_string(payload.data(), payload.size(), 1, kKeyIp, 16).has_value());
}

TEST(TlvScanTest, LengthBeyondCapacityStopsScan) {
    // length = 0xFFFF，但缓冲只有 8 字节：越界必须被容量挡住。
    std::vector<std::uint8_t> payload;
    append_tlv(payload, kKeyIp, std::string("1.1.1.1", 7), 0);
    // 把长度字段改成 0xFFFF。
    payload[2] = 0xFF;
    payload[3] = 0xFF;
    EXPECT_FALSE(tlv_find_string(payload.data(), payload.size(), 1, kKeyIp, 16).has_value());
}

TEST(TlvScanTest, StopsWhenParamCountExceedsPayload) {
    std::vector<std::uint8_t> payload;
    append_tlv(payload, kKeyPort, std::string("\x01\x02", 2), 4);
    // param_num = 5，但负载里只有 1 条：第二轮就该停下，而不是继续往后读。
    EXPECT_FALSE(tlv_find_string(payload.data(), payload.size(), 5, kKeyIp, 16).has_value());
}

TEST(TlvScanTest, ReturnsNulloptWhenKeyMissing) {
    std::vector<std::uint8_t> payload;
    append_tlv(payload, kKeyPort, std::string("\x01\x02", 2), 4);
    EXPECT_FALSE(tlv_find_string(payload.data(), payload.size(), 1, kKeyIp, 16).has_value());
}

TEST(TlvScanTest, EmptyAndNullPayloadsAreRejected) {
    const std::uint8_t byte = 0;
    EXPECT_FALSE(tlv_find_string(nullptr, 16, 1, kKeyIp, 16).has_value());
    EXPECT_FALSE(tlv_find_string(&byte, 0, 1, kKeyIp, 16).has_value());
    EXPECT_FALSE(tlv_find_string(&byte, 1, 0, kKeyIp, 16).has_value());
}

} // namespace foray::platform::driver::livox::detail
