#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "foray/platform/net/net_types.hpp"

namespace foray::platform::driver::hikvision::detail {

/// @brief 主机侧桥地址的规划结果；`reason` 供现场打印，解释为什么选这个地址。
struct HostAddressPlan {
    std::optional<net::Ipv4> host_ip;
    std::uint8_t prefix = 16;
    std::string reason;
};

/**
 * @brief 按「相机 IP + 桥上现有地址」推导主机地址。
 *
 * 规则（按顺序）：
 * 1. 相机是链路本地地址（169.254/16）或尚未枚举到：桥上已有链路本地地址则复用，
 *    否则用固定的 `169.254.209.1/16`（/16 覆盖整个 LLA 空间）；
 * 2. 相机在其它网段：用同一 /24 的 `.1`；若相机自己就是 `.1`，改用 `.254`；
 * 3. 相机地址为空且桥上也没有 LLA：仍按 `169.254.209.1/16` 规划，便于先配网再枚举。
 *
 * @param device_ip 第一台枚举到的相机的 IPv4 文本；空表示尚未看到设备。
 * @param bridge 目标网桥的探测快照；可为 nullptr（桥还不存在）。
 */
HostAddressPlan plan_host_address(std::string_view device_ip,
                                  const net::InterfaceInfo* bridge);

} // namespace foray::platform::driver::hikvision::detail
