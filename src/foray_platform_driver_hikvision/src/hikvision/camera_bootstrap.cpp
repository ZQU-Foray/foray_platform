#include "camera_bootstrap.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace foray::platform::driver::hikvision::detail {
namespace {

/// @brief RFC 3927 链路本地地址前缀 169.254.0.0/16。
constexpr std::uint8_t kLinkLocalOctet0 = 169;
constexpr std::uint8_t kLinkLocalOctet1 = 254;
constexpr std::uint8_t kLinkLocalPrefix = 16;
constexpr std::uint8_t kSameSubnetPrefix = 24;

/// @brief 主机侧固定 LLA。选 169.254.209.1 是为了避开常见相机自动地址（169.254.1.x / .100+）。
constexpr std::array<std::uint8_t, 4> kDefaultHost{{169, 254, 209, 1}};
constexpr std::uint8_t kLastOctet = 3;
constexpr std::uint8_t kFirstHostOctet = 1;
constexpr std::uint8_t kFallbackHostOctet = 254;

bool is_link_local(const net::Ipv4& address) noexcept {
    return address.octets[0] == kLinkLocalOctet0 && address.octets[1] == kLinkLocalOctet1;
}

bool same_as(const net::Ipv4& lhs, const std::array<std::uint8_t, 4>& rhs) noexcept {
    return lhs.octets == rhs;
}

net::Ipv4 make_host(const net::Ipv4& device, std::uint8_t last_octet) noexcept {
    net::Ipv4 host = device;
    host.octets[kLastOctet] = last_octet;
    return host;
}

HostAddressPlan default_link_local_plan(std::string reason) {
    HostAddressPlan plan;
    plan.host_ip = net::Ipv4{kDefaultHost};
    plan.prefix = kLinkLocalPrefix;
    plan.reason = std::move(reason);
    return plan;
}

} // namespace

HostAddressPlan plan_host_address(std::string_view device_ip,
                                  const net::InterfaceInfo* bridge) {
    const auto device = net::Ipv4::parse(device_ip);
    const bool device_is_link_local = device.has_value() && is_link_local(*device);

    // 相机在链路本地网段（或还没枚举到）时，优先复用桥上已有的 LLA，避免堆叠第二条地址。
    if (!device.has_value() || device_is_link_local) {
        if (bridge != nullptr) {
            for (const auto& [address, prefix] : bridge->addresses) {
                if (is_link_local(address)) {
                    HostAddressPlan plan;
                    plan.host_ip = address;
                    plan.prefix = prefix;
                    plan.reason = "复用桥上已有的链路本地地址 " + address.to_string() +
                                  "/" + std::to_string(prefix);
                    return plan;
                }
            }
        }
        if (device_is_link_local && same_as(*device, kDefaultHost)) {
            net::Ipv4 host{kDefaultHost};
            host.octets[kLastOctet] = kFallbackHostOctet;
            HostAddressPlan plan;
            plan.host_ip = host;
            plan.prefix = kLinkLocalPrefix;
            plan.reason = "相机占用 169.254.209.1，主机改用 .254 避免冲突";
            return plan;
        }
        if (device_is_link_local) {
            return default_link_local_plan("相机为链路本地地址 " + device->to_string() +
                                           "，主机使用 169.254.209.1/16 覆盖整个 LLA 空间");
        }
        return default_link_local_plan("尚未枚举到相机，先按链路本地地址 169.254.209.1/16 配网");
    }

    // 相机配置了固定地址：只能给主机配同网段地址。
    const bool device_is_first_host = device->octets[kLastOctet] == kFirstHostOctet;
    const std::uint8_t host_octet = device_is_first_host ? kFallbackHostOctet : kFirstHostOctet;
    const net::Ipv4 host = make_host(*device, host_octet);

    HostAddressPlan plan;
    plan.host_ip = host;
    plan.prefix = kSameSubnetPrefix;
    plan.reason = "相机 " + device->to_string() + " 不在链路本地网段，主机使用同网段 " +
                  host.to_string() + "/24";
    return plan;
}

} // namespace foray::platform::driver::hikvision::detail
