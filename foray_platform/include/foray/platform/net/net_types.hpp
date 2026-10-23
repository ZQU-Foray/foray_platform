#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "foray/platform/result/result.hpp"

namespace foray::platform::net {

/**
 * @brief IPv4 地址（四个八位组）。
 *
 * 只支持点分十进制；解析失败返回 `std::nullopt`，便于配置校验阶段直接拒绝。
 */
struct Ipv4 {
    std::array<std::uint8_t, 4> octets{};

    static std::optional<Ipv4> parse(std::string_view text) noexcept;
    std::string to_string() const;
    bool same_subnet(const Ipv4& other, std::uint8_t prefix) const noexcept;
};

/// 接口角色，用于诊断报告与后端选择（桥、物理以太网、无线）。
enum class LinkRole { kUnknown, kBridge, kEthernet, kWireless };

/**
 * @brief 一个网络接口的只读快照。
 *
 * 全部字段来自 sysfs / `getifaddrs()`，**不需要提权**；
 * `managed_by_nm` 由 NetworkManager 后端探测后回填，sysfs 探测恒为 false。
 */
struct InterfaceInfo {
    std::string name;
    int index = 0;
    std::string mac;
    LinkRole role = LinkRole::kUnknown;
    bool up = false;
    /// @brief sysfs `carrier`；对物理口表示对端链路是否建立。
    bool carrier = false;
    bool managed_by_nm = false;
    /// @brief 所属桥名；空字符串表示未挂桥。
    std::string master;
    std::vector<std::pair<Ipv4, std::uint8_t>> addresses;
};

/// 一次探测得到的全部接口与宿主环境信息。
struct Topology {
    std::vector<InterfaceInfo> interfaces;
    bool network_manager_present = false;

    const InterfaceInfo* find(std::string_view name) const;
};

/// 探测参数；`sysfs_root` 可替换以便单元测试用假 sysfs 目录。
struct ProbeOptions {
    std::string sysfs_root = "/sys/class/net";
};

/**
 * @brief 只读探测当前网络拓扑。
 *
 * 不修改任何状态、不需要 root；读不到 sysfs 根目录时返回 `kDeviceNotFound`。
 */
Result<Topology> probe_topology(const ProbeOptions& options = {});

} // namespace foray::platform::net
