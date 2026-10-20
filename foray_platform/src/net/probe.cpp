#include "foray/platform/net/net_types.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <system_error>

namespace foray::platform::net {
namespace {

namespace fs = std::filesystem;

std::string read_first_line(const fs::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        return {};
    }
    std::string line;
    std::getline(stream, line);
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    return line;
}

/// 772 == ARPHRD_LOOPBACK；回环口对拓扑决策没有意义。
bool is_loopback(const fs::path& dir) {
    return read_first_line(dir / "type") == "772";
}

LinkRole detect_role(const fs::path& dir) {
    std::error_code ec;
    if (fs::exists(dir / "bridge", ec)) {
        return LinkRole::kBridge;
    }
    if (fs::exists(dir / "wireless", ec)) {
        return LinkRole::kWireless;
    }
    if (fs::exists(dir / "device", ec) || read_first_line(dir / "type") == "1") {
        return LinkRole::kEthernet;
    }
    return LinkRole::kUnknown;
}

std::string detect_master(const fs::path& dir) {
    std::error_code ec;
    const fs::path link = dir / "master";
    if (!fs::exists(link, ec)) {
        return {};
    }
    const fs::path target = fs::read_symlink(link, ec);
    if (ec) {
        return {};
    }
    return target.filename().string();
}

void fill_ipv4_addresses(InterfaceInfo& info) {
    ifaddrs* head = nullptr;
    if (::getifaddrs(&head) != 0) {
        return;
    }
    for (const ifaddrs* entry = head; entry != nullptr; entry = entry->ifa_next) {
        if (entry->ifa_addr == nullptr || entry->ifa_addr->sa_family != AF_INET ||
            entry->ifa_name == nullptr || info.name != entry->ifa_name) {
            continue;
        }
        const auto* addr = reinterpret_cast<const sockaddr_in*>(entry->ifa_addr);
        Ipv4 ip;
        std::memcpy(ip.octets.data(), &addr->sin_addr.s_addr, ip.octets.size());
        std::uint8_t prefix = 0;
        if (entry->ifa_netmask != nullptr) {
            const auto* mask = reinterpret_cast<const sockaddr_in*>(entry->ifa_netmask);
            std::uint32_t bits = 0;
            std::memcpy(&bits, &mask->sin_addr.s_addr, sizeof(bits));
            while (bits != 0) {
                prefix += static_cast<std::uint8_t>(bits & 0x1U);
                bits >>= 1U;
            }
        }
        info.addresses.emplace_back(ip, prefix);
    }
    ::freeifaddrs(head);
}

} // namespace

std::optional<Ipv4> Ipv4::parse(std::string_view text) noexcept {
    if (text.empty()) {
        return std::nullopt;
    }
    in_addr parsed{};
    std::string copy(text);
    if (::inet_pton(AF_INET, copy.c_str(), &parsed) != 1) {
        return std::nullopt;
    }
    std::array<std::uint8_t, 4> octets{};
    std::memcpy(octets.data(), &parsed.s_addr, octets.size());
    return Ipv4{octets};
}

std::string Ipv4::to_string() const {
    in_addr raw{};
    std::memcpy(&raw.s_addr, octets.data(), octets.size());
    std::array<char, INET_ADDRSTRLEN> buffer{};
    if (::inet_ntop(AF_INET, &raw, buffer.data(), buffer.size()) == nullptr) {
        return {};
    }
    return buffer.data();
}

bool Ipv4::same_subnet(const Ipv4& other, std::uint8_t prefix) const noexcept {
    if (prefix > 32) {
        return false;
    }
    // 整字节部分直接比较，最后一个不满 8 位的字节做掩码比较。
    const std::uint8_t full_bytes = static_cast<std::uint8_t>(prefix / 8);
    const std::uint8_t remainder = static_cast<std::uint8_t>(prefix % 8);
    if (!std::equal(octets.begin(), octets.begin() + full_bytes, other.octets.begin())) {
        return false;
    }
    if (remainder == 0) {
        return true;
    }
    const std::uint8_t mask = static_cast<std::uint8_t>(0xFFU << (8U - remainder));
    return (octets[full_bytes] & mask) == (other.octets[full_bytes] & mask);
}

const InterfaceInfo* Topology::find(std::string_view name) const {
    const auto it = std::find_if(interfaces.begin(), interfaces.end(),
                                 [name](const InterfaceInfo& info) { return info.name == name; });
    return it == interfaces.end() ? nullptr : &*it;
}

Result<Topology> probe_topology(const ProbeOptions& options) {
    std::error_code ec;
    const fs::path root(options.sysfs_root);
    if (!fs::is_directory(root, ec)) {
        return Error{ErrorCode::kDeviceNotFound,
                     "网络探测失败：sysfs 目录不存在: " + options.sysfs_root};
    }

    Topology topology;
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (ec) {
            break;
        }
        const fs::path dir = entry.path();
        if (!fs::is_directory(dir, ec) || is_loopback(dir)) {
            continue;
        }
        InterfaceInfo info;
        info.name = dir.filename().string();
        info.index = static_cast<int>(::if_nametoindex(info.name.c_str()));
        info.mac = read_first_line(dir / "address");
        info.role = detect_role(dir);
        info.master = detect_master(dir);
        info.up = read_first_line(dir / "operstate") == "up";
        info.carrier = read_first_line(dir / "carrier") == "1";
        fill_ipv4_addresses(info);
        topology.interfaces.push_back(std::move(info));
    }

    std::sort(topology.interfaces.begin(), topology.interfaces.end(),
              [](const InterfaceInfo& lhs, const InterfaceInfo& rhs) { return lhs.name < rhs.name; });
    return topology;
}

} // namespace foray::platform::net
