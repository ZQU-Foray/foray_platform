#include "foray/platform/net/netlink_backend.hpp"

#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include <arpa/inet.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace foray::platform::net {
namespace {

Result<void> map_errno(int error, const std::string& what) {
    ErrorCode code = ErrorCode::kInternal;
    switch (error) {
    case EPERM:
    case EACCES:
        code = ErrorCode::kPermissionDenied;
        break;
    case ENODEV:
    case ENXIO:
        code = ErrorCode::kDeviceNotFound;
        break;
    case EADDRINUSE:
    case EEXIST:
        code = ErrorCode::kAlreadyOpen;
        break;
    case EINVAL:
        code = ErrorCode::kInvalidArgument;
        break;
    default:
        break;
    }
    return Error{code, what + ": " + std::strerror(error)};
}

class NetlinkSocket {
  public:
    NetlinkSocket() : fd_(::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE)) {
    }
    ~NetlinkSocket() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }
    NetlinkSocket(const NetlinkSocket&) = delete;
    NetlinkSocket& operator=(const NetlinkSocket&) = delete;

    bool valid() const {
        return fd_ >= 0;
    }

    int fd() const {
        return fd_;
    }

  private:
    int fd_;
};

void add_attr(std::vector<char>& buffer, std::uint16_t type, const void* data,
              std::uint16_t length) {
    // 显式按 4 字节对齐计算长度，避免依赖 RTA_ALIGN 宏的推断。
    const std::size_t aligned_payload = (static_cast<std::size_t>(length) + 3U) & ~std::size_t{3U};
    const std::size_t offset = buffer.size();
    buffer.resize(offset + sizeof(rtattr) + aligned_payload, 0);

    rtattr attr{};
    attr.rta_type = type;
    attr.rta_len = static_cast<std::uint16_t>(RTA_LENGTH(length));
    std::memcpy(buffer.data() + offset, &attr, sizeof(attr));
    if (length > 0) {
        std::memcpy(buffer.data() + offset + sizeof(attr), data, length);
    }
}

std::size_t begin_nested(std::vector<char>& buffer, std::uint16_t type) {
    rtattr attr{};
    attr.rta_type = type;
    attr.rta_len = static_cast<std::uint16_t>(RTA_LENGTH(0));
    const std::size_t offset = buffer.size();
    buffer.resize(offset + sizeof(attr), 0);
    std::memcpy(buffer.data() + offset, &attr, sizeof(attr));
    return offset;
}

void end_nested(std::vector<char>& buffer, std::size_t offset) {
    auto* attr = reinterpret_cast<rtattr*>(buffer.data() + offset);
    attr->rta_len = static_cast<std::uint16_t>(buffer.size() - offset);
}

Result<void> send_request(std::vector<char>& request, const std::string& what) {
    NetlinkSocket socket;
    if (!socket.valid()) {
        return map_errno(errno, what);
    }
    sockaddr_nl kernel{};
    kernel.nl_family = AF_NETLINK;
    auto* header = reinterpret_cast<nlmsghdr*>(request.data());
    header->nlmsg_seq = 1;
    const ssize_t sent =
        ::sendto(socket.fd(), request.data(), request.size(), 0,
                 reinterpret_cast<const sockaddr*>(&kernel), sizeof(kernel));
    if (sent < 0) {
        return map_errno(errno, what);
    }

    std::vector<char> response(8192);
    const ssize_t received = ::recv(socket.fd(), response.data(), response.size(), 0);
    if (received < 0) {
        return map_errno(errno, what);
    }
    // NLMSG_NEXT 会就地递减长度，因此需要非 const 的副本。
    auto remaining = received;
    for (auto* message = reinterpret_cast<nlmsghdr*>(response.data());
         NLMSG_OK(message, remaining); message = NLMSG_NEXT(message, remaining)) {
        if (message->nlmsg_type == NLMSG_ERROR) {
            const auto* error = reinterpret_cast<const nlmsgerr*>(NLMSG_DATA(message));
            if (error->error != 0) {
                return map_errno(-error->error, what);
            }
            return Result<void>();
        }
    }
    return Result<void>();
}

std::uint32_t index_of(const std::string& name) {
    return ::if_nametoindex(name.c_str());
}

class NetlinkBackend final : public NetBackend {
  public:
    BackendKind kind() const override {
        return BackendKind::kNetlink;
    }

    Result<void> ensure_bridge(const std::string& bridge) override {
        if (index_of(bridge) != 0) {
            return set_up(bridge);
        }
        std::vector<char> request(NLMSG_SPACE(sizeof(ifinfomsg)), 0);
        auto* header = reinterpret_cast<nlmsghdr*>(request.data());
        header->nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
        header->nlmsg_type = RTM_NEWLINK;
        header->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_CREATE | NLM_F_EXCL;
        auto* info = reinterpret_cast<ifinfomsg*>(NLMSG_DATA(header));
        info->ifi_family = AF_UNSPEC;

        add_attr(request, IFLA_IFNAME, bridge.c_str(),
                 static_cast<std::uint16_t>(bridge.size() + 1));
        const std::size_t linkinfo = begin_nested(request, IFLA_LINKINFO);
        const char kind[] = "bridge";
        add_attr(request, IFLA_INFO_KIND, kind, sizeof(kind));
        end_nested(request, linkinfo);
        header->nlmsg_len = static_cast<std::uint32_t>(request.size());

        auto created = send_request(request, "创建桥 " + bridge);
        if (!created.ok()) {
            return created.error();
        }
        return set_up(bridge);
    }

    Result<void> set_slave(const std::string& iface, const std::string& bridge) override {
        const std::uint32_t iface_index = index_of(iface);
        const std::uint32_t bridge_index = index_of(bridge);
        if (iface_index == 0) {
            return Error{ErrorCode::kDeviceNotFound, "找不到网口: " + iface};
        }
        if (bridge_index == 0) {
            return Error{ErrorCode::kDeviceNotFound, "找不到桥: " + bridge};
        }
        std::vector<char> request(NLMSG_SPACE(sizeof(ifinfomsg)), 0);
        auto* header = reinterpret_cast<nlmsghdr*>(request.data());
        header->nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
        header->nlmsg_type = RTM_NEWLINK;
        header->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
        auto* info = reinterpret_cast<ifinfomsg*>(NLMSG_DATA(header));
        info->ifi_family = AF_UNSPEC;
        info->ifi_index = static_cast<int>(iface_index);
        add_attr(request, IFLA_MASTER, &bridge_index, sizeof(bridge_index));
        header->nlmsg_len = static_cast<std::uint32_t>(request.size());
        return send_request(request, "把 " + iface + " 挂到 " + bridge);
    }

    Result<void> clear_slave(const std::string& iface) override {
        const std::uint32_t iface_index = index_of(iface);
        if (iface_index == 0) {
            return Error{ErrorCode::kDeviceNotFound, "找不到网口: " + iface};
        }
        std::vector<char> request(NLMSG_SPACE(sizeof(ifinfomsg)), 0);
        auto* header = reinterpret_cast<nlmsghdr*>(request.data());
        header->nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
        header->nlmsg_type = RTM_NEWLINK;
        header->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
        auto* info = reinterpret_cast<ifinfomsg*>(NLMSG_DATA(header));
        info->ifi_family = AF_UNSPEC;
        info->ifi_index = static_cast<int>(iface_index);
        const std::uint32_t no_master = 0;
        add_attr(request, IFLA_MASTER, &no_master, sizeof(no_master));
        header->nlmsg_len = static_cast<std::uint32_t>(request.size());
        return send_request(request, "解除 " + iface + " 的桥归属");
    }

    Result<void> set_up(const std::string& name) override {
        const std::uint32_t index = index_of(name);
        if (index == 0) {
            return Error{ErrorCode::kDeviceNotFound, "找不到网口: " + name};
        }
        std::vector<char> request(NLMSG_SPACE(sizeof(ifinfomsg)), 0);
        auto* header = reinterpret_cast<nlmsghdr*>(request.data());
        header->nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
        header->nlmsg_type = RTM_NEWLINK;
        header->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
        auto* info = reinterpret_cast<ifinfomsg*>(NLMSG_DATA(header));
        info->ifi_family = AF_UNSPEC;
        info->ifi_index = static_cast<int>(index);
        info->ifi_flags = IFF_UP;
        info->ifi_change = IFF_UP;
        return send_request(request, "拉起 " + name);
    }

    Result<void> set_mtu(const std::string& name, std::uint32_t mtu) override {
        if (mtu < 576 || mtu > 65535) {
            return Error{ErrorCode::kInvalidArgument,
                         "MTU 超出 576–65535: " + std::to_string(mtu)};
        }
        const std::uint32_t index = index_of(name);
        if (index == 0) {
            return Error{ErrorCode::kDeviceNotFound, "找不到网口: " + name};
        }
        std::vector<char> request(NLMSG_SPACE(sizeof(ifinfomsg)), 0);
        auto* header = reinterpret_cast<nlmsghdr*>(request.data());
        header->nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
        header->nlmsg_type = RTM_NEWLINK;
        header->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
        auto* info = reinterpret_cast<ifinfomsg*>(NLMSG_DATA(header));
        info->ifi_family = AF_UNSPEC;
        info->ifi_index = static_cast<int>(index);
        // 只改 MTU，ifi_flags/ifi_change 保持 0，避免顺带改动 flags。
        add_attr(request, IFLA_MTU, &mtu, sizeof(mtu));
        header->nlmsg_len = static_cast<std::uint32_t>(request.size());
        return send_request(request,
                            "设置 " + name + " 的 MTU 为 " + std::to_string(mtu));
    }

    Result<void> add_ipv4(const std::string& name, const Ipv4& ip, std::uint8_t prefix) override {
        return address_request(RTM_NEWADDR, name, ip, prefix, "配置地址");
    }

    Result<void> remove_ipv4(const std::string& name, const Ipv4& ip, std::uint8_t prefix) override {
        return address_request(RTM_DELADDR, name, ip, prefix, "删除地址");
    }

  private:
    Result<void> address_request(std::uint16_t type, const std::string& name, const Ipv4& ip,
                                 std::uint8_t prefix, const std::string& what) {
        const std::uint32_t index = index_of(name);
        if (index == 0) {
            return Error{ErrorCode::kDeviceNotFound, "找不到网口: " + name};
        }
        if (prefix > 32) {
            return Error{ErrorCode::kInvalidArgument, "前缀长度超出 0–32"};
        }
        std::vector<char> request(NLMSG_SPACE(sizeof(ifaddrmsg)), 0);
        auto* header = reinterpret_cast<nlmsghdr*>(request.data());
        header->nlmsg_len = NLMSG_LENGTH(sizeof(ifaddrmsg));
        header->nlmsg_type = type;
        header->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
        if (type == RTM_NEWADDR) {
            header->nlmsg_flags |= NLM_F_CREATE | NLM_F_EXCL;
        }
        auto* info = reinterpret_cast<ifaddrmsg*>(NLMSG_DATA(header));
        info->ifa_family = AF_INET;
        info->ifa_prefixlen = prefix;
        info->ifa_scope = RT_SCOPE_UNIVERSE;
        info->ifa_index = index;
        in_addr raw{};
        std::memcpy(&raw.s_addr, ip.octets.data(), ip.octets.size());
        add_attr(request, IFA_LOCAL, &raw.s_addr, sizeof(raw.s_addr));
        add_attr(request, IFA_ADDRESS, &raw.s_addr, sizeof(raw.s_addr));
        header->nlmsg_len = static_cast<std::uint32_t>(request.size());
        return send_request(request, what + " " + ip.to_string() + "/" + std::to_string(prefix));
    }
};

} // namespace

Result<std::unique_ptr<NetBackend>> make_netlink_backend() {
    return std::unique_ptr<NetBackend>(new NetlinkBackend());
}

} // namespace foray::platform::net
