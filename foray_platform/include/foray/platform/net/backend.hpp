#pragma once

#include <cstdint>
#include <string>

#include "foray/platform/net/net_types.hpp"
#include "foray/platform/result/result.hpp"

namespace foray::platform::net {

/// 网络配置后端。
enum class BackendKind {
    kAuto,           ///< 由探测结果决定
    kNetworkManager, ///< nmcli 路径（持久化 profile，掉电重启仍生效）
    kNetlink,        ///< 直接 rtnetlink（需要 CAP_NET_ADMIN）
};

/// @brief 后端名称，用于诊断输出与报告。
inline const char* to_string(BackendKind kind) {
    switch (kind) {
    case BackendKind::kAuto:
        return "auto";
    case BackendKind::kNetworkManager:
        return "network-manager";
    case BackendKind::kNetlink:
        return "netlink";
    }
    return "unknown";
}

/**
 * @brief 网络配置的最小操作集。
 *
 * 规划层只依赖这几个操作，因此后端可以整体替换为测试假实现；
 * 全部操作都要求幂等（重复调用不应报错）。
 */
class NetBackend {
  public:
    virtual ~NetBackend() = default;

    virtual BackendKind kind() const = 0;

    /// @brief 确保桥存在并处于可用配置（不存在时创建）。
    virtual Result<void> ensure_bridge(const std::string& bridge) = 0;

    /// @brief 把物理口挂到桥上。
    virtual Result<void> set_slave(const std::string& iface, const std::string& bridge) = 0;

    /// @brief 解除物理口的桥归属。
    virtual Result<void> clear_slave(const std::string& iface) = 0;

    /// @brief 把接口拉起来。
    virtual Result<void> set_up(const std::string& name) = 0;

    /// @brief 给接口加一个 IPv4 地址。
    virtual Result<void> add_ipv4(const std::string& name, const Ipv4& ip, std::uint8_t prefix) = 0;

    /// @brief 删除接口上的 IPv4 地址（回滚用）。
    virtual Result<void> remove_ipv4(const std::string& name, const Ipv4& ip, std::uint8_t prefix) = 0;
};

} // namespace foray::platform::net
