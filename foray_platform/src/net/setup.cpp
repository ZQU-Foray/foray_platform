#include "foray/platform/net/setup.hpp"

#include <algorithm>
#include <sstream>

#include "foray/platform/net/nm_backend.hpp"
#include "foray/platform/net/netlink_backend.hpp"
#include "foray/platform/net/process.hpp"

namespace foray::platform::net {
namespace {

const InterfaceInfo* find_bridge(const Topology& topology, const std::string& name) {
    const InterfaceInfo* info = topology.find(name);
    return info != nullptr && info->role == LinkRole::kBridge ? info : nullptr;
}

bool has_address(const InterfaceInfo* info, const Ipv4& ip, std::uint8_t prefix) {
    if (info == nullptr) {
        return false;
    }
    return std::any_of(info->addresses.begin(), info->addresses.end(),
                       [&ip, prefix](const auto& entry) {
                           return entry.second == prefix && entry.first.octets == ip.octets;
                       });
}

std::string action_detail(const NetIntent& intent, const LinkAction& action) {
    std::ostringstream stream;
    stream << action.operation << " " << action.target;
    if (action.operation == "add_ipv4" && intent.host_ip.has_value()) {
        stream << " " << intent.host_ip->to_string() << "/" << static_cast<int>(intent.prefix);
    }
    return stream.str();
}

/// 把已执行动作反向撤销；撤销失败只记警告，不覆盖原始错误。
void rollback(const std::vector<LinkAction>& applied, NetBackend& backend,
              std::vector<std::string>& warnings) {
    for (auto it = applied.rbegin(); it != applied.rend(); ++it) {
        Result<void> result;
        if (it->operation == "set_slave") {
            result = backend.clear_slave(it->target);
        } else if (it->operation == "add_ipv4" && it->address.has_value()) {
            result = backend.remove_ipv4(it->target, *it->address, it->prefix);
        } else if (it->operation == "remove_ipv4" && it->address.has_value()) {
            // 回滚：把误删的地址加回来。
            result = backend.add_ipv4(it->target, *it->address, it->prefix);
        }
        if (!result.ok()) {
            warnings.push_back("回滚失败: " + it->operation + " " + it->target + ": " +
                               result.error().message);
        }
    }
}

} // namespace

bool link_ready(const Topology& topology, const std::string& iface,
                const std::string& bridge, const Ipv4& host_ip,
                std::uint8_t min_prefix) {
    const InterfaceInfo* port = topology.find(iface);
    const InterfaceInfo* master = topology.find(bridge);
    if (port == nullptr || master == nullptr || port->master != bridge) {
        return false;
    }
    if (!master->up || !master->carrier) {
        return false;
    }
    for (const auto& [address, prefix] : master->addresses) {
        if (address.to_string() == host_ip.to_string() && prefix >= min_prefix) {
            return true;
        }
    }
    return false;
}

Result<std::vector<LinkAction>> plan_link(const NetIntent& intent, const Topology& topology) {
    if (intent.iface.empty() || intent.bridge.empty()) {
        return Error{ErrorCode::kInvalidArgument, "网络意图缺少 iface 或 bridge"};
    }
    if (intent.iface == intent.bridge) {
        return Error{ErrorCode::kInvalidArgument, "iface 与 bridge 不能相同: " + intent.iface};
    }
    if (intent.prefix > 32) {
        return Error{ErrorCode::kInvalidArgument, "前缀长度超出 0–32: " + std::to_string(intent.prefix)};
    }

    const InterfaceInfo* iface = topology.find(intent.iface);
    if (iface == nullptr) {
        return Error{ErrorCode::kDeviceNotFound, "找不到物理网口: " + intent.iface};
    }
    if (iface->role == LinkRole::kBridge) {
        return Error{ErrorCode::kInvalidArgument, intent.iface + " 是桥设备，不能作为物理口"};
    }

    std::vector<LinkAction> actions;
    const InterfaceInfo* bridge = find_bridge(topology, intent.bridge);
    if (bridge == nullptr) {
        actions.push_back({"ensure_bridge", intent.bridge, "创建桥 " + intent.bridge, std::nullopt, 0});
    }
    if (iface->master != intent.bridge) {
        actions.push_back({"set_slave", intent.iface,
                           "把 " + intent.iface + " 挂到 " + intent.bridge, std::nullopt, 0});
    }
    if (bridge == nullptr || !bridge->up) {
        actions.push_back({"set_up", intent.bridge, "拉起桥 " + intent.bridge, std::nullopt, 0});
    }
    if (intent.host_ip.has_value() && bridge != nullptr) {
        // 同网段的旧地址（换过 IP 的现场）先清掉，避免 SDK 绑错源地址。
        for (const auto& [address, prefix] : bridge->addresses) {
            if (address.to_string() == intent.host_ip->to_string()) {
                continue;
            }
            if (address.same_subnet(*intent.host_ip, intent.prefix)) {
                actions.push_back({"remove_ipv4", intent.bridge,
                                   "移除 " + intent.bridge + " 上的旧地址 " +
                                       address.to_string() + "/" + std::to_string(prefix),
                                   address, prefix});
            }
        }
    }
    if (intent.host_ip.has_value() && !has_address(bridge, *intent.host_ip, intent.prefix)) {
        actions.push_back({"add_ipv4", intent.bridge,
                           "给 " + intent.bridge + " 配置 " + intent.host_ip->to_string() + "/" +
                               std::to_string(intent.prefix),
                           intent.host_ip, intent.prefix});
    }
    return actions;
}

Result<LinkReport> ensure_link(const NetIntent& intent, NetBackend& backend, bool dry_run) {
    const auto probe = probe_topology();
    if (!probe.ok()) {
        return probe.error();
    }
    return ensure_link(intent, probe.value(), backend, dry_run);
}

Result<LinkReport> ensure_link(const NetIntent& intent, const Topology& topology,
                               NetBackend& backend, bool dry_run) {
    const auto planned = plan_link(intent, topology);
    if (!planned.ok()) {
        return planned.error();
    }

    LinkReport report;
    report.dry_run = dry_run;
    report.backend = backend.kind();
    report.planned = planned.value();

    const InterfaceInfo* iface = topology.find(intent.iface);
    if (iface != nullptr && !iface->carrier) {
        report.warnings.push_back("物理口 " + intent.iface +
                                  " 当前没有载波（雷达未上电、未插好或线序不对）");
    }
    if (iface != nullptr && iface->role == LinkRole::kWireless) {
        report.warnings.push_back("物理口 " + intent.iface + " 是无线接口，不建议承载雷达");
    }
    if (dry_run) {
        report.ok = true;
        return report;
    }

    for (const auto& action : report.planned) {
        Result<void> result;
        if (action.operation == "ensure_bridge") {
            result = backend.ensure_bridge(action.target);
        } else if (action.operation == "set_slave") {
            result = backend.set_slave(intent.iface, intent.bridge);
        } else if (action.operation == "set_up") {
            result = backend.set_up(action.target);
        } else if (action.operation == "add_ipv4") {
            result = backend.add_ipv4(action.target, *intent.host_ip, intent.prefix);
        } else if (action.operation == "remove_ipv4" && action.address.has_value()) {
            result = backend.remove_ipv4(action.target, *action.address, action.prefix);
        }
        if (!result.ok()) {
            report.errors.push_back(result.error());
            report.warnings.push_back("执行失败，开始回滚: " + action_detail(intent, action));
            rollback(report.applied, backend, report.warnings);
            report.ok = false;
            return report;
        }
        report.applied.push_back(action);
    }
    report.ok = true;
    return report;
}

Result<BackendSelection> select_backend(ProcessRunner& runner, Topology& topology,
                                        const NetIntent& intent) {
    BackendSelection selection;
    const auto available = network_manager_available(runner);
    if (!available.ok()) {
        return available.error();
    }
    topology.network_manager_present = available.value();

    const InterfaceInfo* iface = topology.find(intent.iface);
    if (available.value() && iface != nullptr) {
        auto backend = make_network_manager_backend(runner);
        if (backend.ok()) {
            selection.backend = std::move(backend.value());
            return selection;
        }
        selection.warnings.push_back("NetworkManager 后端初始化失败，回退 netlink: " +
                                     backend.error().message);
    } else if (!available.value()) {
        selection.warnings.push_back("未检测到可用的 NetworkManager，使用 netlink 后端");
    }

    auto backend = make_netlink_backend();
    if (!backend.ok()) {
        return backend.error();
    }
    selection.backend = std::move(backend.value());
    return selection;
}

} // namespace foray::platform::net
