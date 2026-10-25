#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "foray/platform/net/backend.hpp"
#include "foray/platform/net/net_types.hpp"
#include "foray/platform/result/result.hpp"

namespace foray::platform::net {

class ProcessRunner;

/// 期望的网络拓扑：物理口 → 桥 → 宿主 IP。
struct NetIntent {
    std::string iface;                  ///< 雷达接出的物理网口
    std::string bridge;                 ///< 桥名（不存在时创建）
    std::optional<Ipv4> host_ip;        ///< 桥上的宿主地址；空表示不配置
    std::uint8_t prefix = 24;           ///< 与 host_ip 配套的前缀长度
};

/// 规划出的一步操作。
struct LinkAction {
    std::string operation; ///< ensure_bridge / set_slave / set_up / add_ipv4
    std::string target;
    std::string detail;
    /// @brief add_ipv4 专用：地址与前缀，回滚时原样撤销。
    std::optional<Ipv4> address;
    std::uint8_t prefix = 0;
};

/// 一次网络自举的结果；`planned` 与 `applied` 分开，dry-run 只填前者。
struct LinkReport {
    bool ok = false;
    bool dry_run = false;
    BackendKind backend = BackendKind::kAuto;
    std::vector<LinkAction> planned;
    std::vector<LinkAction> applied;
    std::vector<std::string> warnings;
    std::vector<Error> errors;
};

/**
 * @brief 判定链路是否已经满足意图（纯函数，不做 I/O）。
 *
 * 条件：物理口挂在指定桥下、桥 `up` 且有载波、主 IP 已配置且前缀不短于
 * `min_prefix`。`foray_lidar_up` 用它决定是否还需要跑特权 helper。
 */
bool link_ready(const Topology& topology, const std::string& iface,
                const std::string& bridge, const Ipv4& host_ip,
                std::uint8_t min_prefix = 16);

/// @brief 把期望拓扑与现状求差，得到有序操作列表（纯函数，不做任何 I/O）。
Result<std::vector<LinkAction>> plan_link(const NetIntent& intent, const Topology& topology);

/**
 * @brief 执行规划并处理回滚。
 *
 * 某一步失败时按逆序撤销已完成的操作（set_slave → clear_slave、
 * add_ipv4 → remove_ipv4），失败原因记录在 `errors`。
 */
Result<LinkReport> ensure_link(const NetIntent& intent, NetBackend& backend, bool dry_run = false);

/// @brief 以给定拓扑执行规划（测试注入用，避免真实探测）。
Result<LinkReport> ensure_link(const NetIntent& intent, const Topology& topology,
                               NetBackend& backend, bool dry_run = false);

/// 后端选择结果。
struct BackendSelection {
    std::unique_ptr<NetBackend> backend;
    std::vector<std::string> warnings;
};

/**
 * @brief 选择后端并补全拓扑里的 NetworkManager 信息。
 *
 * NM 可用（nmcli + NetworkManager 运行中）时优先 NM，否则退回 netlink。
 */
Result<BackendSelection> select_backend(ProcessRunner& runner, Topology& topology,
                                        const NetIntent& intent);

} // namespace foray::platform::net
