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
    std::string iface;                  ///< 雷达/相机接出的物理网口
    std::string bridge;                 ///< 桥名（不存在时创建）
    std::optional<Ipv4> host_ip;        ///< 桥上的宿主地址；空表示不配置
    std::uint8_t prefix = 24;           ///< 与 host_ip 配套的前缀长度
    /// @brief 期望的 MTU；空表示不改。物理口先设、桥后设。
    std::optional<std::uint32_t> mtu;
    /**
     * @brief 物理口已属于另一个桥时是否允许抢过来。
     *
     * 默认拒绝：现场同时存在 br-lidar / br-camera 时，静默抢占会让另一台设备
     * 无声掉线且原因难查；显式传 true 才重新挂桥。
     */
    bool allow_reassign_master = false;
};

/// 规划出的一步操作。
struct LinkAction {
    std::string operation; ///< ensure_bridge / set_slave / set_up / set_mtu / add_ipv4
    std::string target;
    std::string detail;
    /// @brief add_ipv4 专用：地址与前缀，回滚时原样撤销。
    std::optional<Ipv4> address;
    std::uint8_t prefix = 0;
    /// @brief set_mtu 专用：目标 MTU。
    std::uint32_t mtu = 0;
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
 * 条件：物理口挂在指定桥下、桥 `up` 且有载波；`host_ip` 非空时还要求桥上已有
 * 同地址且前缀不短于 `min_prefix`；`min_mtu` 非空时要求物理口与桥的 MTU 都不小于
 * 该值（0 表示探测未读到，按未知处理、不判定失败）。
 *
 * 探测阶段还不知道设备地址时传空的 `host_ip`，只校验「挂桥 + master up + carrier」。
 */
bool link_ready(const Topology& topology, const std::string& iface,
                const std::string& bridge,
                std::optional<Ipv4> host_ip = std::nullopt,
                std::uint8_t min_prefix = 16,
                std::optional<std::uint32_t> min_mtu = std::nullopt);

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
