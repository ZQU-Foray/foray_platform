#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "foray/platform/net/net_types.hpp"
#include "foray/platform/net/process.hpp"
#include "foray/platform/result/result.hpp"

namespace foray::platform::net {

/**
 * @brief 一次网络自举的输入。
 *
 * 与 `NetIntent` 的区别：这里描述「怎么调 helper」，`NetIntent` 描述「系统里应该是什么样」。
 * 调用方负责把编译期宏 `FORAY_NET_HELPER_DEFAULT` 解析成 `helper` 的绝对路径。
 */
struct BootstrapOptions {
    std::string iface;             ///< 物理网口名
    std::string bridge;            ///< 目标网桥名
    std::optional<Ipv4> host_ip;   ///< 桥上的宿主地址；空表示不配置地址
    std::uint8_t prefix = 24;      ///< 与 host_ip 配套的前缀长度
    /**
     * @brief 判定「链路已就绪」时要求的地址最小前缀长度。
     *
     * 与 `prefix` 分开：雷达沿用 `/24` 配置地址，但 `/16` 就足以判定可达，
     * 从而与 `foray_lidar_up` 原来的 `link_ready` 行为保持逐字节一致。
     */
    std::uint8_t min_prefix = 16;
    std::optional<std::uint32_t> mtu;  ///< 期望 MTU；空表示不改
    /**
     * @brief 物理口已属于另一个桥时是否允许抢过来。
     *
     * 默认拒绝；helper 也只在显式收到 `--allow-reassign-master` 时放行。
     */
    bool allow_reassign_master = false;
    bool dry_run = false;             ///< 只让 helper 打印计划
    std::string helper = "foray-net-helper";  ///< helper 路径（调用方负责宏替换）
    /// @brief 测试注入：跳过 `geteuid()` 判断，按 root 处理（不前置 pkexec）。
    bool assume_root = false;
};

/// @brief 自举结果。
struct BootstrapOutcome {
    /// @brief 链路本就满足意图，未调用 helper。
    bool skipped = false;
    /// @brief helper 的 stdout/stderr/退出码；skipped 时为空。
    ProcessOutput output;
};

/**
 * @brief 探测拓扑后自举一个物理网口。
 *
 * 链路已就绪时不调用 helper；否则非 root 时以 `pkexec` 前置，再追加
 * `--iface/--bridge[--host-ip][--prefix][--mtu][--allow-reassign-master][--dry-run]`
 * 调用 helper。
 *
 * `observed_output` 非空时，helper 的原始输出在退出码非 0 时也会写入，供调用方
 * 按原有顺序打印（`foray_lidar_up` 依赖这一点保持诊断输出不变）。
 *
 * @return 探测失败、启动 helper 失败或 helper 退出码非 0 时返回错误。
 */
Result<BootstrapOutcome> bootstrap_link(const BootstrapOptions& options, ProcessRunner& runner,
                                        ProcessOutput* observed_output = nullptr);

/**
 * @brief 以给定拓扑执行自举（测试注入用，避免真实探测）。
 *
 * 语义与上一个重载相同，只是跳过 `probe_topology()`。
 */
Result<BootstrapOutcome> bootstrap_link(const BootstrapOptions& options, const Topology& topology,
                                        ProcessRunner& runner,
                                        ProcessOutput* observed_output = nullptr);

} // namespace foray::platform::net
