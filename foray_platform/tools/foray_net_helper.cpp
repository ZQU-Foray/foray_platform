/**
 * @file foray_net_helper.cpp
 * @brief 网络自举的特权 helper：由 `pkexec` 以 root 启动，只接受白名单参数。
 *
 * 设计要点（对应 decision.md 的网络自举决策）：
 * - 不做 setuid 二进制；root 能力只来自 polkit/pkexec；
 * - 参数只有网口、网桥、主机地址/前缀、MTU 与跨桥放行开关，不执行任意命令；
 * - 先探测拓扑，再用 `ensure_link` 做幂等配置，失败自动回滚；
 * - 退出码：0 成功 / 1 用法错误 / 2 权限不足 / 3 配置失败 / 4 环境不支持。
 */

#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "foray/platform/net/backend.hpp"
#include "foray/platform/net/nm_backend.hpp"
#include "foray/platform/net/process.hpp"
#include "foray/platform/net/setup.hpp"

namespace {

namespace net = foray::platform::net;

constexpr std::uint8_t kDefaultPrefix = 24;

struct Options {
    std::string iface;
    std::string bridge;
    std::string host_ip;
    std::optional<std::uint32_t> mtu;
    bool allow_reassign_master = false;
    bool dry_run = false;
    std::vector<std::string> warnings;
};

void print_usage() {
    std::fputs(
        "用法: foray-net-helper --iface <网口> --bridge <网桥> [--host-ip <IPv4>] [--prefix <1-32>]\n"
        "                       [--mtu <576-65535>] [--allow-reassign-master] [--dry-run]\n"
        "      foray-net-helper --help\n"
        "  --mtu                    设置物理口与网桥的 MTU（失败只警告，不阻断）\n"
        "  --allow-reassign-master  允许把已属于另一个网桥的物理口抢过来\n"
        "  --dry-run                只打印计划，不修改系统（仍需要 root 以读取 NM 状态）\n",
        stderr);
}

bool valid_name(const std::string& name) {
    if (name.empty() || name.size() > 15) {
        return false;
    }
    for (const char ch : name) {
        const bool ok = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                        (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.';
        if (!ok) {
            return false;
        }
    }
    return true;
}

/// 576 = IPv4 最小重组缓冲区（RFC 791）；65535 = MTU 的 16 位上限。
bool parse_mtu(const std::string& text, std::optional<std::uint32_t>& mtu) {
    if (text.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long value = std::strtoul(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' || value < 576UL || value > 65535UL) {
        return false;
    }
    mtu = static_cast<std::uint32_t>(value);
    return true;
}

bool parse_options(int argc, char** argv, Options& options, std::optional<net::Ipv4>& host_ip,
                   std::uint8_t& prefix) {
    prefix = kDefaultPrefix;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next_value = [&](std::string& target) {
            if (i + 1 >= argc) {
                return false;
            }
            target = argv[++i];
            return true;
        };
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        } else if (arg == "--iface") {
            if (!next_value(options.iface)) {
                return false;
            }
        } else if (arg == "--bridge") {
            if (!next_value(options.bridge)) {
                return false;
            }
        } else if (arg == "--host-ip") {
            if (!next_value(options.host_ip)) {
                return false;
            }
        } else if (arg == "--mtu") {
            std::string text;
            if (!next_value(text)) {
                return false;
            }
            if (!parse_mtu(text, options.mtu)) {
                std::fprintf(stderr, "MTU 非法: %s（允许 576..65535）\n", text.c_str());
                return false;
            }
        } else if (arg == "--allow-reassign-master") {
            options.allow_reassign_master = true;
        } else if (arg == "--dry-run") {
            options.dry_run = true;
        } else if (arg == "--prefix") {
            std::string text;
            if (!next_value(text)) {
                return false;
            }
            const int value = std::atoi(text.c_str());
            if (value < 1 || value > 32) {
                std::fprintf(stderr, "前缀长度非法: %s\n", text.c_str());
                return false;
            }
            prefix = static_cast<std::uint8_t>(value);
        } else {
            std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
            return false;
        }
    }
    if (!valid_name(options.iface) || !valid_name(options.bridge)) {
        std::fputs("网口/网桥名非法（只允许 [A-Za-z0-9_.-]，最长 15 字符）\n", stderr);
        return false;
    }
    if (!options.host_ip.empty()) {
        host_ip = net::Ipv4::parse(options.host_ip);
        if (!host_ip.has_value()) {
            std::fprintf(stderr, "主机地址非法: %s\n", options.host_ip.c_str());
            return false;
        }
    }
    return true;
}

int run(const Options& options, const std::optional<net::Ipv4>& host_ip, std::uint8_t prefix) {
    if (::geteuid() != 0) {
        std::fputs("需要 root 权限：请通过 pkexec 运行（不允许普通用户直接调用）\n", stderr);
        return 2;
    }

    net::ExecProcessRunner runner;
    net::NetIntent intent{options.iface, options.bridge, host_ip, prefix, options.mtu,
                          options.allow_reassign_master};

    const auto topology = net::probe_topology();
    if (!topology.ok()) {
        std::fprintf(stderr, "拓扑探测失败: %s\n", topology.error().message.c_str());
        return 4;
    }

    net::Topology probed = topology.value();
    const auto selection = net::select_backend(runner, probed, intent);
    if (!selection.ok()) {
        std::fprintf(stderr, "网络后端不可用: %s\n", selection.error().message.c_str());
        return 4;
    }
    for (const auto& warning : selection.value().warnings) {
        std::fprintf(stderr, "warning: %s\n", warning.c_str());
    }

    const auto report =
        net::ensure_link(intent, probed, *selection.value().backend, options.dry_run);
    if (!report.ok()) {
        std::fprintf(stderr, "应用网络配置失败: %s\n", report.error().message.c_str());
        return 3;
    }

    std::printf("backend: %s\n", net::to_string(report.value().backend));
    for (const auto& warning : report.value().warnings) {
        std::printf("warning: %s\n", warning.c_str());
    }
    for (const auto& action : report.value().planned) {
        if (options.dry_run) {
            std::printf("planned: %s %s (%s)\n", action.operation.c_str(),
                        action.target.c_str(), action.detail.c_str());
        }
    }
    if (!options.dry_run) {
        for (const auto& action : report.value().applied) {
            std::printf("applied: %s %s\n", action.operation.c_str(), action.target.c_str());
        }
    }
    for (const auto& error : report.value().errors) {
        std::printf("error: %s\n", error.message.c_str());
    }
    if (!report.value().ok) {
        return 3;
    }
    std::printf("ok: %s -> %s\n", options.iface.c_str(), options.bridge.c_str());
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    std::optional<net::Ipv4> host_ip;
    std::uint8_t prefix = kDefaultPrefix;
    if (!parse_options(argc, argv, options, host_ip, prefix)) {
        print_usage();
        return 1;
    }
    return run(options, host_ip, prefix);
}
