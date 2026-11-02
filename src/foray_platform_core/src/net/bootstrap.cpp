#include "foray/platform/net/bootstrap.hpp"

#include <unistd.h>

#include <string>
#include <utility>
#include <vector>

#include "foray/platform/net/setup.hpp"

namespace foray::platform::net {

Result<BootstrapOutcome> bootstrap_link(const BootstrapOptions& options, ProcessRunner& runner,
                                        ProcessOutput* observed_output) {
    const auto topology = probe_topology();
    if (!topology.ok()) {
        return Error{topology.error().code, "拓扑探测失败: " + topology.error().message};
    }
    return bootstrap_link(options, topology.value(), runner, observed_output);
}

Result<BootstrapOutcome> bootstrap_link(const BootstrapOptions& options, const Topology& topology,
                                        ProcessRunner& runner,
                                        ProcessOutput* observed_output) {
    if (link_ready(topology, options.iface, options.bridge, options.host_ip, options.min_prefix,
                   options.mtu)) {
        BootstrapOutcome outcome;
        outcome.skipped = true;
        return outcome;
    }

    std::vector<std::string> argv;
    if (!options.assume_root && ::geteuid() != 0) {
        // 非 root 时走 polkit 图形授权，不要求操作手在终端敲密码。
        argv.emplace_back("pkexec");
    }
    argv.push_back(options.helper);
    argv.insert(argv.end(), {"--iface", options.iface, "--bridge", options.bridge});
    if (options.host_ip.has_value()) {
        argv.insert(argv.end(), {"--host-ip", options.host_ip->to_string()});
        argv.insert(argv.end(), {"--prefix", std::to_string(options.prefix)});
    }
    if (options.mtu.has_value()) {
        argv.insert(argv.end(), {"--mtu", std::to_string(*options.mtu)});
    }
    if (options.allow_reassign_master) {
        argv.emplace_back("--allow-reassign-master");
    }
    if (options.dry_run) {
        argv.emplace_back("--dry-run");
    }

    const auto executed = runner.run(argv);
    if (!executed.ok()) {
        return Error{executed.error().code, "调用网络自举失败: " + executed.error().message};
    }

    BootstrapOutcome outcome;
    outcome.output = executed.value();
    if (observed_output != nullptr) {
        *observed_output = outcome.output;
    }
    if (outcome.output.exit_code != 0) {
        return Error{ErrorCode::kInternal,
                     "网络自举退出码 " + std::to_string(outcome.output.exit_code) +
                         "（可用 --helper 指定路径）"};
    }
    return outcome;
}

} // namespace foray::platform::net
