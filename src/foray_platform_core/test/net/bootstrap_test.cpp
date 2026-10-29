#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "foray/platform/net/bootstrap.hpp"

namespace foray::platform::net {
namespace {

class FakeRunner final : public ProcessRunner {
  public:
    Result<ProcessOutput> run(const std::vector<std::string>& argv) override {
        calls.push_back(argv);
        ProcessOutput output;
        output.exit_code = exit_code;
        output.out = out;
        output.err = err;
        return output;
    }

    std::vector<std::vector<std::string>> calls;
    int exit_code = 0;
    std::string out;
    std::string err;
};

InterfaceInfo make_iface(std::string name, LinkRole role, bool up, bool carrier,
                         std::string master = {}) {
    InterfaceInfo info;
    info.name = std::move(name);
    info.role = role;
    info.up = up;
    info.carrier = carrier;
    info.master = std::move(master);
    return info;
}

Topology ready_topology() {
    Topology topology;
    auto bridge = make_iface("br-camera", LinkRole::kBridge, true, true);
    bridge.mtu = 9014;
    bridge.addresses.emplace_back(*Ipv4::parse("169.254.209.1"), 16);
    topology.interfaces.push_back(std::move(bridge));
    auto port = make_iface("enp0s13f0u1", LinkRole::kEthernet, true, true, "br-camera");
    port.mtu = 9014;
    topology.interfaces.push_back(std::move(port));
    return topology;
}

BootstrapOptions camera_options() {
    BootstrapOptions options;
    options.iface = "enp0s13f0u1";
    options.bridge = "br-camera";
    options.host_ip = Ipv4::parse("169.254.209.1");
    options.prefix = 16;
    options.min_prefix = 16;
    options.mtu = 9014;
    options.assume_root = true;
    return options;
}

TEST(BootstrapTest, SkipsWhenLinkReady) {
    FakeRunner runner;
    const auto result = bootstrap_link(camera_options(), ready_topology(), runner);
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.value().skipped);
    EXPECT_TRUE(result.value().output.out.empty());
    EXPECT_TRUE(runner.calls.empty());
}

TEST(BootstrapTest, InvokesHelperWithAllArguments) {
    FakeRunner runner;
    const auto result = bootstrap_link(camera_options(), Topology{}, runner);
    ASSERT_TRUE(result.ok());
    ASSERT_EQ(runner.calls.size(), 1U);
    const std::vector<std::string> expected = {
        "foray-net-helper", "--iface",   "enp0s13f0u1",    "--bridge", "br-camera",
        "--host-ip",        "169.254.209.1", "--prefix",  "16",       "--mtu",
        "9014"};
    EXPECT_EQ(runner.calls[0], expected);
    EXPECT_FALSE(result.value().skipped);
}

TEST(BootstrapTest, PropagatesHelperFailure) {
    FakeRunner runner;
    runner.exit_code = 3;
    runner.out = "planned: set_slave eth0 br-lidar\n";
    runner.err = "错误: 物理口已属于 br-lidar\n";
    ProcessOutput observed;

    const auto result = bootstrap_link(camera_options(), Topology{}, runner, &observed);
    ASSERT_FALSE(result.ok());
    EXPECT_NE(result.error().message.find("3"), std::string::npos);
    EXPECT_EQ(observed.exit_code, 3);
    EXPECT_EQ(observed.out, runner.out);
    EXPECT_EQ(observed.err, runner.err);
}

TEST(BootstrapTest, AppendsDryRunAndReassignFlags) {
    FakeRunner runner;
    auto options = camera_options();
    options.dry_run = true;
    options.allow_reassign_master = true;
    const auto result = bootstrap_link(options, Topology{}, runner);
    ASSERT_TRUE(result.ok());
    ASSERT_EQ(runner.calls.size(), 1U);
    const auto& argv = runner.calls[0];
    ASSERT_GE(argv.size(), 3U);
    EXPECT_EQ(argv[argv.size() - 2], "--allow-reassign-master");
    EXPECT_EQ(argv.back(), "--dry-run");
}

TEST(BootstrapTest, OmitsHostArgumentsWhenUnset) {
    FakeRunner runner;
    auto options = camera_options();
    options.host_ip.reset();
    const auto result = bootstrap_link(options, Topology{}, runner);
    ASSERT_TRUE(result.ok());
    ASSERT_EQ(runner.calls.size(), 1U);
    const auto& argv = runner.calls[0];
    EXPECT_EQ(std::find(argv.begin(), argv.end(), "--host-ip"), argv.end());
    EXPECT_EQ(std::find(argv.begin(), argv.end(), "--prefix"), argv.end());
}

} // namespace
} // namespace foray::platform::net
