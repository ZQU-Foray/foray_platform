#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "foray/platform/net/setup.hpp"

namespace foray::platform::net {
namespace {

class FakeBackend final : public NetBackend {
  public:
    BackendKind kind() const override {
        return BackendKind::kNetlink;
    }

    Result<void> ensure_bridge(const std::string& bridge) override {
        return record("ensure_bridge " + bridge);
    }
    Result<void> set_slave(const std::string& iface, const std::string& bridge) override {
        return record("set_slave " + iface + " " + bridge);
    }
    Result<void> clear_slave(const std::string& iface) override {
        return record("clear_slave " + iface);
    }
    Result<void> set_up(const std::string& name) override {
        return record("set_up " + name);
    }
    Result<void> add_ipv4(const std::string& name, const Ipv4& ip, std::uint8_t prefix) override {
        return record("add_ipv4 " + name + " " + ip.to_string() + "/" +
                      std::to_string(static_cast<int>(prefix)));
    }
    Result<void> remove_ipv4(const std::string& name, const Ipv4& ip, std::uint8_t prefix) override {
        return record("remove_ipv4 " + name + " " + ip.to_string() + "/" +
                      std::to_string(static_cast<int>(prefix)));
    }

    std::vector<std::string> calls;
    /// 命中该前缀的调用返回失败（用于验证回滚）。
    std::string fail_prefix;

  private:
    Result<void> record(const std::string& call) {
        calls.push_back(call);
        if (!fail_prefix.empty() && call.rfind(fail_prefix, 0) == 0) {
            return Error{ErrorCode::kInternal, "注入失败: " + call};
        }
        return Result<void>();
    }
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

Topology topology_with_port() {
    Topology topology;
    topology.interfaces.push_back(make_iface("eth0", LinkRole::kEthernet, true, true));
    return topology;
}

Ipv4 ip_of(const char* text) {
    const auto parsed = Ipv4::parse(text);
    return parsed.value_or(Ipv4{});
}

TEST(SetupTest, PlansFullBootstrap) {
    NetIntent intent{"eth0", "br-lidar", ip_of("192.168.1.51"), 24};
    const auto planned = plan_link(intent, topology_with_port());
    ASSERT_TRUE(planned.ok());
    ASSERT_EQ(planned.value().size(), 4U);
    EXPECT_EQ(planned.value()[0].operation, "ensure_bridge");
    EXPECT_EQ(planned.value()[1].operation, "set_slave");
    EXPECT_EQ(planned.value()[2].operation, "set_up");
    EXPECT_EQ(planned.value()[3].operation, "add_ipv4");
    ASSERT_TRUE(planned.value()[3].address.has_value());
    EXPECT_EQ(planned.value()[3].address->to_string(), "192.168.1.51");
    EXPECT_EQ(planned.value()[3].prefix, 24);
}

TEST(SetupTest, PlansNothingWhenAlreadyConfigured) {
    Topology topology;
    auto bridge = make_iface("br-lidar", LinkRole::kBridge, true, true);
    bridge.addresses.emplace_back(ip_of("192.168.1.51"), 24);
    topology.interfaces.push_back(bridge);
    topology.interfaces.push_back(make_iface("eth0", LinkRole::kEthernet, true, true, "br-lidar"));

    NetIntent intent{"eth0", "br-lidar", ip_of("192.168.1.51"), 24};
    const auto planned = plan_link(intent, topology);
    ASSERT_TRUE(planned.ok());
    EXPECT_TRUE(planned.value().empty());
}

TEST(SetupTest, RejectsBadIntent) {
    NetIntent empty_iface{"", "br-lidar", ip_of("192.168.1.51"), 24};
    EXPECT_FALSE(plan_link(empty_iface, topology_with_port()).ok());

    NetIntent same{"eth0", "eth0", std::nullopt, 24};
    EXPECT_FALSE(plan_link(same, topology_with_port()).ok());

    NetIntent missing{"eth9", "br-lidar", std::nullopt, 24};
    const auto planned = plan_link(missing, topology_with_port());
    ASSERT_FALSE(planned.ok());
    EXPECT_EQ(planned.error().code, ErrorCode::kDeviceNotFound);

    NetIntent bad_prefix{"eth0", "br-lidar", ip_of("192.168.1.51"), 33};
    EXPECT_FALSE(plan_link(bad_prefix, topology_with_port()).ok());

    Topology bridge_as_port;
    bridge_as_port.interfaces.push_back(make_iface("br0", LinkRole::kBridge, true, true));
    NetIntent bridge_iface{"br0", "br-lidar", std::nullopt, 24};
    EXPECT_FALSE(plan_link(bridge_iface, bridge_as_port).ok());
}

TEST(SetupTest, DryRunDoesNotTouchBackend) {
    NetIntent intent{"eth0", "br-lidar", ip_of("192.168.1.51"), 24};
    FakeBackend backend;
    const auto report = ensure_link(intent, topology_with_port(), backend, true);
    ASSERT_TRUE(report.ok());
    EXPECT_TRUE(report.value().ok);
    EXPECT_TRUE(report.value().dry_run);
    EXPECT_EQ(report.value().planned.size(), 4U);
    EXPECT_TRUE(report.value().applied.empty());
    EXPECT_TRUE(backend.calls.empty());
}

TEST(SetupTest, AppliesPlannedActionsInOrder) {
    NetIntent intent{"eth0", "br-lidar", ip_of("192.168.1.51"), 24};
    FakeBackend backend;
    const auto report = ensure_link(intent, topology_with_port(), backend);
    ASSERT_TRUE(report.ok());
    EXPECT_TRUE(report.value().ok);
    ASSERT_EQ(backend.calls.size(), 4U);
    EXPECT_EQ(backend.calls[0], "ensure_bridge br-lidar");
    EXPECT_EQ(backend.calls[1], "set_slave eth0 br-lidar");
    EXPECT_EQ(backend.calls[2], "set_up br-lidar");
    EXPECT_EQ(backend.calls[3], "add_ipv4 br-lidar 192.168.1.51/24");
    EXPECT_EQ(report.value().backend, BackendKind::kNetlink);
}

TEST(SetupTest, RollsBackOnFailure) {
    NetIntent intent{"eth0", "br-lidar", ip_of("192.168.1.51"), 24};
    FakeBackend backend;
    backend.fail_prefix = "set_up";
    const auto report = ensure_link(intent, topology_with_port(), backend);
    ASSERT_TRUE(report.ok());
    EXPECT_FALSE(report.value().ok);
    ASSERT_FALSE(report.value().errors.empty());
    // 已完成 ensure_bridge、set_slave；set_up 失败后回滚 set_slave。
    EXPECT_EQ(backend.calls.back(), "clear_slave eth0");
    EXPECT_EQ(report.value().applied.size(), 2U);
}

TEST(SetupTest, WarnsWhenPortHasNoCarrier) {
    Topology topology;
    topology.interfaces.push_back(make_iface("eth0", LinkRole::kEthernet, true, false));
    NetIntent intent{"eth0", "br-lidar", std::nullopt, 24};
    FakeBackend backend;
    const auto report = ensure_link(intent, topology, backend, true);
    ASSERT_TRUE(report.ok());
    ASSERT_EQ(report.value().warnings.size(), 1U);
    EXPECT_NE(report.value().warnings[0].find("没有载波"), std::string::npos);
}

} // namespace
} // namespace foray::platform::net

TEST(SetupTest, LinkReadyChecksMasterCarrierAndAddress) {
    foray::platform::net::Topology topology;
    foray::platform::net::InterfaceInfo bridge;
    bridge.name = "br-lidar";
    bridge.role = foray::platform::net::LinkRole::kBridge;
    bridge.up = true;
    bridge.carrier = true;
    bridge.addresses.emplace_back(*foray::platform::net::Ipv4::parse("192.168.1.51"), 24);
    foray::platform::net::InterfaceInfo port;
    port.name = "enp0s31f6";
    port.role = foray::platform::net::LinkRole::kEthernet;
    port.up = true;
    port.master = "br-lidar";
    topology.interfaces = {bridge, port};

    const auto ip = *foray::platform::net::Ipv4::parse("192.168.1.51");
    EXPECT_TRUE(foray::platform::net::link_ready(topology, "enp0s31f6", "br-lidar", ip));
    // 前缀过短（/8）不算就绪：说明 IP 配在了错误的掩码上。
    EXPECT_FALSE(foray::platform::net::link_ready(topology, "enp0s31f6", "br-lidar", ip, 25));

    topology.interfaces[0].carrier = false;
    EXPECT_FALSE(foray::platform::net::link_ready(topology, "enp0s31f6", "br-lidar", ip));
    topology.interfaces[0].carrier = true;

    topology.interfaces[1].master.clear();
    EXPECT_FALSE(foray::platform::net::link_ready(topology, "enp0s31f6", "br-lidar", ip));
}

TEST(SetupTest, ReplacesStaleAddressInSameSubnet) {
    foray::platform::net::Topology topology;
    foray::platform::net::InterfaceInfo bridge;
    bridge.name = "br-lidar";
    bridge.role = foray::platform::net::LinkRole::kBridge;
    bridge.up = true;
    bridge.carrier = true;
    bridge.addresses.emplace_back(*foray::platform::net::Ipv4::parse("192.168.1.99"), 24);
    foray::platform::net::InterfaceInfo port;
    port.name = "eth0";
    port.role = foray::platform::net::LinkRole::kEthernet;
    port.up = true;
    port.carrier = true;
    port.master = "br-lidar";
    topology.interfaces = {bridge, port};

    foray::platform::net::NetIntent intent;
    intent.iface = "eth0";
    intent.bridge = "br-lidar";
    intent.host_ip = foray::platform::net::Ipv4::parse("192.168.1.51");
    intent.prefix = 24;

    const auto planned = foray::platform::net::plan_link(intent, topology);
    ASSERT_TRUE(planned.ok()) << planned.error().message;
    ASSERT_EQ(planned.value().size(), 2U);
    EXPECT_EQ(planned.value()[0].operation, "remove_ipv4");
    EXPECT_EQ(planned.value()[0].address->to_string(), "192.168.1.99");
    EXPECT_EQ(planned.value()[1].operation, "add_ipv4");

    foray::platform::test::FakeBackend backend;
    const auto report = foray::platform::net::ensure_link(intent, topology, backend, false);
    ASSERT_TRUE(report.ok()) << report.error().message;
    ASSERT_EQ(backend.calls.size(), 2U);
    EXPECT_EQ(backend.calls[0], "remove_ipv4 br-lidar 192.168.1.99/24");
    EXPECT_EQ(backend.calls[1], "add_ipv4 br-lidar 192.168.1.51/24");
}
