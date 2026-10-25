#include "hikvision/camera_bootstrap.hpp"

#include <gtest/gtest.h>

#include <string>

#include "foray/platform/net/net_types.hpp"

namespace foray::platform::driver::hikvision::detail {
namespace {

using net::InterfaceInfo;
using net::Ipv4;

InterfaceInfo bridge_with(std::initializer_list<std::pair<const char*, std::uint8_t>> addresses) {
    InterfaceInfo bridge;
    bridge.name = "br-camera";
    for (const auto& [text, prefix] : addresses) {
        const auto parsed = Ipv4::parse(text);
        EXPECT_TRUE(parsed.has_value());
        if (parsed.has_value()) {
            bridge.addresses.emplace_back(*parsed, prefix);
        }
    }
    return bridge;
}

TEST(PlanHostAddress, LinkLocalCameraWithoutBridgeUsesDefaultLinkLocalAddress) {
    const auto plan = plan_host_address("169.254.1.100", nullptr);

    ASSERT_TRUE(plan.host_ip.has_value());
    EXPECT_EQ(plan.host_ip->to_string(), "169.254.209.1");
    EXPECT_EQ(plan.prefix, 16);
    EXPECT_FALSE(plan.reason.empty());
}

TEST(PlanHostAddress, ReusesLinkLocalAddressAlreadyOnBridge) {
    const auto bridge = bridge_with({{"169.254.1.1", 16}});

    const auto plan = plan_host_address("169.254.1.100", &bridge);

    ASSERT_TRUE(plan.host_ip.has_value());
    EXPECT_EQ(plan.host_ip->to_string(), "169.254.1.1");
    EXPECT_EQ(plan.prefix, 16);
}

TEST(PlanHostAddress, FixedCameraGetsSameSubnetHostAddress) {
    const auto plan = plan_host_address("192.168.1.100", nullptr);

    ASSERT_TRUE(plan.host_ip.has_value());
    EXPECT_EQ(plan.host_ip->to_string(), "192.168.1.1");
    EXPECT_EQ(plan.prefix, 24);
}

TEST(PlanHostAddress, AvoidsCameraAddressWhenCameraIsDotOne) {
    const auto plan = plan_host_address("192.168.1.1", nullptr);

    ASSERT_TRUE(plan.host_ip.has_value());
    EXPECT_EQ(plan.host_ip->to_string(), "192.168.1.254");
    EXPECT_EQ(plan.prefix, 24);
}

TEST(PlanHostAddress, UnknownCameraWithoutBridgeFallsBackToLinkLocal) {
    const auto plan = plan_host_address("", nullptr);

    ASSERT_TRUE(plan.host_ip.has_value());
    EXPECT_EQ(plan.host_ip->to_string(), "169.254.209.1");
    EXPECT_EQ(plan.prefix, 16);
    EXPECT_FALSE(plan.reason.empty());
}

TEST(PlanHostAddress, AvoidsDefaultHostWhenCameraAlreadyUsesIt) {
    const auto plan = plan_host_address("169.254.209.1", nullptr);

    ASSERT_TRUE(plan.host_ip.has_value());
    EXPECT_EQ(plan.host_ip->to_string(), "169.254.209.254");
    EXPECT_EQ(plan.prefix, 16);
}

TEST(PlanHostAddress, InvalidDeviceAddressTreatedAsUnknown) {
    const auto plan = plan_host_address("not-an-ip", nullptr);

    ASSERT_TRUE(plan.host_ip.has_value());
    EXPECT_EQ(plan.host_ip->to_string(), "169.254.209.1");
    EXPECT_EQ(plan.prefix, 16);
}

} // namespace
} // namespace foray::platform::driver::hikvision::detail
