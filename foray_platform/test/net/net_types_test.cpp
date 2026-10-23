#include <gtest/gtest.h>

#include "foray/platform/net/backend.hpp"
#include "foray/platform/net/net_types.hpp"

namespace foray::platform::net {
namespace {

TEST(NetTypesTest, ParsesIpv4) {
    const auto ip = Ipv4::parse("192.168.1.51");
    ASSERT_TRUE(ip.has_value());
    EXPECT_EQ(ip->octets[0], 192);
    EXPECT_EQ(ip->octets[1], 168);
    EXPECT_EQ(ip->octets[2], 1);
    EXPECT_EQ(ip->octets[3], 51);
    EXPECT_EQ(ip->to_string(), "192.168.1.51");
}

TEST(NetTypesTest, RejectsInvalidIpv4) {
    EXPECT_FALSE(Ipv4::parse("").has_value());
    EXPECT_FALSE(Ipv4::parse("192.168.1").has_value());
    EXPECT_FALSE(Ipv4::parse("192.168.1.256").has_value());
    EXPECT_FALSE(Ipv4::parse("lidar").has_value());
}

TEST(NetTypesTest, ComparesSubnets) {
    const auto host = Ipv4::parse("192.168.1.51");
    const auto lidar = Ipv4::parse("192.168.1.195");
    const auto other = Ipv4::parse("192.168.2.51");
    ASSERT_TRUE(host && lidar && other);

    EXPECT_TRUE(host->same_subnet(*lidar, 24));
    EXPECT_FALSE(host->same_subnet(*other, 24));
    EXPECT_FALSE(host->same_subnet(*other, 23));

    // 前缀落在字节中间时只比较高位。
    const auto low = Ipv4::parse("192.168.1.1");
    ASSERT_TRUE(low.has_value());
    EXPECT_TRUE(host->same_subnet(*low, 25));
    EXPECT_FALSE(host->same_subnet(*Ipv4::parse("192.168.1.200"), 25));
    EXPECT_FALSE(host->same_subnet(*lidar, 33));
}

TEST(NetTypesTest, FindsInterface) {
    Topology topology;
    InterfaceInfo info;
    info.name = "eth0";
    info.index = 2;
    info.role = LinkRole::kEthernet;
    info.up = true;
    info.carrier = true;
    topology.interfaces.push_back(info);
    EXPECT_NE(topology.find("eth0"), nullptr);
    EXPECT_EQ(topology.find("eth1"), nullptr);
}

TEST(NetTypesTest, NamesBackendKinds) {
    EXPECT_STREQ(to_string(BackendKind::kAuto), "auto");
    EXPECT_STREQ(to_string(BackendKind::kNetworkManager), "network-manager");
    EXPECT_STREQ(to_string(BackendKind::kNetlink), "netlink");
}

} // namespace
} // namespace foray::platform::net
