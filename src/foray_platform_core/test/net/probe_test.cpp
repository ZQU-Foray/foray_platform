#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "foray/platform/net/net_types.hpp"

namespace foray::platform::net {
namespace {

namespace fs = std::filesystem;

class FakeSysfs {
  public:
    FakeSysfs() {
        root_ = fs::temp_directory_path() / ("foray-net-" + std::to_string(::getpid()) + "-" +
                                             std::to_string(counter_++));
        fs::create_directories(root_);
    }
    ~FakeSysfs() {
        std::error_code ec;
        fs::remove_all(root_, ec);
    }

    const fs::path& root() const {
        return root_;
    }

    void add_dir(const std::string& name) {
        fs::create_directories(root_ / name);
    }

    void write(const std::string& name, const std::string& file, const std::string& value) {
        fs::create_directories(root_ / name);
        std::ofstream stream(root_ / name / file);
        stream << value;
    }

    void link_master(const std::string& name, const std::string& bridge) {
        fs::create_directories(root_ / name);
        fs::create_symlink(root_ / bridge, root_ / name / "master");
    }

  private:
    static inline int counter_ = 0;
    fs::path root_;
};

ProbeOptions options_for(const FakeSysfs& tree) {
    ProbeOptions options;
    options.sysfs_root = tree.root().string();
    return options;
}

TEST(ProbeTest, ClassifiesInterfaces) {
    FakeSysfs tree;
    tree.add_dir("ftbr0/bridge");
    tree.write("ftbr0", "operstate", "down\n");
    tree.write("ftbr0", "carrier", "0\n");
    tree.write("ftbr0", "address", "be:1f:3f:c7:58:a7\n");

    tree.write("fte0", "type", "1\n");
    tree.write("fte0", "operstate", "up\n");
    tree.write("fte0", "carrier", "1\n");
    tree.write("fte0", "address", "84:ba:59:d9:28:b1\n");
    tree.link_master("fte0", "ftbr0");

    tree.add_dir("ftwlan/wireless");
    tree.write("ftwlan", "type", "1\n");
    tree.write("ftwlan", "operstate", "up\n");
    tree.write("ftwlan", "carrier", "1\n");

    // 回环口应被跳过。
    tree.write("ftlo", "type", "772\n");

    const auto topology = probe_topology(options_for(tree));
    ASSERT_TRUE(topology.ok());
    ASSERT_EQ(topology.value().interfaces.size(), 3U);
    EXPECT_EQ(topology.value().interfaces[0].name, "ftbr0");
    EXPECT_EQ(topology.value().interfaces[1].name, "fte0");
    EXPECT_EQ(topology.value().interfaces[2].name, "ftwlan");

    const auto* bridge = topology.value().find("ftbr0");
    ASSERT_NE(bridge, nullptr);
    EXPECT_EQ(bridge->role, LinkRole::kBridge);
    EXPECT_FALSE(bridge->up);
    EXPECT_FALSE(bridge->carrier);

    const auto* ethernet = topology.value().find("fte0");
    ASSERT_NE(ethernet, nullptr);
    EXPECT_EQ(ethernet->role, LinkRole::kEthernet);
    EXPECT_TRUE(ethernet->up);
    EXPECT_TRUE(ethernet->carrier);
    EXPECT_EQ(ethernet->master, "ftbr0");
    EXPECT_EQ(ethernet->mac, "84:ba:59:d9:28:b1");

    const auto* wireless = topology.value().find("ftwlan");
    ASSERT_NE(wireless, nullptr);
    EXPECT_EQ(wireless->role, LinkRole::kWireless);
}

TEST(ProbeTest, MissingRootIsError) {
    ProbeOptions options;
    options.sysfs_root = "/definitely/not/a/sysfs/root";
    const auto topology = probe_topology(options);
    ASSERT_FALSE(topology.ok());
    EXPECT_EQ(topology.error().code, ErrorCode::kDeviceNotFound);
}

TEST(ProbeTest, ReadsMtuAndFallsBackToZero) {
    FakeSysfs tree;
    tree.write("fte0", "type", "1\n");
    tree.write("fte0", "mtu", "9014\n");
    // 没有 mtu 文件：老内核或非常规接口，按未知处理。
    tree.write("fte1", "type", "1\n");
    tree.write("fte2", "type", "1\n");
    tree.write("fte2", "mtu", "bogus\n");
    tree.write("fte3", "type", "1\n");
    tree.write("fte3", "mtu", "0\n");
    tree.write("fte4", "type", "1\n");
    tree.write("fte4", "mtu", "70000\n");

    const auto topology = probe_topology(options_for(tree));
    ASSERT_TRUE(topology.ok());
    ASSERT_NE(topology.value().find("fte0"), nullptr);
    EXPECT_EQ(topology.value().find("fte0")->mtu, 9014U);
    ASSERT_NE(topology.value().find("fte1"), nullptr);
    EXPECT_EQ(topology.value().find("fte1")->mtu, 0U);
    ASSERT_NE(topology.value().find("fte2"), nullptr);
    EXPECT_EQ(topology.value().find("fte2")->mtu, 0U);
    ASSERT_NE(topology.value().find("fte3"), nullptr);
    EXPECT_EQ(topology.value().find("fte3")->mtu, 0U);
    ASSERT_NE(topology.value().find("fte4"), nullptr);
    EXPECT_EQ(topology.value().find("fte4")->mtu, 0U);
}

} // namespace
} // namespace foray::platform::net
