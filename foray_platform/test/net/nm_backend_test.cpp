#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "foray/platform/net/nm_backend.hpp"
#include "foray/platform/net/process.hpp"

namespace foray::platform::net {
namespace {

class FakeRunner final : public ProcessRunner {
  public:
    Result<ProcessOutput> run(const std::vector<std::string>& argv) override {
        std::string key;
        for (const auto& arg : argv) {
            if (!key.empty()) {
                key.push_back(' ');
            }
            key += arg;
        }
        calls.push_back(key);
        const auto it = responses.find(key);
        if (it == responses.end()) {
            return ProcessOutput{0, {}, {}};
        }
        return it->second;
    }

    std::vector<std::string> calls;
    std::map<std::string, ProcessOutput> responses;

    bool called(const std::string& key) const {
        for (const auto& call : calls) {
            if (call == key) {
                return true;
            }
        }
        return false;
    }
};

constexpr const char* kList = "nmcli --colors no -t -f NAME,UUID,TYPE,DEVICE connection show";

FakeRunner running_manager() {
    FakeRunner runner;
    runner.responses["nmcli --version"] = ProcessOutput{0, "nmcli tool, version 1.52.0\n", {}};
    runner.responses["nmcli -t -f RUNNING general"] = ProcessOutput{0, "running\n", {}};
    return runner;
}

TEST(NmBackendTest, DetectsRunningManager) {
    auto runner = running_manager();
    const auto available = network_manager_available(runner);
    ASSERT_TRUE(available.ok());
    EXPECT_TRUE(available.value());
}

TEST(NmBackendTest, DetectsStoppedManager) {
    FakeRunner runner;
    runner.responses["nmcli --version"] = ProcessOutput{0, "nmcli tool\n", {}};
    runner.responses["nmcli -t -f RUNNING general"] = ProcessOutput{0, "off\n", {}};
    const auto available = network_manager_available(runner);
    ASSERT_TRUE(available.ok());
    EXPECT_FALSE(available.value());
}

TEST(NmBackendTest, ReportsMissingNmcli) {
    FakeRunner runner;
    runner.responses["nmcli --version"] = ProcessOutput{127, {}, "未找到命令"};
    const auto available = network_manager_available(runner);
    ASSERT_TRUE(available.ok());
    EXPECT_FALSE(available.value());

    const auto backend = make_network_manager_backend(runner);
    ASSERT_FALSE(backend.ok());
    EXPECT_EQ(backend.error().code, ErrorCode::kNotSupported);
}

TEST(NmBackendTest, SkipsExistingAddress) {
    auto runner = running_manager();
    runner.responses[kList] = ProcessOutput{0, "br-lidar:uuid-1:bridge:br-lidar\n", {}};
    runner.responses["nmcli --colors no -t -f IP4.ADDRESS device show br-lidar"] =
        ProcessOutput{0, "IP4.ADDRESS[1]:192.168.1.51/24\n", {}};

    auto backend = make_network_manager_backend(runner);
    ASSERT_TRUE(backend.ok());
    const auto added = backend.value()->add_ipv4("br-lidar", *Ipv4::parse("192.168.1.51"), 24);
    ASSERT_TRUE(added.ok());
    EXPECT_FALSE(runner.called("nmcli --colors no connection modify br-lidar +ipv4.addresses "
                               "192.168.1.51/24"));
}

TEST(NmBackendTest, AddsNewAddress) {
    auto runner = running_manager();
    runner.responses[kList] = ProcessOutput{0, "br-lidar:uuid-1:bridge:br-lidar\n", {}};

    auto backend = make_network_manager_backend(runner);
    ASSERT_TRUE(backend.ok());
    const auto added = backend.value()->add_ipv4("br-lidar", *Ipv4::parse("192.168.1.51"), 24);
    ASSERT_TRUE(added.ok());
    EXPECT_TRUE(runner.called("nmcli --colors no connection modify br-lidar +ipv4.addresses "
                              "192.168.1.51/24"));
    EXPECT_TRUE(runner.called("nmcli --colors no connection modify br-lidar ipv4.method manual"));
    EXPECT_TRUE(runner.called("nmcli --colors no connection up br-lidar"));
}

TEST(NmBackendTest, CreatesSlaveProfile) {
    auto runner = running_manager();
    runner.responses[kList] = ProcessOutput{0, "br-lidar:uuid-1:bridge:br-lidar\n", {}};

    auto backend = make_network_manager_backend(runner);
    ASSERT_TRUE(backend.ok());
    const auto slaved = backend.value()->set_slave("enp0s31f6", "br-lidar");
    ASSERT_TRUE(slaved.ok());
    EXPECT_TRUE(runner.called("nmcli --colors no connection add type bridge-slave ifname "
                              "enp0s31f6 master br-lidar con-name foray-enp0s31f6-slave "
                              "autoconnect yes"));
    EXPECT_TRUE(runner.called("nmcli --colors no connection up foray-enp0s31f6-slave"));
}

TEST(NmBackendTest, CreatesBridgeWhenMissing) {
    auto runner = running_manager();
    auto backend = make_network_manager_backend(runner);
    ASSERT_TRUE(backend.ok());
    const auto created = backend.value()->ensure_bridge("br-lidar");
    ASSERT_TRUE(created.ok());
    EXPECT_TRUE(runner.called("nmcli --colors no connection add type bridge ifname br-lidar "
                              "con-name br-lidar autoconnect yes"));
}

TEST(NmBackendTest, ClearSlaveDeletesOwnProfile) {
    auto runner = running_manager();
    runner.responses[kList] =
        ProcessOutput{0, "foray-enp0s31f6-slave:uuid-2:bridge-slave:enp0s31f6\n", {}};
    auto backend = make_network_manager_backend(runner);
    ASSERT_TRUE(backend.ok());
    const auto cleared = backend.value()->clear_slave("enp0s31f6");
    ASSERT_TRUE(cleared.ok());
    EXPECT_TRUE(runner.called("nmcli --colors no connection delete foray-enp0s31f6-slave"));
}

TEST(NmBackendTest, ClearSlaveKeepsForeignProfile) {
    auto runner = running_manager();
    runner.responses[kList] = ProcessOutput{0, "有线连接 1:uuid-3:bridge-slave:enp0s31f6\n", {}};
    auto backend = make_network_manager_backend(runner);
    ASSERT_TRUE(backend.ok());
    const auto cleared = backend.value()->clear_slave("enp0s31f6");
    ASSERT_TRUE(cleared.ok());
    EXPECT_TRUE(runner.called("nmcli --colors no connection down 有线连接 1"));
    EXPECT_FALSE(runner.called("nmcli --colors no connection delete 有线连接 1"));
}

} // namespace
} // namespace foray::platform::net
