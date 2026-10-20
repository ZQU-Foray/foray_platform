#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "livox/livox_control.hpp"
#include "mock_livox_api.hpp"

namespace foray::platform::driver::livox::detail {
namespace {

using test::MockLivoxApi;

TEST(LivoxControlTest, RetriesUntilSuccess) {
    auto api = std::make_shared<MockLivoxApi>();
    api->control_failures = 2;
    LivoxControl control(*api, ControlPolicy{3, std::chrono::milliseconds{0}});
    const auto result = control.set_lidar_ip(7, "192.168.1.195", "255.255.255.0",
                                             "192.168.1.1");
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(api->control_calls, 3);
    EXPECT_EQ(api->lidar_ip, "192.168.1.195");
    EXPECT_EQ(api->lidar_net_mask, "255.255.255.0");
    EXPECT_EQ(api->last_handle, 7U);
}

TEST(LivoxControlTest, GivesUpAfterMaxAttempts) {
    auto api = std::make_shared<MockLivoxApi>();
    api->control_failures = 10;
    LivoxControl control(*api, ControlPolicy{2, std::chrono::milliseconds{0}});
    const auto result = control.set_pps_sync_mode(7, 0x01);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kTimeout);
    EXPECT_EQ(api->control_calls, 2);
}

TEST(LivoxControlTest, RecordsAttitudeAndPortConfig) {
    auto api = std::make_shared<MockLivoxApi>();
    LivoxControl control(*api, ControlPolicy{1, std::chrono::milliseconds{0}});
    ASSERT_TRUE(control.set_install_attitude(7, 1.5F, -2.0F, 90.0F, 10, -20, 30).ok());
    EXPECT_FLOAT_EQ(api->attitude[0], 1.5F);
    EXPECT_FLOAT_EQ(api->attitude[2], 90.0F);
    EXPECT_EQ(api->attitude[5], 30.0F);

    ASSERT_TRUE(control.set_host_port_cfg(7, "192.168.1.51", 56100, 56001).ok());
    EXPECT_EQ(api->host_ip_cfg, "192.168.1.51");
    EXPECT_EQ(api->host_port_cfg, 56100);
    EXPECT_EQ(api->lidar_port_cfg, 56001);
}

TEST(LivoxControlTest, VerifiesReadbackIp) {
    auto api = std::make_shared<MockLivoxApi>();
    api->reported_ip = "192.168.1.195";
    LivoxControl control(*api);
    EXPECT_TRUE(control.verify_lidar_ip(3, "192.168.1.195").ok());

    const auto mismatched = control.verify_lidar_ip(3, "192.168.1.196");
    ASSERT_FALSE(mismatched.ok());
    EXPECT_EQ(mismatched.error().code, ErrorCode::kSdkError);
    EXPECT_EQ(api->last_handle, 3U);
}

TEST(LivoxControlTest, ReportsMissingReadback) {
    auto api = std::make_shared<MockLivoxApi>();
    LivoxControl control(*api);
    const auto result = control.verify_lidar_ip(3, "192.168.1.195");
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kNoData);
    EXPECT_EQ(api->query_calls, 1);
}

TEST(LivoxControlTest, RetriesReadbackAfterTimeout) {
    auto api = std::make_shared<MockLivoxApi>();
    api->query_failures = 1;
    api->reported_ip = "192.168.1.195";
    LivoxControl control(*api);
    EXPECT_FALSE(control.verify_lidar_ip(3, "192.168.1.195").ok());
    EXPECT_EQ(api->query_calls, 1);
    EXPECT_TRUE(control.verify_lidar_ip(3, "192.168.1.195").ok());
    EXPECT_EQ(api->query_calls, 2);
}

} // namespace
} // namespace foray::platform::driver::livox::detail
