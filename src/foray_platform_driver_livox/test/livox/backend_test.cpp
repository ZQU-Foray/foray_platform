#include <gtest/gtest.h>

#include <filesystem>
#include <memory>

#include "foray/platform/driver/livox/lidar.hpp"
#include "foray/platform/hal/config.hpp"

namespace foray::platform::driver::livox {
namespace {

hal::LidarConfig backend_config() {
    hal::LidarConfig config;
    config.selector.ip_address = "192.168.1.130";
    config.host_ip = "127.0.0.1";
    config.runtime_dir =
        (std::filesystem::temp_directory_path() / "foray_livox_backend_test").string();
    config.frame_timeout_ms = 50;
    return config;
}

#if FORAY_DRIVER_LIVOX_HAS_SDK
TEST(BackendTest, PublicFactoryConstructsWithSdk) {
    const hal::LidarConfig config = backend_config();
    Result<std::unique_ptr<hal::Lidar>> created = make_lidar(config);
    ASSERT_TRUE(created.ok()) << created.error().message;
    EXPECT_NE(created.value(), nullptr);
    // 不调用 open()：本用例只验证真实后端能构造，避免测试依赖硬件与网络。
    std::filesystem::remove_all(config.runtime_dir);
}
#else
TEST(BackendTest, PublicFactoryReportsSdkUnavailable) {
    const hal::LidarConfig config = backend_config();
    Result<std::unique_ptr<hal::Lidar>> created = make_lidar(config);
    ASSERT_TRUE(created.ok()) << created.error().message;

    const Result<void> opened = created.value()->open();
    EXPECT_FALSE(opened.ok());
    EXPECT_EQ(opened.error().code, ErrorCode::kSdkUnavailable);
    std::filesystem::remove_all(config.runtime_dir);
}
#endif

} // namespace
} // namespace foray::platform::driver::livox
