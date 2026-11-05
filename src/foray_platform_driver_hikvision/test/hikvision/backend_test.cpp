#include <gtest/gtest.h>

#include <memory>

#include "foray/platform/driver/hikvision/camera.hpp"

namespace foray::platform::driver::hikvision {
namespace {

hal::CameraConfig make_config() {
    hal::CameraConfig config;
    config.selector.serial = "DB0985207";
    config.width = 720;
    config.height = 540;
    return config;
}

#if FORAY_DRIVER_HIKVISION_HAS_SDK
/**
 * @brief 启用 SDK 时只验证「能构造」，**不** open。
 *
 * 单测机器上没有相机、也不该依赖现场网络；真实取流由 HIL 脚本与
 * `foray_hikvision_dump` 覆盖。
 */
TEST(HikvisionBackend, PublicFactoryConstructsWithSdk) {
    auto camera = make_camera(make_config());

    ASSERT_TRUE(camera.ok()) << camera.error().message;
    ASSERT_NE(camera.value(), nullptr);
    EXPECT_EQ(camera.value()->state(), hal::DeviceState::kClosed);
}
#else
/**
 * @brief 未启用 SDK 时后端是 stub：构造成功但 open() 报 `kSdkUnavailable`。
 *
 * 这样上层配置/状态机可以照常构建与加载，失败点集中在设备访问处。
 */
TEST(HikvisionBackend, PublicFactoryReportsSdkUnavailable) {
    auto camera = make_camera(make_config());

    ASSERT_TRUE(camera.ok()) << camera.error().message;
    const auto status = camera.value()->open();
    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kSdkUnavailable);
}
#endif

} // namespace
} // namespace foray::platform::driver::hikvision
