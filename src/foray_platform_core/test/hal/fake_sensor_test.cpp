#include <gtest/gtest.h>

#include <optional>

#include "fake_sensor.hpp"
#include "foray/platform/time/duration.hpp"

namespace foray::platform::hal {
namespace {

const time::Duration kTimeout = time::Duration::from_milliseconds(1);

TEST(FakeCameraTest, LifecycleTransitions) {
    test::FakeCamera camera;

    EXPECT_EQ(camera.state(), DeviceState::kClosed);
    ASSERT_TRUE(camera.open().ok());
    EXPECT_EQ(camera.state(), DeviceState::kOpen);
    ASSERT_TRUE(camera.start().ok());
    EXPECT_EQ(camera.state(), DeviceState::kStreaming);
    ASSERT_TRUE(camera.stop().ok());
    EXPECT_EQ(camera.state(), DeviceState::kOpen);
    ASSERT_TRUE(camera.close().ok());
    EXPECT_EQ(camera.state(), DeviceState::kClosed);
    EXPECT_TRUE(camera.close().ok());
}

TEST(FakeCameraTest, ReadRequiresStreaming) {
    test::FakeCamera camera;
    FrameLease<ImageFrame> lease;

    EXPECT_EQ(camera.read(lease, kTimeout).error().code, ErrorCode::kNotOpen);

    ASSERT_TRUE(camera.open().ok());
    EXPECT_EQ(camera.read(lease, kTimeout).error().code, ErrorCode::kNotOpen);
}

TEST(FakeCameraTest, ReadTimeoutAndInjectedFrame) {
    test::FakeCamera camera;
    ASSERT_TRUE(camera.open().ok());
    ASSERT_TRUE(camera.start().ok());

    FrameLease<ImageFrame> lease;
    const Result<void> timeout = camera.read(lease, kTimeout);
    ASSERT_FALSE(timeout.ok());
    EXPECT_EQ(timeout.error().code, ErrorCode::kTimeout);

    ASSERT_TRUE(camera.inject_frame(42));
    const Result<void> read = camera.read(lease, kTimeout);
    ASSERT_TRUE(read.ok()) << read.error().message;
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->sequence, 42U);
    EXPECT_EQ(lease->frame_id, "fake_camera");

    const Result<void> second = camera.read(lease, kTimeout);
    EXPECT_FALSE(second.ok());
}

TEST(FakeCameraTest, RecoveryAndSetters) {
    test::FakeCamera camera;
    ASSERT_TRUE(camera.open().ok());
    ASSERT_TRUE(camera.start().ok());

    ASSERT_TRUE(camera.set_exposure(2000.0).ok());
    ASSERT_TRUE(camera.set_gain(10.0).ok());
    ASSERT_TRUE(camera.set_frame_rate(100.0).ok());
    ASSERT_TRUE(camera.exposure_us().has_value());
    EXPECT_DOUBLE_EQ(*camera.exposure_us(), 2000.0);

    camera.mark_error();
    EXPECT_EQ(camera.state(), DeviceState::kError);
    ASSERT_TRUE(camera.recover().ok());
    EXPECT_EQ(camera.state(), DeviceState::kOpen);
    ASSERT_TRUE(camera.start().ok());
    EXPECT_EQ(camera.state(), DeviceState::kStreaming);
}

TEST(FakeLidarTest, PointCloudAndImu) {
    test::FakeLidar lidar;
    ASSERT_TRUE(lidar.open().ok());
    ASSERT_TRUE(lidar.start().ok());

    FrameLease<PointCloudFrame> cloud;
    EXPECT_EQ(lidar.read(cloud, kTimeout).error().code, ErrorCode::kTimeout);

    ASSERT_TRUE(lidar.inject_cloud(7, 4));
    ASSERT_TRUE(lidar.read(cloud, kTimeout).ok());
    ASSERT_TRUE(cloud);
    EXPECT_EQ(cloud->sequence, 7U);
    EXPECT_EQ(cloud->count, 4U);
    EXPECT_EQ(cloud->frame_id, "fake_lidar");

    ImuSample imu;
    EXPECT_EQ(lidar.read_imu(imu, kTimeout).error().code, ErrorCode::kNoData);

    ImuSample sample;
    sample.gyro_y = 1.5F;
    lidar.inject_imu(sample);
    ASSERT_TRUE(lidar.read_imu(imu, kTimeout).ok());
    EXPECT_FLOAT_EQ(imu.gyro_y, 1.5F);
    EXPECT_EQ(lidar.read_imu(imu, kTimeout).error().code, ErrorCode::kNoData);
}

TEST(FakeLidarTest, LifecycleAndIdempotentClose) {
    test::FakeLidar lidar;
    EXPECT_EQ(lidar.state(), DeviceState::kClosed);
    ASSERT_TRUE(lidar.open().ok());
    ASSERT_TRUE(lidar.start().ok());
    ASSERT_TRUE(lidar.stop().ok());
    ASSERT_TRUE(lidar.close().ok());
    ASSERT_TRUE(lidar.close().ok());
    EXPECT_EQ(lidar.state(), DeviceState::kClosed);
}

} // namespace
} // namespace foray::platform::hal
