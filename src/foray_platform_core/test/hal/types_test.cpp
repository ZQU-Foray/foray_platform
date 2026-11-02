#include <gtest/gtest.h>

#include <string>

#include "foray/platform/hal/config.hpp"
#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/sensor_info.hpp"
#include "foray/platform/hal/sensor_stats.hpp"
#include "foray/platform/hal/types.hpp"

namespace foray::platform::hal {
namespace {

TEST(HalTypesTest, EnumNames) {
    EXPECT_STREQ(to_string(DeviceState::kStreaming), "streaming");
    EXPECT_STREQ(to_string(Transport::kGigE), "gige");
    EXPECT_STREQ(to_string(PixelFormat::kBayerRG8), "bayer_rgg8");
    EXPECT_STREQ(to_string(TimestampSync::kHostReceive), "host_receive");
    EXPECT_STREQ(to_string(TriggerMode::kLine0), "line0");
    EXPECT_STREQ(to_string(TriggerActivation::kRisingEdge), "rising_edge");
}

TEST(HalTypesTest, PixelFormatBytes) {
    EXPECT_EQ(bytes_per_pixel(PixelFormat::kMono8), 1U);
    EXPECT_EQ(bytes_per_pixel(PixelFormat::kBayerRG8), 1U);
    EXPECT_EQ(bytes_per_pixel(PixelFormat::kMono12), 2U);
    EXPECT_EQ(bytes_per_pixel(PixelFormat::kRgb8), 3U);
    EXPECT_EQ(bytes_per_pixel(PixelFormat::kYuv422), 2U);
}

TEST(HalTypesTest, FirstPhaseFormats) {
    EXPECT_TRUE(is_supported_pixel_format(PixelFormat::kMono8));
    EXPECT_TRUE(is_supported_pixel_format(PixelFormat::kBayerRG8));
    EXPECT_TRUE(is_supported_pixel_format(PixelFormat::kBayerGR8));
    EXPECT_TRUE(is_supported_pixel_format(PixelFormat::kBayerGB8));
    EXPECT_TRUE(is_supported_pixel_format(PixelFormat::kBayerBG8));
    EXPECT_FALSE(is_supported_pixel_format(PixelFormat::kRgb8));
    EXPECT_FALSE(is_supported_pixel_format(PixelFormat::kMono12));
}

TEST(HalConfigTest, CameraDefaults) {
    const CameraConfig config;

    EXPECT_EQ(config.pixel_format, PixelFormat::kBayerRG8);
    EXPECT_EQ(config.trigger.mode, TriggerMode::kOff);
    EXPECT_EQ(config.trigger.activation, TriggerActivation::kRisingEdge);
    EXPECT_EQ(config.buffer_count, 4U);
    EXPECT_EQ(config.sdk_node_count, 5U);
    EXPECT_FALSE(config.enable_ptp);
    EXPECT_TRUE(config.allow_unsynced_stamp);
    EXPECT_EQ(config.frame_id, "camera");
}

TEST(HalConfigTest, LidarDefaults) {
    const LidarConfig config;

    EXPECT_EQ(config.data_type, LidarConfig::PointDataType::kHigh);
    EXPECT_EQ(config.scan_pattern, LidarConfig::ScanPattern::kNonRepetitive);
    EXPECT_TRUE(config.imu_enabled);
    EXPECT_EQ(config.packet_ring_slots, 4096U);
    EXPECT_EQ(config.max_packet_bytes, 1500U);
    EXPECT_EQ(config.point_buffer_capacity, 200000U);
    EXPECT_EQ(config.frame_buffer_count, 3U);
    EXPECT_EQ(config.frame_timeout_ms, 200U);
    EXPECT_EQ(config.frame_id, "lidar");
}

TEST(HalFrameTest, DefaultsAreZeroed) {
    const ImageFrame image;
    EXPECT_EQ(image.data, nullptr);
    EXPECT_EQ(image.size, 0U);
    EXPECT_EQ(image.width, 0U);
    EXPECT_EQ(image.sync, TimestampSync::kUnknown);
    EXPECT_TRUE(image.stamp.is_zero());

    const PointCloudFrame cloud;
    EXPECT_EQ(cloud.points, nullptr);
    EXPECT_EQ(cloud.count, 0U);
    EXPECT_TRUE(cloud.base_time.is_zero());

    const ImuSample imu;
    EXPECT_FLOAT_EQ(imu.gyro_x, 0.0F);
    EXPECT_FLOAT_EQ(imu.acc_z, 0.0F);
    EXPECT_TRUE(imu.stamp.is_zero());
}

TEST(HalSensorTest, InfoAndStatsDefaults) {
    const SensorInfo info;
    EXPECT_EQ(info.transport, Transport::kAny);
    EXPECT_TRUE(info.serial.empty());

    const SensorStats stats;
    EXPECT_EQ(stats.frames_received, 0U);
    EXPECT_EQ(stats.frames_dropped, 0U);
    EXPECT_EQ(stats.reconnects, 0U);
}

} // namespace
} // namespace foray::platform::hal
