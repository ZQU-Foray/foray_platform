#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "foray/platform/hal/config_binder.hpp"
#include "foray/platform/param/store.hpp"

namespace foray::platform::hal {
namespace {

param::ParamSnapshot make_snapshot(const param::ParamSchema& schema,
                                   const param::ParamValueMap& values) {
    param::ParamStore store(schema);
    const param::ParamResult result = store.apply(values, param::ParamUpdateOrigin::kInitialLoad);
    if (!result.ok()) {
        return param::ParamSnapshot(param::ParamValueMap{}, 0);
    }
    return *store.snapshot();
}

TEST(ConfigBinderTest, CameraBindsDefaults) {
    const param::ParamSchema schema = make_camera_param_schema();
    const param::ParamSnapshot snapshot =
        make_snapshot(schema, param::ParamValueMap{{"camera.serial", param::ParamValue("DA123")}});

    const Result<CameraConfig> result = bind_camera_config(snapshot);

    ASSERT_TRUE(result.ok()) << result.error().message;
    const CameraConfig& config = result.value();
    EXPECT_EQ(config.selector.serial, "DA123");
    EXPECT_EQ(config.selector.transport, Transport::kAny);
    EXPECT_EQ(config.pixel_format, PixelFormat::kBayerRG8);
    EXPECT_EQ(config.trigger.mode, TriggerMode::kOff);
    EXPECT_EQ(config.buffer_count, 4U);
    EXPECT_EQ(config.sdk_node_count, 5U);
    EXPECT_EQ(config.frame_id, "camera");
}

TEST(ConfigBinderTest, CameraUsbRequiresSerial) {
    const param::ParamSchema schema = make_camera_param_schema();

    const param::ParamSnapshot missing = make_snapshot(
        schema, param::ParamValueMap{{"camera.transport", param::ParamValue("usb3")}});
    const Result<CameraConfig> missing_result = bind_camera_config(missing);
    EXPECT_FALSE(missing_result.ok());

    const param::ParamSnapshot present =
        make_snapshot(schema, param::ParamValueMap{{"camera.transport", param::ParamValue("usb3")},
                                                   {"camera.serial", param::ParamValue("DA123")}});
    EXPECT_TRUE(bind_camera_config(present).ok());
}

TEST(ConfigBinderTest, CameraGigEAcceptsIp) {
    const param::ParamSchema schema = make_camera_param_schema();
    const param::ParamSnapshot snapshot = make_snapshot(
        schema, param::ParamValueMap{{"camera.transport", param::ParamValue("gige")},
                                     {"camera.ip_address", param::ParamValue("192.168.1.10")}});

    const Result<CameraConfig> result = bind_camera_config(snapshot);

    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().selector.transport, Transport::kGigE);
    EXPECT_EQ(result.value().selector.ip_address, "192.168.1.10");
}

TEST(ConfigBinderTest, CameraRejectsNoSelector) {
    const param::ParamSchema schema = make_camera_param_schema();
    const param::ParamSnapshot snapshot = make_snapshot(schema, param::ParamValueMap{});

    EXPECT_FALSE(bind_camera_config(snapshot).ok());
}

TEST(ConfigBinderTest, CameraRejectsFrameRateWhenTriggered) {
    const param::ParamSchema schema = make_camera_param_schema();
    const param::ParamSnapshot snapshot = make_snapshot(
        schema, param::ParamValueMap{{"camera.serial", param::ParamValue("DA123")},
                                     {"camera.trigger_mode", param::ParamValue("line0")},
                                     {"camera.frame_rate_hz", param::ParamValue(100.0)}});

    EXPECT_FALSE(bind_camera_config(snapshot).ok());
}

TEST(ConfigBinderTest, CameraAcceptsHardwareTriggerWithoutFrameRate) {
    const param::ParamSchema schema = make_camera_param_schema();
    const param::ParamSnapshot snapshot = make_snapshot(
        schema,
        param::ParamValueMap{{"camera.serial", param::ParamValue("DA123")},
                             {"camera.trigger_mode", param::ParamValue("line0")},
                             {"camera.trigger_activation", param::ParamValue("rising_edge")}});

    const Result<CameraConfig> result = bind_camera_config(snapshot);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().trigger.mode, TriggerMode::kLine0);
    EXPECT_FALSE(result.value().frame_rate_hz.has_value());
}

TEST(ConfigBinderTest, LidarBindsDefaults) {
    const param::ParamSchema schema = make_lidar_param_schema();
    const param::ParamSnapshot snapshot = make_snapshot(
        schema, param::ParamValueMap{{"lidar.ip_address", param::ParamValue("192.168.1.100")},
                                     {"lidar.host_ip", param::ParamValue("192.168.1.5")}});

    const Result<LidarConfig> result = bind_lidar_config(snapshot);

    ASSERT_TRUE(result.ok()) << result.error().message;
    const LidarConfig& config = result.value();
    EXPECT_EQ(config.selector.ip_address, "192.168.1.100");
    EXPECT_EQ(config.host_ip, "192.168.1.5");
    EXPECT_EQ(config.data_type, LidarConfig::PointDataType::kHigh);
    EXPECT_EQ(config.scan_pattern, LidarConfig::ScanPattern::kNonRepetitive);
    EXPECT_TRUE(config.imu_enabled);
    EXPECT_EQ(config.packet_ring_slots, 4096U);
    EXPECT_EQ(config.point_buffer_capacity, 200000U);
    EXPECT_EQ(config.frame_id, "lidar");
}

TEST(ConfigBinderTest, LidarRequiresIpAndHost) {
    const param::ParamSchema schema = make_lidar_param_schema();

    const param::ParamSnapshot no_ip = make_snapshot(
        schema, param::ParamValueMap{{"lidar.host_ip", param::ParamValue("192.168.1.5")}});
    EXPECT_FALSE(bind_lidar_config(no_ip).ok());

    const param::ParamSnapshot no_host = make_snapshot(
        schema, param::ParamValueMap{{"lidar.ip_address", param::ParamValue("192.168.1.100")}});
    EXPECT_FALSE(bind_lidar_config(no_host).ok());
}

TEST(ConfigBinderTest, LidarLineNumZeroMeansAuto) {
    const param::ParamSchema schema = make_lidar_param_schema();

    const param::ParamSnapshot automatic = make_snapshot(
        schema, param::ParamValueMap{{"lidar.ip_address", param::ParamValue("192.168.1.100")},
                                     {"lidar.host_ip", param::ParamValue("192.168.1.5")}});
    const Result<LidarConfig> automatic_result = bind_lidar_config(automatic);
    ASSERT_TRUE(automatic_result.ok()) << automatic_result.error().message;
    EXPECT_FALSE(automatic_result.value().line_num.has_value());

    const param::ParamSnapshot explicit_line = make_snapshot(
        schema, param::ParamValueMap{{"lidar.ip_address", param::ParamValue("192.168.1.100")},
                                     {"lidar.host_ip", param::ParamValue("192.168.1.5")},
                                     {"lidar.line_num", param::ParamValue(4)}});
    const Result<LidarConfig> explicit_result = bind_lidar_config(explicit_line);
    ASSERT_TRUE(explicit_result.ok()) << explicit_result.error().message;
    ASSERT_TRUE(explicit_result.value().line_num.has_value());
    EXPECT_EQ(*explicit_result.value().line_num, 4U);
}

TEST(ConfigBinderTest, CameraBindsExtendedDefaults) {
    const param::ParamSchema schema = make_camera_param_schema();
    const param::ParamSnapshot snapshot =
        make_snapshot(schema, param::ParamValueMap{{"camera.serial", param::ParamValue("DA123")}});

    const Result<CameraConfig> result = bind_camera_config(snapshot);

    ASSERT_TRUE(result.ok()) << result.error().message;
    const CameraConfig& config = result.value();
    EXPECT_EQ(config.exposure_auto, AutoMode::kOff);
    EXPECT_EQ(config.gain_auto, AutoMode::kOff);
    EXPECT_EQ(config.timestamp_source, TimestampSource::kAuto);
    EXPECT_EQ(config.packet_size, 0U);
    EXPECT_EQ(config.binning_horizontal, 0U);
    EXPECT_EQ(config.decimation_vertical, 0U);
    EXPECT_EQ(config.pool_memory_limit_bytes, 0U);
    EXPECT_TRUE(config.frame_rate_enable);
    EXPECT_FALSE(config.trigger.source.has_value());
    EXPECT_FALSE(config.trigger.debounce_us.has_value());
    EXPECT_FALSE(config.exposure_auto_lower_us.has_value());
    EXPECT_FALSE(config.gain_auto_upper_db.has_value());
    EXPECT_FALSE(config.net_export.has_value());
    EXPECT_FALSE(config.device_to_host_offset_ns.has_value());
}

TEST(ConfigBinderTest, CameraBindsExtendedValues) {
    const param::ParamSchema schema = make_camera_param_schema();
    const param::ParamSnapshot snapshot = make_snapshot(
        schema, param::ParamValueMap{
                    {"camera.serial", param::ParamValue("DA123")},
                    {"camera.binning_horizontal", param::ParamValue(2)},
                    {"camera.decimation_vertical", param::ParamValue(3)},
                    {"camera.exposure_auto", param::ParamValue("continuous")},
                    {"camera.exposure_auto_lower_us", param::ParamValue(100.0)},
                    {"camera.exposure_auto_upper_us", param::ParamValue(8000.0)},
                    {"camera.gain_auto", param::ParamValue("once")},
                    {"camera.gain_auto_lower_db", param::ParamValue(0.0)},
                    {"camera.gain_auto_upper_db", param::ParamValue(12.0)},
                    {"camera.trigger_mode", param::ParamValue("software")},
                    {"camera.trigger_source", param::ParamValue("software")},
                    {"camera.trigger_debounce_us", param::ParamValue(50.0)},
                    {"camera.packet_size", param::ParamValue(9000)},
                    {"camera.inter_packet_delay_us", param::ParamValue(20)},
                    {"camera.heartbeat_timeout_ms", param::ParamValue(5000)},
                    {"camera.net_export", param::ParamValue(2)},
                    {"camera.pool_memory_limit_bytes", param::ParamValue(268435456)},
                    {"camera.timestamp_source", param::ParamValue("device")},
                    {"camera.device_to_host_offset_ns", param::ParamValue(-1500)},
                    {"camera.exposure_to_host_latency_ns", param::ParamValue(2500000.0)},
                });

    const Result<CameraConfig> result = bind_camera_config(snapshot);

    ASSERT_TRUE(result.ok()) << result.error().message;
    const CameraConfig& config = result.value();
    EXPECT_EQ(config.binning_horizontal, 2U);
    EXPECT_EQ(config.decimation_vertical, 3U);
    EXPECT_EQ(config.exposure_auto, AutoMode::kContinuous);
    ASSERT_TRUE(config.exposure_auto_lower_us.has_value());
    EXPECT_DOUBLE_EQ(*config.exposure_auto_lower_us, 100.0);
    ASSERT_TRUE(config.exposure_auto_upper_us.has_value());
    EXPECT_DOUBLE_EQ(*config.exposure_auto_upper_us, 8000.0);
    EXPECT_EQ(config.gain_auto, AutoMode::kOnce);
    ASSERT_TRUE(config.gain_auto_upper_db.has_value());
    EXPECT_DOUBLE_EQ(*config.gain_auto_upper_db, 12.0);
    EXPECT_EQ(config.trigger.mode, TriggerMode::kSoftware);
    ASSERT_TRUE(config.trigger.source.has_value());
    EXPECT_EQ(*config.trigger.source, TriggerMode::kSoftware);
    ASSERT_TRUE(config.trigger.debounce_us.has_value());
    EXPECT_DOUBLE_EQ(*config.trigger.debounce_us, 50.0);
    EXPECT_EQ(config.packet_size, 9000U);
    ASSERT_TRUE(config.inter_packet_delay_us.has_value());
    EXPECT_EQ(*config.inter_packet_delay_us, 20U);
    ASSERT_TRUE(config.heartbeat_timeout_ms.has_value());
    EXPECT_EQ(*config.heartbeat_timeout_ms, 5000U);
    ASSERT_TRUE(config.net_export.has_value());
    EXPECT_EQ(*config.net_export, 2);
    EXPECT_EQ(config.pool_memory_limit_bytes, 268435456U);
    EXPECT_EQ(config.timestamp_source, TimestampSource::kDevice);
    ASSERT_TRUE(config.device_to_host_offset_ns.has_value());
    EXPECT_EQ(*config.device_to_host_offset_ns, -1500);
    ASSERT_TRUE(config.exposure_to_host_latency_ns.has_value());
    EXPECT_DOUBLE_EQ(*config.exposure_to_host_latency_ns, 2500000.0);
}

TEST(ConfigBinderTest, CameraRejectsTriggerSourceWithoutMode) {
    const param::ParamSchema schema = make_camera_param_schema();
    const param::ParamSnapshot snapshot = make_snapshot(
        schema, param::ParamValueMap{{"camera.serial", param::ParamValue("DA123")},
                                     {"camera.trigger_source", param::ParamValue("line0")}});

    const Result<CameraConfig> result = bind_camera_config(snapshot);

    EXPECT_FALSE(result.ok());
}

TEST(ConfigBinderTest, CameraRejectsTriggerSourceMismatch) {
    const param::ParamSchema schema = make_camera_param_schema();
    const param::ParamSnapshot snapshot = make_snapshot(
        schema, param::ParamValueMap{{"camera.serial", param::ParamValue("DA123")},
                                     {"camera.trigger_mode", param::ParamValue("software")},
                                     {"camera.trigger_source", param::ParamValue("line1")}});

    const Result<CameraConfig> result = bind_camera_config(snapshot);

    EXPECT_FALSE(result.ok());
}

TEST(ConfigBinderTest, CameraRejectsInvertedAutoRanges) {
    const param::ParamSchema schema = make_camera_param_schema();

    const param::ParamSnapshot exposure = make_snapshot(
        schema, param::ParamValueMap{{"camera.serial", param::ParamValue("DA123")},
                                     {"camera.exposure_auto_lower_us", param::ParamValue(9000.0)},
                                     {"camera.exposure_auto_upper_us", param::ParamValue(100.0)}});
    EXPECT_FALSE(bind_camera_config(exposure).ok());

    const param::ParamSnapshot gain = make_snapshot(
        schema, param::ParamValueMap{{"camera.serial", param::ParamValue("DA123")},
                                     {"camera.gain_auto_lower_db", param::ParamValue(20.0)},
                                     {"camera.gain_auto_upper_db", param::ParamValue(4.0)}});
    EXPECT_FALSE(bind_camera_config(gain).ok());
}

} // namespace
} // namespace foray::platform::hal
