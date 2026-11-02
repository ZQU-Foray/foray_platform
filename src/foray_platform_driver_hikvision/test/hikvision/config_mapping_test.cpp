#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>

#include "hikvision/mvs_nodes.hpp"
#include "mock_mvs_api.hpp"

namespace foray::platform::driver::hikvision {
namespace {

namespace node = detail::node;

/// @brief 每个用例独立一份 mock，避免写入记录互相污染。
struct MappingFixture {
    test::MockMvsApi api;
    std::uint32_t optional_errors = 0;

    Result<void> apply(const hal::CameraConfig& config) {
        return detail::apply_config(api, nullptr, config, optional_errors);
    }

    Result<void> tune(const hal::CameraTuning& tuning) {
        return detail::apply_tuning(api, nullptr, tuning);
    }

    /// @brief 某节点最后一次写入是否等于给定字符串。
    bool wrote(std::string_view node_name, const std::string& value) {
        return api.last_write(node_name) == std::optional<std::string>(value);
    }

    bool touched(std::string_view node_name) {
        return api.last_write(node_name).has_value();
    }
};

/**
 * @brief 几何：非 0 的分辨率下发；其余字段 0 表示「保留设备值」必须跳过。
 */
TEST(HikvisionConfigMapping, GeometryWritesResolutionAndSkipsZeroOptionals) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;

    const auto status = fixture.apply(config);

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_TRUE(fixture.wrote(node::kWidth, "720"));
    EXPECT_TRUE(fixture.wrote(node::kHeight, "540"));
    EXPECT_TRUE(fixture.wrote(node::kPixelFormat, "BayerRG8"));
    EXPECT_FALSE(fixture.touched(node::kOffsetX));
    EXPECT_FALSE(fixture.touched(node::kOffsetY));
    EXPECT_FALSE(fixture.touched(node::kBinningHorizontal));
    EXPECT_FALSE(fixture.touched(node::kDecimationVertical));
}

/**
 * @brief `width`/`height` 为 0 时不得写 `Width`/`Height`。
 *
 * `CameraConfig` 用 0 表示「保留设备当前值」，而裸写 0 会让 MVS 返回
 * `MV_E_GC_RANGE`（实测 0x80000102），导致不带分辨率的配置永远打不开设备。
 */
TEST(HikvisionConfigMapping, GeometrySkipsZeroResolution) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 0;
    config.height = 0;

    const auto status = fixture.apply(config);

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_FALSE(fixture.touched(node::kWidth));
    EXPECT_FALSE(fixture.touched(node::kHeight));
}

/// @brief 非 0 的 ROI/抽降采样项必须落到对应节点。
TEST(HikvisionConfigMapping, GeometryWritesRoiAndSampling) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 640;
    config.height = 480;
    config.offset_x = 8;
    config.offset_y = 16;
    config.binning_horizontal = 2;
    config.binning_vertical = 4;
    config.decimation_horizontal = 2;
    config.decimation_vertical = 3;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kOffsetX, "8"));
    EXPECT_TRUE(fixture.wrote(node::kOffsetY, "16"));
    EXPECT_TRUE(fixture.wrote(node::kBinningHorizontal, "2"));
    EXPECT_TRUE(fixture.wrote(node::kBinningVertical, "4"));
    EXPECT_TRUE(fixture.wrote(node::kDecimationHorizontal, "2"));
    EXPECT_TRUE(fixture.wrote(node::kDecimationVertical, "3"));
}

/// @brief 所有 8 位格式都要映射到 MVS 枚举名。
TEST(HikvisionConfigMapping, MapsEverySupportedPixelFormat) {
    struct Case {
        hal::PixelFormat format;
        const char* name;
    };
    const Case cases[] = {
        {hal::PixelFormat::kMono8, "Mono8"},
        {hal::PixelFormat::kBayerRG8, "BayerRG8"},
        {hal::PixelFormat::kBayerGR8, "BayerGR8"},
        {hal::PixelFormat::kBayerGB8, "BayerGB8"},
        {hal::PixelFormat::kBayerBG8, "BayerBG8"},
    };

    for (const auto& item : cases) {
        MappingFixture fixture;
        hal::CameraConfig config;
        config.width = 1;
        config.height = 1;
        config.pixel_format = item.format;

        ASSERT_TRUE(fixture.apply(config).ok()) << item.name;
        EXPECT_TRUE(fixture.wrote(node::kPixelFormat, item.name)) << item.name;
    }
}

/// @brief 10/12 位与彩色格式在 core 层就拒绝，不把未知语义传给设备。
TEST(HikvisionConfigMapping, RejectsUnsupportedPixelFormat) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.pixel_format = hal::PixelFormat::kMono12;

    const auto status = fixture.apply(config);

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kUnsupportedFormat);
    EXPECT_FALSE(fixture.touched(node::kPixelFormat));
}

/// @brief 帧率：使能位总是写，频率只在配置了才写。
TEST(HikvisionConfigMapping, FrameRateWritesEnableFlagThenValue) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.frame_rate_enable = true;
    config.frame_rate_hz = 100.0;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kFrameRateEnable, "true"));
    const auto rate = fixture.api.last_write(node::kFrameRate);
    ASSERT_TRUE(rate.has_value());
    EXPECT_NE(rate->find("100"), std::string::npos);
}

/// @brief 不配置帧率时不得写 `AcquisitionFrameRate`（否则会覆盖设备默认值）。
TEST(HikvisionConfigMapping, FrameRateValueOmittedWhenUnset) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.frame_rate_enable = false;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kFrameRateEnable, "false"));
    EXPECT_FALSE(fixture.touched(node::kFrameRate));
}

/// @brief 自动曝光模式下仍下发手动值，便于后续切回手动时沿用。
TEST(HikvisionConfigMapping, ExposureAutoWritesLimits) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.exposure_auto = hal::AutoMode::kContinuous;
    config.exposure_us = 5000.0;
    config.exposure_auto_lower_us = 100.0;
    config.exposure_auto_upper_us = 9000.0;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kExposureAuto, "Continuous"));
    EXPECT_TRUE(fixture.touched(node::kExposureTime));
    EXPECT_TRUE(fixture.touched(node::kExposureLowerLimit));
    EXPECT_TRUE(fixture.touched(node::kExposureUpperLimit));
}

/// @brief 手动曝光模式只写 `ExposureAuto=Off` 与曝光时间，不碰上下限。
TEST(HikvisionConfigMapping, ManualExposureSkipsAutoLimits) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.exposure_auto = hal::AutoMode::kOff;
    config.exposure_us = 5000.0;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kExposureAuto, "Off"));
    EXPECT_TRUE(fixture.touched(node::kExposureTime));
    EXPECT_FALSE(fixture.touched(node::kExposureLowerLimit));
}

/// @brief 增益与曝光同构：`GainAuto` 加手动值加自动上下限。
TEST(HikvisionConfigMapping, GainWritesAutoAndLimits) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.gain_auto = hal::AutoMode::kOnce;
    config.gain_db = 12.5;
    config.gain_auto_lower_db = 0.0;
    config.gain_auto_upper_db = 24.0;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kGainAuto, "Once"));
    EXPECT_TRUE(fixture.touched(node::kGain));
    EXPECT_TRUE(fixture.touched(node::kGainLowerLimit));
    EXPECT_TRUE(fixture.touched(node::kGainUpperLimit));
}

/// @brief 触发关闭时只写 `TriggerMode=Off`，不得残留 Software/Line 配置。
TEST(HikvisionConfigMapping, TriggerOffWritesOnlyMode) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.trigger.mode = hal::TriggerMode::kOff;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kTriggerMode, "Off"));
    EXPECT_FALSE(fixture.touched(node::kTriggerSource));
    EXPECT_FALSE(fixture.touched(node::kTriggerActivation));
}

/// @brief 软触发没有物理线路，不该写激活沿与去抖。
TEST(HikvisionConfigMapping, SoftwareTriggerSkipsLineNodes) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.trigger.mode = hal::TriggerMode::kSoftware;
    config.trigger.activation = hal::TriggerActivation::kFallingEdge;
    config.trigger.debounce_us = 50.0;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kTriggerMode, "On"));
    EXPECT_TRUE(fixture.wrote(node::kTriggerSource, "Software"));
    EXPECT_FALSE(fixture.touched(node::kTriggerActivation));
    EXPECT_FALSE(fixture.touched(node::kLineDebouncerTime));
}

/// @brief 线路触发要写全：模式、源、沿、去抖。
TEST(HikvisionConfigMapping, LineTriggerWritesActivationAndDebounce) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.trigger.mode = hal::TriggerMode::kLine1;
    config.trigger.activation = hal::TriggerActivation::kFallingEdge;
    config.trigger.debounce_us = 50.0;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kTriggerMode, "On"));
    EXPECT_TRUE(fixture.wrote(node::kTriggerSource, "Line1"));
    EXPECT_TRUE(fixture.wrote(node::kTriggerActivation, "FallingEdge"));
    const auto debounce = fixture.api.last_write(node::kLineDebouncerTime);
    ASSERT_TRUE(debounce.has_value());
    EXPECT_NE(debounce->find("50"), std::string::npos);
}

/// @brief `trigger.source` 显式覆盖 `trigger.mode` 推导出的线路。
TEST(HikvisionConfigMapping, ExplicitTriggerSourceWins) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.trigger.mode = hal::TriggerMode::kLine0;
    config.trigger.source = hal::TriggerMode::kLine3;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_TRUE(fixture.wrote(node::kTriggerSource, "Line3"));
}

/// @brief GigE 调优项全部下发。
TEST(HikvisionConfigMapping, GigEOptionalNodesApplied) {
    MappingFixture fixture;
    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.packet_size = 9000;
    config.inter_packet_delay_us = 0;
    config.heartbeat_timeout_ms = 3000;
    config.enable_ptp = true;

    ASSERT_TRUE(fixture.apply(config).ok());
    EXPECT_EQ(fixture.optional_errors, 0U);
    EXPECT_TRUE(fixture.wrote(node::kPacketSize, "9000"));
    EXPECT_TRUE(fixture.wrote(node::kPacketDelay, "0"));
    EXPECT_TRUE(fixture.wrote(node::kHeartbeatTimeout, "3000"));
    EXPECT_TRUE(fixture.wrote(node::kPtpEnable, "true"));
}

/**
 * @brief 可选节点写失败只计数，不阻断 open。
 *
 * 现场网卡与相机型号差异大（例如 USB3 相机没有 `GevIEEE1588`），把这些当成致命
 * 错误会让相机整体不可用。
 */
TEST(HikvisionConfigMapping, OptionalNodeFailuresAreCountedNotFatal) {
    MappingFixture fixture;
    fixture.api.failing_nodes.emplace(std::string(node::kPacketSize), detail::mvs_error::kAccessDenied);
    fixture.api.failing_nodes.emplace(std::string(node::kPtpEnable), detail::mvs_error::kSupport);

    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;
    config.packet_size = 9000;
    config.enable_ptp = true;

    const auto status = fixture.apply(config);

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_EQ(fixture.optional_errors, 2U);
}

/// @brief 语义相关的节点写失败必须让 open 整体失败。
TEST(HikvisionConfigMapping, RequiredNodeFailureAborts) {
    MappingFixture fixture;
    fixture.api.failing_nodes.emplace(std::string(node::kWidth), detail::mvs_error::kAccessDenied);

    hal::CameraConfig config;
    config.width = 720;
    config.height = 540;

    const auto status = fixture.apply(config);

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kPermissionDenied);
}

/// @brief 空 tuning 不得产生任何设备写入。
TEST(HikvisionConfigMapping, EmptyTuningWritesNothing) {
    MappingFixture fixture;

    ASSERT_TRUE(fixture.tune(hal::CameraTuning{}).ok());
    EXPECT_TRUE(fixture.api.snapshot_writes().empty());
}

/// @brief 内层空 = 清除限速：关掉帧率使能而不是写 0。
TEST(HikvisionConfigMapping, TuningClearsFrameRateByDisabling) {
    MappingFixture fixture;
    hal::CameraTuning tuning;
    tuning.frame_rate_hz = std::optional<double>{};

    ASSERT_TRUE(fixture.tune(tuning).ok());
    EXPECT_TRUE(fixture.wrote(node::kFrameRateEnable, "false"));
    EXPECT_FALSE(fixture.touched(node::kFrameRate));
}

/// @brief 内层空 = 交回自动曝光。
TEST(HikvisionConfigMapping, TuningRestoresAutoExposure) {
    MappingFixture fixture;
    hal::CameraTuning tuning;
    tuning.exposure_us = std::optional<double>{};

    ASSERT_TRUE(fixture.tune(tuning).ok());
    EXPECT_TRUE(fixture.wrote(node::kExposureAuto, "Continuous"));
}

/// @brief 内层空 = 交回自动增益。
TEST(HikvisionConfigMapping, TuningRestoresAutoGain) {
    MappingFixture fixture;
    hal::CameraTuning tuning;
    tuning.gain_db = std::optional<double>{};

    ASSERT_TRUE(fixture.tune(tuning).ok());
    EXPECT_TRUE(fixture.wrote(node::kGainAuto, "Continuous"));
}

/// @brief 运行期改曝光：只写 `ExposureTime`，不动自动模式。
TEST(HikvisionConfigMapping, TuningExposureWritesValueOnly) {
    MappingFixture fixture;
    hal::CameraTuning tuning;
    tuning.exposure_us = std::optional<double>(2500.0);

    ASSERT_TRUE(fixture.tune(tuning).ok());
    const auto exposure = fixture.api.last_write(node::kExposureTime);
    ASSERT_TRUE(exposure.has_value());
    EXPECT_NE(exposure->find("2500"), std::string::npos);
    EXPECT_FALSE(fixture.touched(node::kExposureAuto));
}

/// @brief 运行期去抖为 0 时明确写 0，而不是跳过。
TEST(HikvisionConfigMapping, TuningDebounceZeroIsWritten) {
    MappingFixture fixture;
    hal::CameraTuning tuning;
    tuning.trigger_debounce_us = std::optional<double>{};

    ASSERT_TRUE(fixture.tune(tuning).ok());
    const auto debounce = fixture.api.last_write(node::kLineDebouncerTime);
    ASSERT_TRUE(debounce.has_value());
    EXPECT_NE(debounce->find("0"), std::string::npos);
}

/// @brief 时间戳字段不写设备，只由驱动侧换算使用。
TEST(HikvisionConfigMapping, TuningTimestampFieldsDoNotTouchDevice) {
    MappingFixture fixture;
    hal::CameraTuning tuning;
    tuning.timestamp_source = hal::TimestampSource::kDevice;
    tuning.device_to_host_offset_ns = std::optional<std::int64_t>(1000);

    ASSERT_TRUE(fixture.tune(tuning).ok());
    EXPECT_TRUE(fixture.api.snapshot_writes().empty());
}

/// @brief PTP 状态只在字符串恰为 `Slave` 时为真。
TEST(HikvisionConfigMapping, QueryPtpSlaveRequiresExactStatus) {
    test::MockMvsApi api;
    api.enum_values.emplace(std::string(node::kPtpStatus), "Slave");
    EXPECT_TRUE(detail::query_ptp_slave(api, nullptr));

    api.enum_values[std::string(node::kPtpStatus)] = "Master";
    EXPECT_FALSE(detail::query_ptp_slave(api, nullptr));
}

/// @brief PTP 节点不存在时不报错，只是视为非 Slave。
TEST(HikvisionConfigMapping, QueryPtpSlaveToleratesMissingNode) {
    test::MockMvsApi api;
    api.failing_nodes.emplace(std::string(node::kPtpStatus), detail::mvs_error::kSupport);
    EXPECT_FALSE(detail::query_ptp_slave(api, nullptr));
}

/// @brief tick 频率读不到或非正数都归零，时间换算据此降级。
TEST(HikvisionConfigMapping, QueryTickFrequencyRejectsNonPositive) {
    test::MockMvsApi api;
    api.int_values[std::string(node::kDeviceTicksFrequency)] = 0;
    EXPECT_EQ(detail::query_tick_frequency(api, nullptr), 0U);

    api.int_values[std::string(node::kDeviceTicksFrequency)] = -1;
    EXPECT_EQ(detail::query_tick_frequency(api, nullptr), 0U);

    api.int_values[std::string(node::kDeviceTicksFrequency)] = 80'000'000;
    EXPECT_EQ(detail::query_tick_frequency(api, nullptr), 80'000'000U);
}

} // namespace
} // namespace foray::platform::driver::hikvision
