#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "foray/platform/hal/frame_lease.hpp"
#include "foray/platform/result/error.hpp"
#include "foray/platform/time/duration.hpp"
#include "hikvision/camera_impl.hpp"
#include "hikvision/mvs_error.hpp"
#include "hikvision/mvs_nodes.hpp"
#include "mock_mvs_api.hpp"

namespace foray::platform::driver::hikvision {
namespace {

namespace node = detail::node;

/// @brief 一帧 720×540 Mono8 的测试载荷。
detail::FrameInfoView mono8_frame_info(std::uint32_t frame_number = 1) {
    detail::FrameInfoView info;
    info.width = 720;
    info.height = 540;
    info.frame_length = 720U * 540U;
    info.pixel_format = hal::PixelFormat::kMono8;
    info.frame_number = frame_number;
    info.trigger_index = static_cast<std::uint32_t>(frame_number);
    info.lost_packets = 0;
    info.exposure_us = 5000.0F;
    info.gain_db = 0.0F;
    info.device_ticks = 0;
    return info;
}

std::unique_ptr<detail::FrameView> make_frame(const detail::FrameInfoView& info) {
    std::vector<std::uint8_t> payload(info.frame_length, 0x5A);
    return std::make_unique<test::TestFrame>(info, std::move(payload));
}

bool wait_for_state(detail::CameraImpl& camera, hal::DeviceState wanted,
                    std::chrono::milliseconds budget = std::chrono::milliseconds(2000)) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
        if (camera.state() == wanted) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return camera.state() == wanted;
}

/// @brief 反复读直到拿到帧；`kNotOpen` 之类的状态错误立刻返回。
Result<void> read_until_frame(detail::CameraImpl& camera,
                              hal::FrameLease<hal::ImageFrame>& lease,
                              int attempts = 300) {
    Result<void> last = Result<void>::failure(
        make_error(ErrorCode::kInternal, "read_until_frame: 没有尝试过"));
    for (int attempt = 0; attempt < attempts; ++attempt) {
        last = camera.read(lease, time::Duration::from_milliseconds(20));
        if (last.ok()) {
            return last;
        }
        if (last.error().code != ErrorCode::kTimeout) {
            return last;
        }
    }
    return last;
}

/**
 * @brief 一套可复用的「一台 GigE 相机 + mock 后端」。
 *
 * `make()` 之前可以随意改 `config` / `api` 的可编程字段；`make()` 之后相机持有
 * 配置副本，改动不再生效。
 */
struct Harness {
    std::shared_ptr<test::MockMvsApi> api = std::make_shared<test::MockMvsApi>();
    hal::CameraConfig config;
    std::unique_ptr<detail::CameraImpl> camera;

    Harness() {
        detail::DeviceDescriptor device;
        device.serial = "DB0985207";
        device.user_defined_name = "front";
        device.model = "MV-CS004-10GC";
        device.firmware = "V1.0.0 build20251231";
        device.transport = hal::Transport::kGigE;
        device.ip_address = "192.168.1.51";
        api->devices.push_back(device);

        // open() 以设备读回的分辨率为准，所以必须给 mock 配上。
        api->int_values[std::string(node::kWidth)] = 720;
        api->int_values[std::string(node::kHeight)] = 540;

        config.selector.serial = "DB0985207";
        config.width = 720;
        config.height = 540;
        config.buffer_count = 4;
        config.pixel_format = hal::PixelFormat::kMono8;
    }

    void make() {
        camera = std::make_unique<detail::CameraImpl>(config, api);
    }

    /// @brief 构造并 open()，断言成功。
    void open_ok() {
        make();
        ASSERT_NE(camera, nullptr);
        const auto status = camera->open();
        ASSERT_TRUE(status.ok()) << status.error().message;
        ASSERT_EQ(camera->state(), hal::DeviceState::kOpen);
    }

    /// @brief 构造 + open() + start()，断言都成功。
    void stream_ok() {
        open_ok();
        const auto status = camera->start();
        ASSERT_TRUE(status.ok()) << status.error().message;
        ASSERT_EQ(camera->state(), hal::DeviceState::kStreaming);
    }
};

// ---------------------------------------------------------------------------
// 设备选择
// ---------------------------------------------------------------------------

/// @brief 空 selector 不得默认取第 0 台（HAL 明令禁止）。
TEST(HikvisionCamera, OpenRejectsEmptySelector) {
    Harness harness;
    harness.config.selector = hal::DeviceSelector{};
    harness.make();

    const auto status = harness.camera->open();

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kInvalidArgument);
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kClosed);
}

/// @brief serial 未命中时给出 `kDeviceNotFound`，不静默退回其它设备。
TEST(HikvisionCamera, OpenReportsDeviceNotFound) {
    Harness harness;
    harness.config.selector.serial = "NOPE";
    harness.make();

    const auto status = harness.camera->open();

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kDeviceNotFound);
}

/**
 * @brief 同型号多台且未给确认条件时必须 `kDeviceAmbiguous`。
 *
 * `DeviceSelector::index` 默认 0，无法区分「没写」与「要第 0 台」，因此驱动要求
 * 显式条件而不是猜。
 */
TEST(HikvisionCamera, OpenReportsAmbiguousModel) {
    Harness harness;
    detail::DeviceDescriptor second = harness.api->devices.front();
    second.serial = "DB0985208";
    second.ip_address = "192.168.1.52";
    harness.api->devices.push_back(second);

    harness.config.selector.serial.clear();
    harness.config.selector.model = "MV-CS004-10GC";
    harness.make();

    const auto status = harness.camera->open();

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kDeviceAmbiguous);
}

/// @brief 显式 `index` 可以选中同型号的第二台。
TEST(HikvisionCamera, OpenHonoursExplicitIndex) {
    Harness harness;
    detail::DeviceDescriptor second = harness.api->devices.front();
    second.serial = "DB0985208";
    second.ip_address = "192.168.1.52";
    harness.api->devices.push_back(second);

    harness.config.selector.serial.clear();
    harness.config.selector.model = "MV-CS004-10GC";
    harness.config.selector.index = 1;
    harness.make();

    const auto status = harness.camera->open();

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_EQ(harness.camera->info().serial, "DB0985208");
}

/**
 * @brief 同一台相机被枚举两次（主机双 IP）时必须按 serial 去重。
 *
 * 现场症状：`br-lidar` 同时有 /24 与 /16 地址，MVS 枚举返回 `nDeviceNum=2`，
 * 两条记录 serial/IP 完全相同。不去重会让 model 路径误判歧义。
 */
TEST(HikvisionCamera, OpenDeduplicatesSameSerial) {
    Harness harness;
    detail::DeviceDescriptor duplicate = harness.api->devices.front();
    duplicate.ip_address = "169.254.114.1";
    harness.api->devices.push_back(duplicate);

    harness.config.selector.serial.clear();
    harness.config.selector.model = "MV-CS004-10GC";
    harness.make();

    const auto status = harness.camera->open();

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_EQ(harness.camera->info().ip_address, "192.168.1.51");
}

// ---------------------------------------------------------------------------
// 生命周期
// ---------------------------------------------------------------------------

/// @brief open() 成功后状态与 `SensorInfo` 都要就位。
TEST(HikvisionCamera, OpenFillsInfo) {
    Harness harness;
    harness.open_ok();

    const auto info = harness.camera->info();
    EXPECT_EQ(info.vendor, "Hikrobot");
    EXPECT_EQ(info.model, "MV-CS004-10GC");
    EXPECT_EQ(info.serial, "DB0985207");
    EXPECT_EQ(info.user_defined_name, "front");
    EXPECT_EQ(info.firmware_version, "V1.0.0 build20251231");
    EXPECT_EQ(info.transport, hal::Transport::kGigE);
    EXPECT_EQ(info.ip_address, "192.168.1.51");
    EXPECT_EQ(info.frame_id, "camera");
}

/// @brief open() 不可重入。
TEST(HikvisionCamera, OpenIsNotIdempotent) {
    Harness harness;
    harness.open_ok();

    const auto status = harness.camera->open();

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kAlreadyOpen);
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kOpen);
}

/// @brief 没 open 就 start 要报 `kNotOpen`。
TEST(HikvisionCamera, StartRequiresOpen) {
    Harness harness;
    harness.make();

    const auto status = harness.camera->start();

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kNotOpen);
}

/**
 * @brief `MV_CC_SetImageNodeNum` 必须早于 `MV_CC_StartGrabbing`。
 *
 * SDK 要求节点数在开流前设定，顺序错了会被服务器静默忽略，表现为高帧率下丢帧。
 */
TEST(HikvisionCamera, StartSetsImageNodeCountBeforeGrabbing) {
    Harness harness;
    harness.stream_ok();

    const auto events = harness.api->snapshot_events();
    std::size_t node_count_index = events.size();
    std::size_t grabbing_index = events.size();
    for (std::size_t index = 0; index < events.size(); ++index) {
        if (events[index] == "set:MV_CC_SetImageNodeNum") {
            node_count_index = index;
        }
        if (events[index] == "start_grabbing" && grabbing_index == events.size()) {
            grabbing_index = index;
        }
    }
    ASSERT_NE(node_count_index, events.size()) << "未调用 MV_CC_SetImageNodeNum";
    ASSERT_NE(grabbing_index, events.size()) << "未调用 MV_CC_StartGrabbing";
    EXPECT_LT(node_count_index, grabbing_index);
}

/// @brief start() 幂等：重复调用不再触发 `StartGrabbing`。
TEST(HikvisionCamera, StartIsIdempotent) {
    Harness harness;
    harness.stream_ok();

    const auto again = harness.camera->start();

    EXPECT_TRUE(again.ok());
    EXPECT_EQ(harness.api->start_grabbing_calls.load(), 1);
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kStreaming);
}

/// @brief stop() 回到 kOpen，并停掉 SDK 取流。
TEST(HikvisionCamera, StopReturnsToOpen) {
    Harness harness;
    harness.stream_ok();

    const auto status = harness.camera->stop();

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kOpen);
    EXPECT_GE(harness.api->stop_grabbing_calls.load(), 1);
}

/// @brief close() 必须销毁句柄，避免下一次 open() 泄漏。
TEST(HikvisionCamera, CloseDestroysHandle) {
    Harness harness;
    harness.open_ok();

    const auto status = harness.camera->close();

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kClosed);
    EXPECT_EQ(harness.api->close_calls.load(), 1);
    EXPECT_EQ(harness.api->destroy_calls.load(), 1);
}

/**
 * @brief 关设备失败也必须销毁句柄。
 *
 * 否则设备留在「已被占用」状态，下一次 open() 会拿到 `MV_E_ACCESS_DENIED`。
 */
TEST(HikvisionCamera, CloseDestroysHandleEvenWhenCloseFails) {
    Harness harness;
    harness.open_ok();
    harness.api->close_error = detail::mvs_error::kHandle;

    const auto status = harness.camera->close();

    EXPECT_TRUE(status.ok());
    EXPECT_EQ(harness.api->close_calls.load(), 1);
    EXPECT_EQ(harness.api->destroy_calls.load(), 1);
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kClosed);
}

/// @brief close() 幂等，不得重复销毁句柄。
TEST(HikvisionCamera, CloseIsIdempotent) {
    Harness harness;
    harness.open_ok();

    ASSERT_TRUE(harness.camera->close().ok());
    ASSERT_TRUE(harness.camera->close().ok());

    EXPECT_EQ(harness.api->destroy_calls.load(), 1);
}

/// @brief 从 streaming 直接 close()，线程要先停再释放。
TEST(HikvisionCamera, CloseStopsStreamingFirst) {
    Harness harness;
    harness.stream_ok();

    const auto status = harness.camera->close();

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kClosed);
    EXPECT_EQ(harness.api->destroy_calls.load(), 1);
}

/// @brief 未 open 就 recover() 要报 `kNotOpen`。
TEST(HikvisionCamera, RecoverRequiresOpen) {
    Harness harness;
    harness.make();

    const auto status = harness.camera->recover();

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kNotOpen);
}

/// @brief 异常之后 recover() 重新 open + start，并累计 reconnects。
TEST(HikvisionCamera, RecoverReopensAndRestreams) {
    Harness harness;
    harness.stream_ok();

    harness.api->fire_exception(detail::mvs_error::kUdpRecvData);
    ASSERT_TRUE(wait_for_state(*harness.camera, hal::DeviceState::kError));

    const auto status = harness.camera->recover();

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kStreaming);
    EXPECT_GE(harness.camera->stats().reconnects, 1U);
}

// ---------------------------------------------------------------------------
// 取流与读帧
// ---------------------------------------------------------------------------

/// @brief 未 start 时 read() 必须 `kNotOpen`。
TEST(HikvisionCamera, ReadRequiresStreaming) {
    Harness harness;
    harness.open_ok();

    hal::FrameLease<hal::ImageFrame> lease;
    const auto status = harness.camera->read(lease, time::Duration::from_milliseconds(10));

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kNotOpen);
}

/// @brief 正常的帧要原样透传：尺寸、格式、序号、载荷。
TEST(HikvisionCamera, ReadsPublishedFrame) {
    Harness harness;
    harness.stream_ok();
    harness.api->push_frame(make_frame(mono8_frame_info(7)));

    hal::FrameLease<hal::ImageFrame> lease;
    const auto status = read_until_frame(*harness.camera, lease);

    ASSERT_TRUE(status.ok()) << status.error().message;
    ASSERT_TRUE(static_cast<bool>(lease));
    const hal::ImageFrame& frame = *lease;
    EXPECT_EQ(frame.width, 720U);
    EXPECT_EQ(frame.height, 540U);
    EXPECT_EQ(frame.stride, 720U);
    EXPECT_EQ(frame.pixel_format, hal::PixelFormat::kMono8);
    EXPECT_EQ(frame.device_frame_number, 7U);
    EXPECT_EQ(frame.size, 720U * 540U);
    ASSERT_NE(frame.data, nullptr);
    EXPECT_EQ(frame.data[0], 0x5A);
    EXPECT_EQ(frame.data[frame.size - 1], 0x5A);
    EXPECT_EQ(harness.camera->stats().frames_delivered, 1U);
}

/// @brief 拿到帧时 `receive_stamp` 必须非零，`sync` 至少是主机接收。
TEST(HikvisionCamera, FrameCarriesTiming) {
    Harness harness;
    harness.stream_ok();
    harness.api->push_frame(make_frame(mono8_frame_info()));

    hal::FrameLease<hal::ImageFrame> lease;
    ASSERT_TRUE(read_until_frame(*harness.camera, lease).ok());

    const hal::ImageFrame& frame = *lease;
    EXPECT_EQ(frame.sync, hal::TimestampSync::kHostReceive);
    EXPECT_FALSE(frame.stamp.is_zero());
    EXPECT_FALSE(frame.receive_stamp.is_zero());
}

/// @brief 没有帧时 read() 超时，并累计 timeouts。
TEST(HikvisionCamera, ReadTimesOutAndCounts) {
    Harness harness;
    harness.stream_ok();

    hal::FrameLease<hal::ImageFrame> lease;
    const auto status = harness.camera->read(lease, time::Duration::from_milliseconds(30));

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kTimeout);
    EXPECT_FALSE(static_cast<bool>(lease));
    EXPECT_GE(harness.camera->stats().timeouts, 1U);
}

/**
 * @brief 帧长度小于 `stride * height` 时丢弃并计 SDK 错误。
 *
 * 这种帧是「元数据与分辨率不自洽」的损坏帧，拷进帧池只会让上层解到垃圾。
 */
TEST(HikvisionCamera, RejectsUndersizedFrame) {
    Harness harness;
    harness.stream_ok();

    auto info = mono8_frame_info();
    info.frame_length = 720U * 540U - 1U;
    harness.api->push_frame(make_frame(info));

    hal::FrameLease<hal::ImageFrame> lease;
    const auto status = harness.camera->read(lease, time::Duration::from_milliseconds(60));

    EXPECT_FALSE(status.ok());
    EXPECT_GE(harness.camera->stats().sdk_errors, 1U);

    // 合法帧到达后仍能正常读取，说明丢弃没有破坏状态机。
    harness.api->push_frame(make_frame(mono8_frame_info()));
    ASSERT_TRUE(read_until_frame(*harness.camera, lease).ok());
}

/// @brief 非 8 位格式的帧必须拒绝，不能靠调用方猜布局。
TEST(HikvisionCamera, RejectsUnsupportedPixelFormat) {
    Harness harness;
    harness.stream_ok();

    auto info = mono8_frame_info();
    info.pixel_format = hal::PixelFormat::kMono12;
    harness.api->push_frame(make_frame(info));

    hal::FrameLease<hal::ImageFrame> lease;
    const auto status = harness.camera->read(lease, time::Duration::from_milliseconds(60));

    EXPECT_FALSE(status.ok());
    EXPECT_GE(harness.camera->stats().sdk_errors, 1U);
}

// ---------------------------------------------------------------------------
// 运行期调参与软触发
// ---------------------------------------------------------------------------

/// @brief 未 open 时 apply() 返回 `kNotOpen`。
TEST(HikvisionCamera, ApplyRequiresOpen) {
    Harness harness;
    harness.make();

    hal::CameraTuning tuning;
    tuning.exposure_us = std::optional<double>(1234.0);
    const auto status = harness.camera->apply(tuning);

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kNotOpen);
}

/// @brief 运行期改曝光要落到 `ExposureTime` 节点。
TEST(HikvisionCamera, ApplyWritesExposure) {
    Harness harness;
    harness.open_ok();

    hal::CameraTuning tuning;
    tuning.exposure_us = std::optional<double>(1234.0);
    const auto status = harness.camera->apply(tuning);

    ASSERT_TRUE(status.ok()) << status.error().message;
    const auto exposure = harness.api->last_write(node::kExposureTime);
    ASSERT_TRUE(exposure.has_value());
    EXPECT_NE(exposure->find("1234"), std::string::npos);
}

/// @brief `set_exposure()` 走同一套映射。
TEST(HikvisionCamera, SetExposureDelegatesToApply) {
    Harness harness;
    harness.open_ok();

    const auto status = harness.camera->set_exposure(2500.0);

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_TRUE(harness.api->last_write(node::kExposureTime).has_value());
}

/// @brief 段落级失败要保留 MVS 原始码，便于现场定位。
TEST(HikvisionCamera, ApplyFailureKeepsVendorCode) {
    Harness harness;
    harness.open_ok();
    harness.api->failing_nodes.emplace(std::string(node::kExposureTime), detail::mvs_error::kAccessDenied);

    hal::CameraTuning tuning;
    tuning.exposure_us = std::optional<double>(1234.0);
    const auto status = harness.camera->apply(tuning);

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kPermissionDenied);
    EXPECT_EQ(static_cast<std::uint32_t>(harness.camera->stats().last_error_vendor_code),
              detail::mvs_error::kAccessDenied);
}

/// @brief 非软触发模式下 trigger() 返回 `kNotSupported`。
TEST(HikvisionCamera, TriggerRequiresSoftwareMode) {
    Harness harness;
    harness.stream_ok();

    const auto status = harness.camera->trigger();

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kNotSupported);
    EXPECT_EQ(harness.api->trigger_calls.load(), 0);
}

/// @brief 软触发模式下 trigger() 调用 SDK 并转发错误。
TEST(HikvisionCamera, TriggerInvokesSdk) {
    Harness harness;
    harness.config.trigger.mode = hal::TriggerMode::kSoftware;
    harness.stream_ok();

    const auto status = harness.camera->trigger();

    ASSERT_TRUE(status.ok()) << status.error().message;
    EXPECT_EQ(harness.api->trigger_calls.load(), 1);
}

/// @brief 未开流时触发没有意义，返回 `kNotOpen`。
TEST(HikvisionCamera, TriggerRequiresStreaming) {
    Harness harness;
    harness.config.trigger.mode = hal::TriggerMode::kSoftware;
    harness.open_ok();

    const auto status = harness.camera->trigger();

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kNotOpen);
}

// ---------------------------------------------------------------------------
// 故障与自检
// ---------------------------------------------------------------------------

/**
 * @brief SDK 异常回调只把状态置 `kError`，不在 SDK 线程里碰句柄。
 *
 * 这里让回调在测试线程上同步触发，语义等价：状态必须立刻变，且不抛异常。
 */
TEST(HikvisionCamera, ExceptionCallbackMarksError) {
    Harness harness;
    harness.stream_ok();

    harness.api->fire_exception(detail::mvs_error::kUdpRecvData);

    EXPECT_TRUE(wait_for_state(*harness.camera, hal::DeviceState::kError));
    EXPECT_GE(harness.camera->stats().sdk_errors, 1U);
    EXPECT_EQ(static_cast<std::uint32_t>(harness.camera->stats().last_error_vendor_code),
              detail::mvs_error::kUdpRecvData);
}

/// @brief 连续 5 次取帧失败就判定链路已坏，不等上层超时。
TEST(HikvisionCamera, ConsecutiveGrabErrorsMarkError) {
    Harness harness;
    harness.api->frame_error = detail::mvs_error::kUdpRecvData;
    harness.stream_ok();

    EXPECT_TRUE(wait_for_state(*harness.camera, hal::DeviceState::kError));
    EXPECT_GE(harness.camera->stats().sdk_errors, 5U);
}

/// @brief `supported_keys()` 必须声明驱动认识的键，供配置期自检。
TEST(HikvisionCamera, SupportedKeysCoverTuningSurface) {
    Harness harness;
    harness.open_ok();

    const auto keys = harness.camera->supported_keys();

    ASSERT_FALSE(keys.empty());
    const auto contains = [&keys](const std::string& key) {
        for (const auto& item : keys) {
            if (item == key) {
                return true;
            }
        }
        return false;
    };
    EXPECT_TRUE(contains("camera.exposure_us"));
    EXPECT_TRUE(contains("camera.gain_db"));
    EXPECT_TRUE(contains("camera.trigger_debounce_us"));
    EXPECT_TRUE(contains("camera.timestamp_source"));
}

/// @brief 构造阶段不做任何 I/O：make() 之后必须仍是 `kClosed` 且没枚举。
TEST(HikvisionCamera, ConstructionTouchesNoDevice) {
    Harness harness;
    harness.make();

    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kClosed);
    EXPECT_EQ(harness.api->enumerate_calls.load(), 0);
    EXPECT_EQ(harness.api->create_calls.load(), 0);
}

} // namespace
} // namespace foray::platform::driver::hikvision
