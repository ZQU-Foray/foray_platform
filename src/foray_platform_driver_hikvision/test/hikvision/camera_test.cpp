#include <gtest/gtest.h>

#include <atomic>
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

        // 设备开机默认手动曝光/增益：运行期写 ExposureTime / Gain 的前置条件（审计 H11）。
        api->enum_values.emplace(std::string(node::kExposureAuto), "Off");
        api->enum_values.emplace(std::string(node::kGainAuto), "Off");

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

/// @brief H10：异常回调注册失败必须让 open() 失败，并把已拿到的句柄还回去。
TEST(HikvisionCamera, OpenFailsWhenExceptionRegistrationFails) {
    Harness harness;
    harness.api->register_exception_error = detail::mvs_error::kHandle;
    harness.make();
    ASSERT_NE(harness.camera, nullptr);

    const auto status = harness.camera->open();

    EXPECT_FALSE(status.ok());
    // MV_E_HANDLE 经 mvs_error 映射为 kInternal，原始码保留在 vendor_code 里。
    EXPECT_EQ(status.error().code, ErrorCode::kInternal);
    EXPECT_EQ(status.error().vendor_code, static_cast<std::int32_t>(detail::mvs_error::kHandle));
    EXPECT_NE(status.error().message.find("MV_CC_RegisterExceptionCallBack"), std::string::npos);
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kClosed);
    EXPECT_EQ(harness.api->register_exception_calls.load(), 1);
    // 失败路径必须销毁句柄，不能留下悬挂设备。
    EXPECT_EQ(harness.api->destroy_calls.load(), 1);
}

/// @brief H10：close() 期间 SDK 线程仍在触发异常回调，不得崩溃或读已释放内存。
TEST(HikvisionCamera, CloseToleratesConcurrentExceptionCallback) {
    Harness harness;
    harness.stream_ok();

    std::atomic<bool> stop{false};
    std::thread firing([&harness, &stop] {
        while (!stop.load(std::memory_order_acquire)) {
            harness.api->fire_exception(detail::mvs_error::kUdpRecvData);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    const auto status = harness.camera->close();
    stop.store(true, std::memory_order_release);
    firing.join();

    EXPECT_TRUE(status.ok());
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kClosed);
    EXPECT_EQ(harness.api->destroy_calls.load(), 1);
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

/**
 * @brief 审计 H15：`recover()` 不许把 `sequence_` 与会话统计清零。
 *
 * 改前 `recover()` 复用 `open_locked()`，而 `open_locked()` 里顺手把 `sequence_`、
 * `frames_received_`、`timeouts_` 等全部归零：自动恢复一次之后，消费者看到 sequence
 * 从 3 掉回 1（断帧/乱序判定直接失效），统计看起来像"从没收到过帧"——现场只能看到
 * `reconnects` 涨了，于是"掉线过，但一帧没丢"。
 *
 * 现在清零只在用户主动 `open()` 时发生（`reset_session_counters_locked()`），
 * 恢复后计数器接着往上走。
 */
TEST(HikvisionCamera, RecoverKeepsSequenceAndSessionCounters) {
    Harness harness;
    harness.stream_ok();

    hal::FrameLease<hal::ImageFrame> lease;
    for (std::uint32_t index = 1; index <= 3; ++index) {
        harness.api->push_frame(make_frame(mono8_frame_info(index)));
        ASSERT_TRUE(read_until_frame(*harness.camera, lease).ok()) << "第 " << index << " 帧";
        ASSERT_EQ(lease->sequence, index);
    }
    const auto before = harness.camera->stats();
    ASSERT_EQ(before.frames_received, 3U);
    ASSERT_EQ(before.last_sequence, 3U);

    harness.api->fire_exception(detail::mvs_error::kUdpRecvData);
    ASSERT_TRUE(wait_for_state(*harness.camera, hal::DeviceState::kError));
    ASSERT_TRUE(harness.camera->recover().ok());

    const auto after = harness.camera->stats();
    EXPECT_GE(after.reconnects, 1U);
    EXPECT_EQ(after.frames_received, before.frames_received);
    EXPECT_EQ(after.last_sequence, before.last_sequence);
    EXPECT_GE(after.timeouts, before.timeouts);

    // 序号续着走，不回退。
    harness.api->push_frame(make_frame(mono8_frame_info(4)));
    ASSERT_TRUE(read_until_frame(*harness.camera, lease).ok());
    EXPECT_EQ(lease->sequence, 4U);
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

/**
 * @brief 审计 H26 附带项③：PTP 在 open() 之后才收敛，状态必须能被重新读到。
 *
 * 改前 `GevIEEE1588Status` / `DeviceTicksFrequency` 只在 `open()` 读一次：开机的
 * 那一瞬相机还在 `Master`，于是整场运行都拿不到 `kPtp`，所有帧只能退到
 * 「主机接收时刻」——明明 PTP 后来同步上了，时间戳精度却一直是最差的那档。
 */
TEST(HikvisionCamera, PtpSlaveDetectedMidStreamUpgradesTiming) {
    Harness harness;
    harness.api->enum_values[std::string(node::kPtpStatus)] = "Master";
    harness.api->int_values[std::string(node::kDeviceTicksFrequency)] = 1000000000;
    harness.stream_ok();

    detail::FrameInfoView info = mono8_frame_info();
    info.device_ticks = 5000000000ULL; // 5 s（1 GHz tick）
    harness.api->push_frame(make_frame(info));

    hal::FrameLease<hal::ImageFrame> lease;
    ASSERT_TRUE(read_until_frame(*harness.camera, lease).ok()) << "首帧读取超时";
    EXPECT_EQ(lease->sync, hal::TimestampSync::kHostReceive);

    // PTP 收敛：设备侧状态变成 Slave。取流线程要在一个探测周期内读到它。
    harness.api->enum_values[std::string(node::kPtpStatus)] = "Slave";
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    detail::FrameInfoView info2 = mono8_frame_info(2);
    info2.device_ticks = 6000000000ULL; // 6 s
    harness.api->push_frame(make_frame(info2));

    hal::FrameLease<hal::ImageFrame> lease2;
    ASSERT_TRUE(read_until_frame(*harness.camera, lease2).ok()) << "第二帧读取超时";
    EXPECT_EQ(lease2->sync, hal::TimestampSync::kPtp);
    EXPECT_EQ(lease2->device_time_ns, 6000000000LL);
    EXPECT_EQ(lease2->stamp.nanoseconds_since_epoch(), 6000000000LL);
}

/**
 * @brief 审计 H9：`frame_id` 打开期固定，`apply()` 必须整包拒绝而不是悄悄忽略。
 *
 * 已交付租约的 `ImageFrame::frame_id` 是指向 `frame_id_` 存储的 `string_view`，
 * 热改会让在手的租约变成悬垂视图。这条用例同时守住两件事：`apply()` 返回
 * `kNotSupported`，以及旧租约的 id 读数仍然有效、内容未变（ASan 下即 UAF 检查）。
 */
TEST(HikvisionCamera, ApplyRejectsFrameIdChange) {
    Harness harness;
    harness.stream_ok();
    harness.api->push_frame(make_frame(mono8_frame_info()));

    hal::FrameLease<hal::ImageFrame> lease;
    ASSERT_TRUE(read_until_frame(*harness.camera, lease).ok());
    const std::string stable_id(lease->frame_id);
    ASSERT_EQ(stable_id, harness.config.frame_id);

    hal::CameraTuning tuning;
    tuning.frame_id = std::string("changed_by_apply");
    const auto status = harness.camera->apply(tuning);

    ASSERT_FALSE(status.ok()) << "apply() 必须拒绝热改 frame_id";
    EXPECT_EQ(status.error().code, ErrorCode::kNotSupported);
    // 整包拒绝：相机状态与帧标识都不受影响。
    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kStreaming);
    EXPECT_EQ(lease->frame_id, stable_id);
    EXPECT_EQ(harness.camera->info().frame_id, stable_id);
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

/**
 * @brief 审计 H14：帧几何与帧池不一致时必须整帧拒绝，不能截断后仍宣告新尺寸。
 *
 * 改前只校验 `frame_length >= width*height`，然后 `copy = min(frame_length,
 * payload_size)`：设备换几何（固件 ROI 对齐修正、恢复 user set、在线改分辨率）后
 * 帧变大时只拷槽位容量，`out.width/height/stride` 却写的是新值——消费者按
 * `data + y*stride` 逐行读会越过槽位缓冲，而且这帧不进任何丢帧计数器。
 *
 * 现在几何不一致直接 `sdk_errors_++` + `frames_dropped_++` 丢弃，一帧都不发布。
 */
TEST(HikvisionCamera, RejectsFrameGeometryChangeInsteadOfTruncating) {
    Harness harness;
    harness.stream_ok();

    // 池按 720×540 分配；这帧报 721×540，比槽位大一行像素。
    auto info = mono8_frame_info();
    info.width = 721U;
    info.height = 540U;
    info.frame_length = 721U * 540U;
    harness.api->push_frame(make_frame(info));

    hal::FrameLease<hal::ImageFrame> lease;
    const auto status = harness.camera->read(lease, time::Duration::from_milliseconds(60));

    EXPECT_FALSE(status.ok());
    EXPECT_FALSE(static_cast<bool>(lease));
    EXPECT_GE(harness.camera->stats().sdk_errors, 1U);
    EXPECT_GE(harness.camera->stats().frames_dropped, 1U);
    EXPECT_EQ(harness.camera->stats().frames_received, 0U);

    // 几何恢复一致的帧照样能读，说明拒绝没有破坏状态机。
    harness.api->push_frame(make_frame(mono8_frame_info(2)));
    ASSERT_TRUE(read_until_frame(*harness.camera, lease).ok());
    EXPECT_EQ(lease->width, 720U);
    EXPECT_EQ(lease->height, 540U);
    EXPECT_EQ(lease->size, 720U * 540U);
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

/// @brief 自动曝光下 `set_exposure()` 必须失败并说清原因（审计 H11）。
TEST(HikvisionCamera, SetExposureRejectedWhileAuto) {
    Harness harness;
    harness.open_ok();
    harness.api->enum_values[std::string(node::kExposureAuto)] = "Continuous";

    const auto status = harness.camera->set_exposure(2500.0);

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kNotSupported);
    EXPECT_FALSE(harness.api->last_write(node::kExposureTime).has_value());
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

/**
 * @brief 审计 H13：自由运行时「一直没数据」必须判成停摆，而不是永远静默等待。
 *
 * 改前的实现把 `MV_E_NODATA` 与超时一起当正常控制流 `continue`，相机会一直停在
 * `kStreaming`、`read()` 永远返回 `kTimeout`，上层没有任何线索去调 `recover()`。
 * 现在连续 `kMaxConsecutiveStarvations`（25 次，对应真实设备约 2.5 s）没数据即置
 * `kError`，`read()` 随之返回 `kNotOpen` 并把当前状态带出来。
 */
TEST(HikvisionCamera, FreeRunStarvationMarksError) {
    Harness harness;
    harness.stream_ok();

    EXPECT_TRUE(
        wait_for_state(*harness.camera, hal::DeviceState::kError, std::chrono::milliseconds(3000)));
    EXPECT_GE(harness.camera->stats().timeouts, 1U);
    EXPECT_GE(harness.camera->stats().sdk_errors, 1U);

    hal::FrameLease<hal::ImageFrame> lease;
    const auto status = harness.camera->read(lease, time::Duration::from_milliseconds(10));

    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.error().code, ErrorCode::kNotOpen);
}

/// @brief 触发模式下等触发是正常状态：同样没数据，但不能判成停摆。
TEST(HikvisionCamera, TriggerModeDoesNotTreatSilenceAsStall) {
    Harness harness;
    harness.config.trigger.mode = hal::TriggerMode::kSoftware;
    harness.stream_ok();

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    EXPECT_EQ(harness.camera->state(), hal::DeviceState::kStreaming);
    EXPECT_GE(harness.camera->stats().timeouts, 1U);
    EXPECT_EQ(harness.camera->stats().sdk_errors, 0U);
}

/**
 * @brief `MV_E_GC_TIMEOUT` 是真实故障，即使处在等触发的模式也必须计入失败。
 *
 * 改前它与 `MV_E_NODATA` 混在同一个 `continue` 分支里，GenICam 控制通道超时被当成
 * 「这次没数据」，链路坏掉也不会进 `kError`（审计 H13）。
 */
TEST(HikvisionCamera, GenicamTimeoutIsAnSdkErrorOnEveryPath) {
    Harness harness;
    harness.config.trigger.mode = hal::TriggerMode::kSoftware;
    harness.api->frame_error = detail::mvs_error::kGcTimeout;
    harness.stream_ok();

    EXPECT_TRUE(wait_for_state(*harness.camera, hal::DeviceState::kError));
    EXPECT_GE(harness.camera->stats().sdk_errors, 5U);
    EXPECT_EQ(static_cast<std::uint32_t>(harness.camera->stats().last_error_vendor_code),
              detail::mvs_error::kGcTimeout);
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
