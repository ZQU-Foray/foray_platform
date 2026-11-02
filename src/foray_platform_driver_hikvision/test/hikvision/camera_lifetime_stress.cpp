/**
 * @file camera_lifetime_stress.cpp
 * @brief 审计 H8 的回归用例：`read()` 与 `close()`/`recover()` 并发时的帧池生命周期。
 *
 * 这里**故意不注册进 ctest**：默认迭代 10 万次，跑一遍要一分钟上下，放进 CI 只会
 * 让常规回归变慢。文件始终参与编译（所以不会腐烂），回执里直接跑二进制：
 *
 * ```bash
 * ./build/foray_platform_driver_hikvision/foray_platform_driver_hikvision_stress
 * FORAY_STRESS_ITERATIONS=20000 ./.../foray_platform_driver_hikvision_stress  # TSan 版
 * ```
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "foray/platform/hal/frame_lease.hpp"
#include "foray/platform/result/error.hpp"
#include "foray/platform/time/duration.hpp"
#include "hikvision/camera_impl.hpp"
#include "hikvision/mvs_nodes.hpp"
#include "mock_mvs_api.hpp"

namespace foray::platform::driver::hikvision {
namespace {

namespace node = detail::node;

/// @brief 10 万次是审计任务书给的验收下限；调试时可临时调小。
constexpr std::size_t kDefaultIterations = 100'000;

std::size_t stress_iterations() {
    const char* raw = std::getenv("FORAY_STRESS_ITERATIONS");
    if (raw == nullptr) {
        return kDefaultIterations;
    }
    const long long parsed = std::strtoll(raw, nullptr, 10);
    if (parsed <= 0) {
        return kDefaultIterations;
    }
    return static_cast<std::size_t>(parsed);
}

/// @brief 一帧 720×540 Mono8 的测试载荷。
detail::FrameInfoView stress_frame_info(std::uint32_t frame_number) {
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

std::unique_ptr<detail::FrameView> make_frame(std::uint32_t frame_number) {
    const auto info = stress_frame_info(frame_number);
    std::vector<std::uint8_t> payload(info.frame_length, 0x5A);
    return std::make_unique<test::TestFrame>(info, std::move(payload));
}

/// @brief 一台 GigE 相机 + mock 后端，配置与 camera_test.cpp 的 Harness 保持一致。
struct StressHarness {
    std::shared_ptr<test::MockMvsApi> api = std::make_shared<test::MockMvsApi>();
    hal::CameraConfig config;
    std::unique_ptr<detail::CameraImpl> camera;

    StressHarness() {
        detail::DeviceDescriptor device;
        device.serial = "DB0985207";
        device.user_defined_name = "front";
        device.model = "MV-CS004-10GC";
        device.firmware = "V1.0.0 build20251231";
        device.transport = hal::Transport::kGigE;
        device.ip_address = "192.168.1.51";
        api->devices.push_back(device);
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
};

/// @brief 消费线程主体：一直 `read()`，直到主线程喊停。
struct ReaderState {
    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> calls{0};
    std::atomic<std::uint64_t> frames{0};
};

void run_reader(detail::CameraImpl& camera, ReaderState& state) {
    while (!state.stop.load(std::memory_order_acquire)) {
        hal::FrameLease<hal::ImageFrame> lease;
        const auto status = camera.read(lease, time::Duration::from_nanoseconds(200'000));
        state.calls.fetch_add(1, std::memory_order_relaxed);
        if (status.ok()) {
            state.frames.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

/**
 * @brief 消费线程持续 `read()`，控制线程反复 `open/start/close`（并周期性 `recover()`）。
 *
 * 审计 H8 的现场是 `read()` 拿着裸 `pool_` 成员指针、`close()` 在另一线程
 * `pool_.reset()`：修好之前这里就是 use-after-free。修好之后整段没有任何
 * `read()` 能碰到已释放的帧池，TSan 也不该报 race。
 */
TEST(HikvisionCameraStress, ReadSurvivesCloseAndRecover) {
    const std::size_t iterations = stress_iterations();
    StressHarness harness;
    harness.make();
    ASSERT_NE(harness.camera, nullptr);

    ReaderState state;
    std::vector<std::thread> readers;
    readers.reserve(3);
    for (int index = 0; index < 3; ++index) {
        readers.emplace_back(run_reader, std::ref(*harness.camera), std::ref(state));
    }

    const auto sleep_between = std::chrono::microseconds(100);
    std::uint32_t frame_number = 1;
    for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        const auto opened = harness.camera->open();
        ASSERT_TRUE(opened.ok()) << opened.error().message;
        const auto started = harness.camera->start();
        ASSERT_TRUE(started.ok()) << started.error().message;

        // 让取流线程真的走到「取一帧 → 往池里写 → 发布」这条路上。
        harness.api->push_frame(make_frame(frame_number++));
        harness.api->push_frame(make_frame(frame_number++));
        std::this_thread::sleep_for(sleep_between);

        if (iteration % 1000 == 999) {
            const auto recovered = harness.camera->recover();
            ASSERT_TRUE(recovered.ok()) << recovered.error().message;
        }

        const auto closed = harness.camera->close();
        ASSERT_TRUE(closed.ok()) << closed.error().message;
    }

    state.stop.store(true, std::memory_order_release);
    for (auto& reader : readers) {
        reader.join();
    }

    EXPECT_GT(state.calls.load(), 0ULL);
    EXPECT_GT(state.frames.load(), 0ULL);
}

} // namespace
} // namespace foray::platform::driver::hikvision
