#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include "foray/platform/container/spsc_queue.hpp"
#include "livox/livox_runtime.hpp"

namespace foray::platform::driver::livox::detail {
namespace {

constexpr std::size_t kPacketsPerProducer = 2000;
constexpr std::uint32_t kQueueSlots = 1U << 16;

std::array<std::uint8_t, 64> make_packet(std::uint8_t tag) {
    std::array<std::uint8_t, 64> bytes{};
    bytes.fill(tag);
    return bytes;
}

} // namespace

/**
 * @brief 审计 H20：点云与 IMU 两条 SDK 回调可能在不同线程上，队列不能被写坏。
 *
 * 官方 `livox_ros_driver2` 恰好把两类回调放在同一个回调线程里，但 SDK2 没有承诺
 * 这一点；而 `container::SpscQueue` 只允许一个生产者。改前只要 SDK 真用两个线程
 * 回调，两个线程就会同时写 `head_`（丢包、错槽位，TSan 直接报 data race）。
 *
 * 这里用两个线程各推 2000 个可区分的包，要求全部入队、逐包intact——
 * 顺序不保证（生产者之间本来就没有顺序），但不许丢、不许串。
 */
TEST(CallbackSinkTest, TwoConcurrentProducersNeverCorruptOrLosePackets) {
    auto queue = std::make_shared<container::SpscQueue<PacketSlot>>(kQueueSlots);
    auto sink = std::make_shared<CallbackSink>();
    sink->handle.store(7U);
    sink->queue = queue;

    const std::array<std::uint8_t, 64> packet_a = make_packet(0xA5U);
    const std::array<std::uint8_t, 64> packet_b = make_packet(0x5AU);

    std::atomic<int> ready{0};
    const auto producer = [&](const std::array<std::uint8_t, 64>& payload, std::uint8_t tag) {
        PacketView view;
        view.handle = 7U;
        view.dev_type = tag;
        view.data = payload.data();
        view.size = static_cast<std::uint32_t>(payload.size());
        ready.fetch_add(1);
        while (ready.load() < 2) {
            std::this_thread::yield();
        }
        for (std::size_t index = 0; index < kPacketsPerProducer; ++index) {
            sink->push(view);
        }
    };

    std::thread imu_thread(producer, std::cref(packet_a), static_cast<std::uint8_t>(0x01U));
    std::thread point_thread(producer, std::cref(packet_b), static_cast<std::uint8_t>(0x02U));
    imu_thread.join();
    point_thread.join();

    EXPECT_EQ(sink->packets_dropped.load(), 0U);
    EXPECT_EQ(sink->packets_received.load(), 2U * kPacketsPerProducer);

    std::size_t drained = 0;
    std::size_t mismatched = 0;
    std::array<std::size_t, 2> per_device{0, 0};
    PacketSlot slot;
    while (queue->try_pop(slot)) {
        ++drained;
        const std::uint8_t expected =
            slot.dev_type == 0x01U ? 0xA5U : (slot.dev_type == 0x02U ? 0x5AU : 0xFFU);
        const bool intact =
            slot.length == 64U && slot.data[0] == expected && slot.data[63] == expected;
        if (!intact) {
            ++mismatched;
        }
        if (slot.dev_type == 0x01U) {
            ++per_device[0];
        } else if (slot.dev_type == 0x02U) {
            ++per_device[1];
        }
    }

    EXPECT_EQ(drained, 2U * kPacketsPerProducer);
    EXPECT_EQ(mismatched, 0U);
    EXPECT_EQ(per_device[0], kPacketsPerProducer);
    EXPECT_EQ(per_device[1], kPacketsPerProducer);
}

} // namespace foray::platform::driver::livox::detail
