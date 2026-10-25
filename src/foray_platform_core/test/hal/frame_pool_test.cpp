#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <thread>

#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/frame_lease.hpp"
#include "foray/platform/hal/frame_pool.hpp"

namespace foray::platform::hal {
namespace {

using ImagePool = FramePool<ImageFrame>;

TEST(FramePoolTest, PublishesAndAcquiresLatest) {
    auto pool = std::make_shared<ImagePool>(4, 1024);

    auto writer = pool->try_acquire_writer();
    ASSERT_TRUE(writer.has_value());
    EXPECT_EQ(writer->payload_size(), 1024U);

    std::uint8_t* payload = writer->data();
    std::memset(payload, 0x5A, 16);
    writer->frame().data = payload;
    writer->frame().size = 16;
    writer->frame().width = 640;
    writer->frame().height = 480;
    writer->frame().sequence = 1;
    writer->publish();

    EXPECT_EQ(pool->dropped(), 0U);

    FrameLease<ImageFrame> lease = pool->try_acquire_latest();
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->width, 640U);
    EXPECT_EQ(lease->height, 480U);
    EXPECT_EQ(lease->sequence, 1U);
    EXPECT_EQ(lease->data, payload);
    EXPECT_EQ(lease->data[0], 0x5A);

    // 同一帧不能被两个消费者同时租用。
    EXPECT_FALSE(pool->try_acquire_latest());
}

TEST(FramePoolTest, ReplacesOldestPublishedFrame) {
    auto pool = std::make_shared<ImagePool>(3, 128);

    auto first = pool->try_acquire_writer();
    ASSERT_TRUE(first.has_value());
    first->frame().sequence = 1;
    first->publish();

    auto second = pool->try_acquire_writer();
    ASSERT_TRUE(second.has_value());
    second->frame().sequence = 2;
    second->publish();

    EXPECT_EQ(pool->dropped(), 1U);

    FrameLease<ImageFrame> lease = pool->try_acquire_latest();
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->sequence, 2U);
}

TEST(FramePoolTest, UnpublishedWriterReturnsSlot) {
    auto pool = std::make_shared<ImagePool>(1, 64);

    {
        auto writer = pool->try_acquire_writer();
        ASSERT_TRUE(writer.has_value());
        // 不 publish，析构时归还。
    }

    auto writer = pool->try_acquire_writer();
    EXPECT_TRUE(writer.has_value());
}

TEST(FramePoolTest, ReportsNoFreeSlot) {
    auto pool = std::make_shared<ImagePool>(2, 64);

    auto first = pool->try_acquire_writer();
    auto second = pool->try_acquire_writer();
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());

    EXPECT_FALSE(pool->try_acquire_writer());
    EXPECT_EQ(pool->no_slot(), 1U);

    first.reset();
    second.reset();

    EXPECT_TRUE(pool->try_acquire_writer().has_value());
}

TEST(FramePoolTest, LeaseOutlivesPoolOwner) {
    FrameLease<ImageFrame> lease;

    {
        auto pool = std::make_shared<ImagePool>(2, 64);
        auto writer = pool->try_acquire_writer();
        ASSERT_TRUE(writer.has_value());
        writer->frame().sequence = 7;
        writer->publish();

        lease = pool->try_acquire_latest();
        ASSERT_TRUE(lease);
    }

    // FramePool 的 deleter 持有池的 shared_ptr，租约仍然有效。
    EXPECT_EQ(lease->sequence, 7U);
    lease.reset();
}

TEST(FramePoolTest, ReadsPublishedSequenceWithoutTearing) {
    auto pool = std::make_shared<ImagePool>(4, 64);

    for (std::uint64_t sequence = 1; sequence <= 100; ++sequence) {
        auto writer = pool->try_acquire_writer();
        ASSERT_TRUE(writer.has_value());
        writer->frame().sequence = sequence;
        writer->publish();

        FrameLease<ImageFrame> lease = pool->try_acquire_latest();
        ASSERT_TRUE(lease);
        EXPECT_EQ(lease->sequence, sequence);
    }
}

TEST(FramePoolTest, ConcurrentWriterAndReader) {
    auto pool = std::make_shared<ImagePool>(8, 64);
    constexpr std::uint64_t kFrames = 20000;

    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> last_seen{0};
    std::atomic<bool> out_of_order{false};

    std::thread reader([&pool, &stop, &last_seen, &out_of_order] {
        while (!stop.load(std::memory_order_acquire)) {
            FrameLease<ImageFrame> lease = pool->try_acquire_latest();
            if (!lease) {
                continue;
            }

            const std::uint64_t previous = last_seen.load(std::memory_order_relaxed);
            if (lease->sequence < previous) {
                out_of_order.store(true, std::memory_order_relaxed);
            }
            last_seen.store(lease->sequence, std::memory_order_relaxed);
        }
    });

    for (std::uint64_t sequence = 1; sequence <= kFrames; ++sequence) {
        std::optional<ImagePool::Writer> writer;
        while (!writer.has_value()) {
            writer = pool->try_acquire_writer();
            if (!writer.has_value()) {
                std::this_thread::yield();
            }
        }
        writer->frame().sequence = sequence;
        writer->publish();
    }

    stop.store(true, std::memory_order_release);
    reader.join();

    EXPECT_FALSE(out_of_order.load());
    EXPECT_GT(last_seen.load(), 0U);
}

#if defined(NDEBUG)
// Release：违反「池由 shared_ptr 持有」的契约时降级返回空租约，不得 terminate。
TEST(FramePoolTest, NonSharedOwnerReturnsEmptyLease) {
    ImagePool pool(2, 64); // 栈对象，未交给 shared_ptr

    EXPECT_FALSE(pool.try_acquire_latest());

    // 槽位状态未被错置：写入者仍能正常取槽并发布。
    auto writer = pool.try_acquire_writer();
    ASSERT_TRUE(writer.has_value());
    writer->frame().sequence = 5;
    writer->publish();
    EXPECT_EQ(pool.dropped(), 0U);
    EXPECT_EQ(pool.no_slot(), 0U);
}

TEST(FramePoolTest, NonSharedOwnerKeepsPublishedSlotReadable) {
    ImagePool pool(2, 64);

    auto published = pool.try_acquire_writer();
    ASSERT_TRUE(published.has_value());
    published->frame().sequence = 11;
    published->publish();

    EXPECT_FALSE(pool.try_acquire_latest());
    // 槽位仍在 kPublished：新帧替换旧帧时能回收旧槽并计数。
    auto replacement = pool.try_acquire_writer();
    ASSERT_TRUE(replacement.has_value());
    replacement->publish();
    EXPECT_EQ(pool.dropped(), 1U);
}
#else
// Debug：同一误用以断言中止，便于在开发期立即发现。
TEST(FramePoolDeathTest, NonSharedOwnerTripsAssertion) {
    ImagePool pool(2, 64);
    EXPECT_DEATH((void)pool.try_acquire_latest(), "shared_ptr");
}
#endif

} // namespace
} // namespace foray::platform::hal
