#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>

#include "foray/platform/container/spsc_queue.hpp"

namespace foray::platform::container {
namespace {

TEST(SpscQueueTest, RoundsCapacityUpToPowerOfTwo) {
    const SpscQueue<int> queue(3);

    EXPECT_EQ(queue.capacity(), 4U);
    EXPECT_EQ(queue.size_approx(), 0U);
}

TEST(SpscQueueTest, RejectsZeroCapacity) {
    EXPECT_THROW((SpscQueue<int>(0)), std::invalid_argument);
}

TEST(SpscQueueTest, PopsInFifoOrder) {
    SpscQueue<int> queue(4);

    ASSERT_TRUE(queue.try_push(1));
    ASSERT_TRUE(queue.try_push(2));
    ASSERT_TRUE(queue.try_push(3));
    EXPECT_EQ(queue.size_approx(), 3U);

    int value = 0;
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, 1);
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, 2);
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, 3);
    EXPECT_FALSE(queue.try_pop(value));
}

TEST(SpscQueueTest, DropsWhenFull) {
    SpscQueue<int> queue(2);

    ASSERT_TRUE(queue.try_push(1));
    ASSERT_TRUE(queue.try_push(2));
    EXPECT_FALSE(queue.try_push(3));
    EXPECT_EQ(queue.dropped(), 1U);

    int value = 0;
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, 1);
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, 2);
    EXPECT_FALSE(queue.try_pop(value));
}

TEST(SpscQueueTest, WrapsAround) {
    SpscQueue<int> queue(4);

    for (int i = 0; i < 100; ++i) {
        ASSERT_TRUE(queue.try_push(i));
        int value = -1;
        ASSERT_TRUE(queue.try_pop(value));
        EXPECT_EQ(value, i);
    }

    EXPECT_EQ(queue.size_approx(), 0U);
    EXPECT_EQ(queue.dropped(), 0U);
}

TEST(SpscQueueTest, SupportsMoveOnlyValues) {
    SpscQueue<std::unique_ptr<int>> queue(2);

    ASSERT_TRUE(queue.try_push(std::make_unique<int>(7)));

    std::unique_ptr<int> value;
    ASSERT_TRUE(queue.try_pop(value));
    ASSERT_NE(value, nullptr);
    EXPECT_EQ(*value, 7);
}

TEST(SpscQueueTest, ConcurrentProducerConsumer) {
    constexpr int kItemCount = 100'000;
    SpscQueue<int> queue(1024);

    std::atomic<std::uint64_t> consumed_sum{0};
    std::atomic<int> consumed_count{0};

    std::thread consumer([&queue, &consumed_sum, &consumed_count] {
        int value = 0;
        while (consumed_count.load(std::memory_order_acquire) < kItemCount) {
            if (queue.try_pop(value)) {
                consumed_sum.fetch_add(static_cast<std::uint64_t>(value),
                                       std::memory_order_relaxed);
                consumed_count.fetch_add(1, std::memory_order_release);
            }
        }
    });

    for (int i = 0; i < kItemCount; ++i) {
        while (!queue.try_push(i)) {
            std::this_thread::yield();
        }
    }

    consumer.join();

    const std::uint64_t expected =
        static_cast<std::uint64_t>(kItemCount - 1) * static_cast<std::uint64_t>(kItemCount) / 2;
    EXPECT_EQ(consumed_count.load(), kItemCount);
    EXPECT_EQ(consumed_sum.load(), expected);
}

} // namespace
} // namespace foray::platform::container
