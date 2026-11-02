#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace foray::platform::container {

/**
 * @brief 有界单生产者/单消费者队列。
 *
 * 约束：
 * - `T` 必须可默认构造，且移动构造/移动赋值不抛异常；
 * - 容量在构造时确定并向上取 2 的幂；
 * - `try_push` / `try_pop` 不分配内存、不加锁；
 * - 允许一个生产者线程和一个消费者线程并发调用；
 * - 队列满时 `try_push` 返回 false，并累计 `dropped()`（失败尝试次数；
 *   调用方可以重试，因此它不一定等于真实丢失的数据量）。
 *
 * 典型用途：SDK 回调/取流线程 → 解析/消费线程的帧或包传递。
 */
template <typename T> class SpscQueue {
  public:
    static_assert(std::is_default_constructible_v<T>,
                  "SpscQueue<T> requires a default-constructible T");
    static_assert(std::is_nothrow_move_constructible_v<T>,
                  "SpscQueue<T> requires nothrow move construction");
    static_assert(std::is_nothrow_move_assignable_v<T>,
                  "SpscQueue<T> requires nothrow move assignment");

    /**
     * @brief 构造队列。
     * @param capacity 请求容量；向上取 2 的幂，最小为 1。
     * @throws std::invalid_argument 容量为 0。
     * @throws std::length_error 容量过大，无法向上取 2 的幂。
     */
    explicit SpscQueue(std::size_t capacity)
        : capacity_(round_up_power_of_two(capacity)), buffer_(std::make_unique<T[]>(capacity_)) {
    }

    SpscQueue(const SpscQueue&) = delete;
    SpscQueue& operator=(const SpscQueue&) = delete;

    /// @brief 入队（拷贝）；满时丢弃并返回 false。
    bool try_push(const T& value) noexcept {
        static_assert(std::is_nothrow_copy_assignable_v<T>,
                      "try_push(const T&) requires nothrow copy assignment");
        return push_impl(value);
    }

    /// @brief 入队（移动）；满时丢弃并返回 false。
    bool try_push(T&& value) noexcept {
        return push_impl(std::move(value));
    }

    /// @brief 出队；空时返回 false。
    bool try_pop(T& out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }

        out = std::move(buffer_[tail & mask()]);
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    /// @brief 近似元素个数（并发场景下仅供诊断）。
    std::size_t size_approx() const noexcept {
        const std::size_t head = head_.load(std::memory_order_acquire);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        return head - tail;
    }

    /// @brief 实际容量（2 的幂）。
    std::size_t capacity() const noexcept {
        return capacity_;
    }

    /// @brief 因队列满而丢弃的元素总数。
    std::uint64_t dropped() const noexcept {
        return dropped_.load(std::memory_order_relaxed);
    }

  private:
    static std::size_t round_up_power_of_two(std::size_t value) {
        if (value == 0) {
            throw std::invalid_argument("SpscQueue capacity must be greater than zero");
        }

        std::size_t power = 1;
        while (power < value) {
            if (power > (std::size_t{1} << (sizeof(std::size_t) * 8 - 2))) {
                throw std::length_error("SpscQueue capacity is too large");
            }
            power <<= 1;
        }
        return power;
    }

    std::size_t mask() const noexcept {
        return capacity_ - 1;
    }

    template <typename U> bool push_impl(U&& value) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);

        if (head - tail >= capacity_) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        buffer_[head & mask()] = std::forward<U>(value);
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    std::size_t capacity_;
    std::unique_ptr<T[]> buffer_;

    // 读写索引分处不同缓存行，避免伪共享。
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
    alignas(64) std::atomic<std::uint64_t> dropped_{0};
};

} // namespace foray::platform::container
