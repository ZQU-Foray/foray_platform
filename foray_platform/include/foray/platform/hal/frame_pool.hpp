#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "foray/platform/hal/frame_lease.hpp"

namespace foray::platform::hal {

/**
 * @brief 固定容量帧池，带最新帧邮箱。
 *
 * 线程模型：
 * - **一个写入者**（取流/解析线程）调用 `try_acquire_writer()`；
 * - **一个读取者**（消费者线程）调用 `try_acquire_latest()`；
 * - 读写路径均不加锁、不分配内存；
 * - 消费者慢时，新发布帧会替换旧的最新帧并累计 `dropped()`；
 * - 槽位耗尽时 `try_acquire_writer()` 返回空并累计 `no_slot()`。
 *
 * 生命周期：
 * - 必须通过 `std::make_shared<FramePool<T>>(...)` 创建；
 * - `FrameLease` 的 deleter 持有帧池的 `shared_ptr`，因此租约可以晚于
 *   设备/帧池所有者销毁。
 */
template <typename T> class FramePool : public std::enable_shared_from_this<FramePool<T>> {
  public:
    /**
     * @brief 写入者句柄。
     *
     * 析构时若未 `publish()`，槽位自动归还。
     */
    class Writer {
      public:
        Writer(FramePool* pool, std::size_t index) noexcept : pool_(pool), index_(index) {
        }

        ~Writer() {
            release_if_unpublished();
        }

        Writer(Writer&& other) noexcept
            : pool_(other.pool_), index_(other.index_), published_(other.published_) {
            other.pool_ = nullptr;
            other.published_ = true;
        }

        Writer& operator=(Writer&& other) noexcept {
            if (this != &other) {
                release_if_unpublished();
                pool_ = other.pool_;
                index_ = other.index_;
                published_ = other.published_;
                other.pool_ = nullptr;
                other.published_ = true;
            }
            return *this;
        }

        Writer(const Writer&) = delete;
        Writer& operator=(const Writer&) = delete;

        /// @brief 可写的帧元数据。
        T& frame() noexcept {
            return pool_->slots_[index_].frame;
        }

        /// @brief 可写的原始载荷缓冲。
        std::uint8_t* data() noexcept {
            return pool_->slots_[index_].payload.get();
        }

        /// @brief 载荷缓冲字节数。
        std::size_t payload_size() const noexcept {
            return pool_->payload_size_;
        }

        /// @brief 发布到最新帧邮箱；重复调用无效果。
        void publish() noexcept {
            if (pool_ == nullptr || published_) {
                return;
            }
            pool_->publish(index_);
            published_ = true;
        }

      private:
        void release_if_unpublished() noexcept {
            if (pool_ != nullptr && !published_) {
                pool_->release_writer(index_);
            }
            pool_ = nullptr;
        }

        FramePool* pool_ = nullptr;
        std::size_t index_ = 0;
        bool published_ = false;
    };

    FramePool(std::size_t slot_count, std::size_t payload_size)
        : slot_count_(slot_count), payload_size_(payload_size),
          slots_(std::make_unique<Slot[]>(slot_count)) {
        for (std::size_t index = 0; index < slot_count_; ++index) {
            slots_[index].state.store(State::kFree, std::memory_order_relaxed);
            slots_[index].payload = std::make_unique<std::uint8_t[]>(payload_size_);
        }
    }

    FramePool(const FramePool&) = delete;
    FramePool& operator=(const FramePool&) = delete;

    /// @brief 获取一个写入者；无空闲槽时返回空。
    std::optional<Writer> try_acquire_writer() noexcept {
        for (std::size_t index = 0; index < slot_count_; ++index) {
            State expected = State::kFree;
            if (slots_[index].state.compare_exchange_strong(expected, State::kWriting,
                                                            std::memory_order_acq_rel)) {
                slots_[index].frame = T{};
                return Writer(this, index);
            }
        }

        no_slot_.fetch_add(1, std::memory_order_relaxed);
        return std::nullopt;
    }

    /// @brief 获取最新已发布帧；没有可读帧时返回空租约。
    FrameLease<T> try_acquire_latest() noexcept {
        const std::size_t index = latest_.load(std::memory_order_acquire);
        if (index == kNone) {
            return FrameLease<T>();
        }

        Slot& slot = slots_[index];
        State expected = State::kPublished;
        if (!slot.state.compare_exchange_strong(expected, State::kReading,
                                                std::memory_order_acq_rel)) {
            return FrameLease<T>();
        }

        std::shared_ptr<FramePool> pool = this->shared_from_this();
        std::shared_ptr<const T> frame(&slot.frame, [pool = std::move(pool), index](const T*) {
            pool->slots_[index].state.store(State::kFree, std::memory_order_release);
        });
        return FrameLease<T>(std::move(frame));
    }

    std::size_t slot_count() const noexcept {
        return slot_count_;
    }

    std::size_t payload_size() const noexcept {
        return payload_size_;
    }

    /// @brief 被新帧替换而丢弃的已发布帧数。
    std::uint64_t dropped() const noexcept {
        return dropped_.load(std::memory_order_relaxed);
    }

    /// @brief 因无空闲槽而失败的写入请求数。
    std::uint64_t no_slot() const noexcept {
        return no_slot_.load(std::memory_order_relaxed);
    }

  private:
    enum class State : std::uint8_t {
        kFree,
        kWriting,
        kPublished,
        kReading,
    };

    struct Slot {
        std::atomic<State> state{State::kFree};
        T frame{};
        std::unique_ptr<std::uint8_t[]> payload;
    };

    static constexpr std::size_t kNone = static_cast<std::size_t>(-1);

    void release_writer(std::size_t index) noexcept {
        slots_[index].state.store(State::kFree, std::memory_order_release);
    }

    void publish(std::size_t index) noexcept {
        slots_[index].state.store(State::kPublished, std::memory_order_release);

        const std::size_t previous = latest_.exchange(index, std::memory_order_acq_rel);
        if (previous == kNone || previous == index) {
            return;
        }

        State expected = State::kPublished;
        if (slots_[previous].state.compare_exchange_strong(expected, State::kFree,
                                                           std::memory_order_acq_rel)) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        // previous 处于 kReading 时由租约释放；处于其他状态说明消费者正在
        // 竞争读取，CAS 失败是安全的。
    }

    std::size_t slot_count_;
    std::size_t payload_size_;
    std::unique_ptr<Slot[]> slots_;

    alignas(64) std::atomic<std::size_t> latest_{kNone};
    alignas(64) std::atomic<std::uint64_t> dropped_{0};
    alignas(64) std::atomic<std::uint64_t> no_slot_{0};
};

} // namespace foray::platform::hal
