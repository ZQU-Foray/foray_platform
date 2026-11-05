#pragma once

#include <memory>
#include <utility>

namespace foray::platform::hal {

/**
 * @brief 帧的 RAII 租约。
 *
 * - 持有指向帧池槽位的 `shared_ptr`；最后一个租约析构时归还槽位；
 * - move-only：同一帧在同一时刻只由一个消费者持有；
 * - `FrameLease` 可以晚于设备销毁——帧池由 deleter 中的 `shared_ptr` 保活。
 *
 * 空租约的 `operator*` / `operator->` 是未定义行为；使用前检查布尔值。
 */
template <typename T> class FrameLease {
  public:
    FrameLease() noexcept = default;

    explicit FrameLease(std::shared_ptr<const T> frame) noexcept : frame_(std::move(frame)) {
    }

    FrameLease(FrameLease&&) noexcept = default;
    FrameLease& operator=(FrameLease&&) noexcept = default;

    FrameLease(const FrameLease&) = delete;
    FrameLease& operator=(const FrameLease&) = delete;

    ~FrameLease() = default;

    const T& operator*() const noexcept {
        return *frame_;
    }

    const T* operator->() const noexcept {
        return frame_.get();
    }

    const T* get() const noexcept {
        return frame_.get();
    }

    explicit operator bool() const noexcept {
        return static_cast<bool>(frame_);
    }

    void reset() noexcept {
        frame_.reset();
    }

  private:
    std::shared_ptr<const T> frame_;
};

} // namespace foray::platform::hal
