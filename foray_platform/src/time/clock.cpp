#include "foray/platform/time/clock.hpp"

#include <chrono>

namespace foray::platform::time {

namespace {

std::int64_t to_nanoseconds(std::chrono::nanoseconds duration) noexcept {
    return duration.count();
}

} // namespace

Timestamp SystemClock::now() const {
    const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
    return Timestamp::from_nanoseconds(
        to_nanoseconds(std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch)));
}

Timestamp SteadyClock::now() const {
    const auto since_epoch = std::chrono::steady_clock::now().time_since_epoch();
    return Timestamp::from_nanoseconds(
        to_nanoseconds(std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch)));
}

ManualClock::ManualClock(Timestamp start) noexcept
    : nanoseconds_(start.nanoseconds_since_epoch()), ready_(true) {
}

Timestamp ManualClock::now() const {
    return Timestamp::from_nanoseconds(nanoseconds_.load(std::memory_order_acquire));
}

bool ManualClock::is_ready() const noexcept {
    return ready_.load(std::memory_order_acquire);
}

void ManualClock::set(Timestamp timestamp) noexcept {
    // 先写入时间，再发布就绪标志：读者看到 ready==true 时一定能看到该时间。
    nanoseconds_.store(timestamp.nanoseconds_since_epoch(), std::memory_order_release);
    ready_.store(true, std::memory_order_release);
}

void ManualClock::advance(Duration duration) noexcept {
    nanoseconds_.fetch_add(duration.nanoseconds(), std::memory_order_acq_rel);
    ready_.store(true, std::memory_order_release);
}

} // namespace foray::platform::time
