#pragma once

#include <cstdint>

#include "foray/platform/time/duration.hpp"

namespace foray::platform::time {

/**
 * @brief 时间点，纳秒精度。
 *
 * 时间点的「纪元」由所属时钟域决定：
 * - `ClockDomain::kSystemWall`：Unix 纪元（1970-01-01T00:00:00Z）；
 * - `ClockDomain::kSteady`：`std::chrono::steady_clock` 的未指定纪元，仅用于同一进程内的差值；
 * - `ClockDomain::kSimulated`：仿真时间纪元，由 `/clock` 发布者定义。
 *
 * **不同时钟域的时间点不得相互比较或相减。** 该约束由调用方保证，
 * 本类型不携带域标签（避免每次运算的运行时开销）。
 */
class Timestamp {
  public:
    /// @brief 零时间点（纪元）。
    constexpr Timestamp() noexcept = default;

    /// @brief 由纳秒构造。
    static constexpr Timestamp from_nanoseconds(std::int64_t nanoseconds) noexcept {
        return Timestamp(nanoseconds);
    }

    /// @brief 由毫秒构造。
    static constexpr Timestamp from_milliseconds(std::int64_t milliseconds) noexcept {
        return Timestamp(milliseconds * 1'000'000);
    }

    /// @brief 由秒构造（浮点）。
    static constexpr Timestamp from_seconds(double seconds) noexcept {
        return Timestamp(static_cast<std::int64_t>(seconds * 1'000'000'000.0));
    }

    /// @brief 返回自纪元起的纳秒数。
    constexpr std::int64_t nanoseconds_since_epoch() const noexcept {
        return nanoseconds_;
    }

    /// @brief 返回自纪元起的秒数（浮点）。
    constexpr double seconds_since_epoch() const noexcept {
        return static_cast<double>(nanoseconds_) / 1'000'000'000.0;
    }

    /// @brief 是否恰为纪元（零）。
    constexpr bool is_zero() const noexcept {
        return nanoseconds_ == 0;
    }

    constexpr Timestamp& operator+=(Duration duration) noexcept {
        nanoseconds_ += duration.nanoseconds();
        return *this;
    }

    constexpr Timestamp& operator-=(Duration duration) noexcept {
        nanoseconds_ -= duration.nanoseconds();
        return *this;
    }

  private:
    explicit constexpr Timestamp(std::int64_t nanoseconds) noexcept : nanoseconds_(nanoseconds) {
    }

    std::int64_t nanoseconds_ = 0;
};

/// @brief 两个时间点之差。
constexpr Duration operator-(Timestamp lhs, Timestamp rhs) noexcept {
    return Duration::from_nanoseconds(lhs.nanoseconds_since_epoch() -
                                      rhs.nanoseconds_since_epoch());
}

constexpr Timestamp operator+(Timestamp timestamp, Duration duration) noexcept {
    timestamp += duration;
    return timestamp;
}

constexpr Timestamp operator+(Duration duration, Timestamp timestamp) noexcept {
    return timestamp + duration;
}

constexpr Timestamp operator-(Timestamp timestamp, Duration duration) noexcept {
    timestamp -= duration;
    return timestamp;
}

constexpr bool operator==(Timestamp lhs, Timestamp rhs) noexcept {
    return lhs.nanoseconds_since_epoch() == rhs.nanoseconds_since_epoch();
}

constexpr bool operator!=(Timestamp lhs, Timestamp rhs) noexcept {
    return !(lhs == rhs);
}

constexpr bool operator<(Timestamp lhs, Timestamp rhs) noexcept {
    return lhs.nanoseconds_since_epoch() < rhs.nanoseconds_since_epoch();
}

constexpr bool operator<=(Timestamp lhs, Timestamp rhs) noexcept {
    return !(rhs < lhs);
}

constexpr bool operator>(Timestamp lhs, Timestamp rhs) noexcept {
    return rhs < lhs;
}

constexpr bool operator>=(Timestamp lhs, Timestamp rhs) noexcept {
    return !(lhs < rhs);
}

} // namespace foray::platform::time
