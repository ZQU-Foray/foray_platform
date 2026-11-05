#pragma once

#include <cstdint>

namespace foray::platform::time {

/**
 * @brief 纳秒精度的有符号时间跨度。
 *
 * 用于表达超时、周期、时间差等。取值范围为 `int64_t` 纳秒（约 ±292 年）。
 * 加减法不做溢出检查，调用方应保证结果落在可表示范围内。
 */
class Duration {
  public:
    /// @brief 零时长。
    constexpr Duration() noexcept = default;

    /**
     * @brief 由纳秒构造。
     * @param nanoseconds 纳秒数，可为负。
     */
    static constexpr Duration from_nanoseconds(std::int64_t nanoseconds) noexcept {
        return Duration(nanoseconds);
    }

    /**
     * @brief 由毫秒构造。
     * @param milliseconds 毫秒数，可为负。
     */
    static constexpr Duration from_milliseconds(std::int64_t milliseconds) noexcept {
        return Duration(milliseconds * 1'000'000);
    }

    /**
     * @brief 由秒构造（浮点）。
     * @param seconds 秒数，可为负。
     *
     * 仅用于配置与测试；精度受 `double` 限制，不要用于累计计时。
     */
    static constexpr Duration from_seconds(double seconds) noexcept {
        return Duration(static_cast<std::int64_t>(seconds * 1'000'000'000.0));
    }

    /// @brief 返回纳秒数。
    constexpr std::int64_t nanoseconds() const noexcept {
        return nanoseconds_;
    }

    /// @brief 返回秒数（浮点）。
    constexpr double seconds() const noexcept {
        return static_cast<double>(nanoseconds_) / 1'000'000'000.0;
    }

    /// @brief 取负。
    constexpr Duration operator-() const noexcept {
        return Duration(-nanoseconds_);
    }

    constexpr Duration& operator+=(Duration other) noexcept {
        nanoseconds_ += other.nanoseconds_;
        return *this;
    }

    constexpr Duration& operator-=(Duration other) noexcept {
        nanoseconds_ -= other.nanoseconds_;
        return *this;
    }

  private:
    explicit constexpr Duration(std::int64_t nanoseconds) noexcept : nanoseconds_(nanoseconds) {
    }

    std::int64_t nanoseconds_ = 0;
};

constexpr Duration operator+(Duration lhs, Duration rhs) noexcept {
    lhs += rhs;
    return lhs;
}

constexpr Duration operator-(Duration lhs, Duration rhs) noexcept {
    lhs -= rhs;
    return lhs;
}

constexpr bool operator==(Duration lhs, Duration rhs) noexcept {
    return lhs.nanoseconds() == rhs.nanoseconds();
}

constexpr bool operator!=(Duration lhs, Duration rhs) noexcept {
    return !(lhs == rhs);
}

constexpr bool operator<(Duration lhs, Duration rhs) noexcept {
    return lhs.nanoseconds() < rhs.nanoseconds();
}

constexpr bool operator<=(Duration lhs, Duration rhs) noexcept {
    return !(rhs < lhs);
}

constexpr bool operator>(Duration lhs, Duration rhs) noexcept {
    return rhs < lhs;
}

constexpr bool operator>=(Duration lhs, Duration rhs) noexcept {
    return !(lhs < rhs);
}

} // namespace foray::platform::time
