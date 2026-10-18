#pragma once

#include <atomic>
#include <cstdint>

#include "foray/platform/time/timestamp.hpp"

namespace foray::platform::time {

/**
 * @brief 时钟域。
 *
 * 时间点只在同一域内有意义。域之间不允许比较或相减。
 */
enum class ClockDomain {
    /// @brief 系统墙钟（Unix 纪元）。会因 NTP / 手动校时而跳变。
    kSystemWall,
    /// @brief 单调时钟。纪元未指定，只保证同一进程内不倒退。
    kSteady,
    /// @brief 仿真/外部驱动时钟。由 `/clock` 等外部源推进，可能跳变。
    kSimulated,
};

/**
 * @brief 返回时钟域名称，用于日志与诊断。
 */
constexpr const char* to_string(ClockDomain domain) noexcept {
    switch (domain) {
    case ClockDomain::kSystemWall:
        return "system_wall";
    case ClockDomain::kSteady:
        return "steady";
    case ClockDomain::kSimulated:
        return "simulated";
    }
    return "unknown";
}

/**
 * @brief 时钟抽象。
 *
 * 使用约定：
 * - **同一个模块内只使用一个时钟域**，禁止把墙钟时间戳与仿真时间戳混用；
 * - `is_steady() == false` 的时钟可能倒退（NTP 校时、仿真时间回退），
 *   因此不能用于计算超时的「截止时刻」以外的假设；
 * - `is_ready() == false` 表示时间尚未可用（典型场景：`use_sim_time=true`
 *   但尚未收到第一帧 `/clock`）。此时 `now()` 返回零，调用方必须显式处理，
 *   **不得把零时刻当成真实时间参与超时判定**；
 * - 时间源的选择发生在启动期，运行期不切换域。
 *
 * 线程安全：实现必须保证 `now()`、`domain()`、`is_steady()`、`is_ready()`
 * 可被多个线程并发调用。
 */
class Clock {
  public:
    virtual ~Clock() = default;

    /// @brief 返回当前时间点。未就绪时返回零时间点。
    virtual Timestamp now() const = 0;

    /// @brief 返回所属时钟域。
    virtual ClockDomain domain() const noexcept = 0;

    /// @brief 是否单调（只保证不倒退，不保证速率）。
    virtual bool is_steady() const noexcept = 0;

    /**
     * @brief 时间是否已可用。
     *
     * 真实时钟恒为 `true`；仿真时钟在收到第一帧外部时间后变为 `true`。
     */
    virtual bool is_ready() const noexcept {
        return true;
    }
};

/**
 * @brief 系统墙钟。
 *
 * 域为 `kSystemWall`，非单调：NTP 校时或手动改时会导致时间跳跃。
 * 用于给消息打时间戳、与外部系统对时，不用于计时或超时。
 */
class SystemClock final : public Clock {
  public:
    Timestamp now() const override;

    ClockDomain domain() const noexcept override {
        return ClockDomain::kSystemWall;
    }

    bool is_steady() const noexcept override {
        return false;
    }
};

/**
 * @brief 单调时钟。
 *
 * 域为 `kSteady`，纪元未指定，只保证同一进程内不倒退。
 * 用于测量间隔、实现实时循环周期与本地看门狗。
 */
class SteadyClock final : public Clock {
  public:
    Timestamp now() const override;

    ClockDomain domain() const noexcept override {
        return ClockDomain::kSteady;
    }

    bool is_steady() const noexcept override {
        return true;
    }
};

/**
 * @brief 手动推进的时钟，用于单元测试与仿真。
 *
 * 语义与 ROS 2 的 `use_sim_time=true` 对齐：
 * - 默认构造后 `is_ready() == false`，`now()` 为零，模拟「尚未收到 `/clock`」；
 * - 通过 `set()` 或 `advance()` 提供第一个时间后变为就绪；
 * - 非单调：`set()` 可以让时间回退。
 *
 * ROS 2 适配层应在 `/clock` 回调中调用 `set()`，不要自行累加。
 * 线程安全：所有操作均为原子操作。
 */
class ManualClock final : public Clock {
  public:
    /// @brief 未就绪的仿真时钟，时间为零。
    ManualClock() noexcept = default;

    /**
     * @brief 以给定时间构造，并立即就绪。
     * @param start 初始时间点。
     */
    explicit ManualClock(Timestamp start) noexcept;

    Timestamp now() const override;

    ClockDomain domain() const noexcept override {
        return ClockDomain::kSimulated;
    }

    bool is_steady() const noexcept override {
        // 仿真时间允许回退（例如回放时跳转），因此不是单调时钟。
        return false;
    }

    bool is_ready() const noexcept override;

    /**
     * @brief 设置当前时间并标记为就绪。
     *
     * 可回退，模拟 `/clock` 的跳变。
     */
    void set(Timestamp timestamp) noexcept;

    /**
     * @brief 前进给定时长并标记为就绪。
     *
     * 时长可为负（模拟时间回退）。
     */
    void advance(Duration duration) noexcept;

  private:
    std::atomic<std::int64_t> nanoseconds_{0};
    std::atomic<bool> ready_{false};
};

} // namespace foray::platform::time
