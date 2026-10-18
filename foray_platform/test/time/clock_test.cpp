#include <gtest/gtest.h>

#include "foray/platform/time/time.hpp"

namespace foray::platform::time {
namespace {

TEST(DurationTest, DefaultIsZero) {
    const Duration duration;
    EXPECT_EQ(duration.nanoseconds(), 0);
    EXPECT_DOUBLE_EQ(duration.seconds(), 0.0);
}

TEST(DurationTest, Constructors) {
    EXPECT_EQ(Duration::from_nanoseconds(1'500).nanoseconds(), 1'500);
    EXPECT_EQ(Duration::from_milliseconds(2).nanoseconds(), 2'000'000);
    EXPECT_NEAR(Duration::from_seconds(1.5).seconds(), 1.5, 1e-12);
}

TEST(DurationTest, ArithmeticAndComparison) {
    const Duration a = Duration::from_milliseconds(10);
    const Duration b = Duration::from_milliseconds(4);

    EXPECT_EQ((a + b).nanoseconds(), 14'000'000);
    EXPECT_EQ((a - b).nanoseconds(), 6'000'000);
    EXPECT_EQ((-a).nanoseconds(), -10'000'000);

    EXPECT_TRUE(b < a);
    EXPECT_TRUE(a > b);
    EXPECT_TRUE(a >= b);
    EXPECT_TRUE(b <= a);
    EXPECT_TRUE(a != b);
    EXPECT_TRUE(a == Duration::from_milliseconds(10));
}

TEST(TimestampTest, Arithmetic) {
    const Timestamp start = Timestamp::from_seconds(10.0);
    const Timestamp later = start + Duration::from_milliseconds(250);

    EXPECT_EQ(later.nanoseconds_since_epoch(), 10'250'000'000);
    EXPECT_EQ((later - start).nanoseconds(), 250'000'000);
    EXPECT_EQ((later - Duration::from_milliseconds(250)), start);
    EXPECT_TRUE(start < later);
    EXPECT_FALSE(start.is_zero());

    EXPECT_TRUE(Timestamp().is_zero());
}

TEST(SystemClockTest, ReportsWallClockDomain) {
    const SystemClock clock;
    EXPECT_EQ(clock.domain(), ClockDomain::kSystemWall);
    EXPECT_FALSE(clock.is_steady());
    EXPECT_TRUE(clock.is_ready());
    EXPECT_NE(clock.now().nanoseconds_since_epoch(), 0);
}

TEST(SteadyClockTest, NeverGoesBackwards) {
    const SteadyClock clock;
    const Timestamp first = clock.now();
    const Timestamp second = clock.now();

    EXPECT_EQ(clock.domain(), ClockDomain::kSteady);
    EXPECT_TRUE(clock.is_steady());
    EXPECT_TRUE(clock.is_ready());
    EXPECT_GE(second, first);
}

TEST(ManualClockTest, IsNotReadyUntilTimeIsProvided) {
    const ManualClock clock;

    EXPECT_EQ(clock.domain(), ClockDomain::kSimulated);
    EXPECT_FALSE(clock.is_steady());
    EXPECT_FALSE(clock.is_ready());
    EXPECT_TRUE(clock.now().is_zero());
}

TEST(ManualClockTest, SetMakesClockReady) {
    ManualClock clock;
    clock.set(Timestamp::from_nanoseconds(42));

    EXPECT_TRUE(clock.is_ready());
    EXPECT_EQ(clock.now().nanoseconds_since_epoch(), 42);
}

TEST(ManualClockTest, AdvanceAndRewind) {
    ManualClock clock(Timestamp::from_milliseconds(100));

    clock.advance(Duration::from_milliseconds(20));
    EXPECT_EQ(clock.now().nanoseconds_since_epoch(), 120'000'000);

    // 仿真时间允许回退（例如回放跳转）。
    clock.advance(Duration::from_milliseconds(-50));
    EXPECT_EQ(clock.now().nanoseconds_since_epoch(), 70'000'000);

    clock.set(Timestamp::from_milliseconds(10));
    EXPECT_EQ(clock.now().nanoseconds_since_epoch(), 10'000'000);
}

TEST(ClockDomainTest, Names) {
    EXPECT_STREQ(to_string(ClockDomain::kSystemWall), "system_wall");
    EXPECT_STREQ(to_string(ClockDomain::kSteady), "steady");
    EXPECT_STREQ(to_string(ClockDomain::kSimulated), "simulated");
}

} // namespace
} // namespace foray::platform::time
