#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "foray/platform/param/store.hpp"

namespace foray::platform::param {
namespace {

ParamSpec make_spec(std::string name, ParamType type, ParamValue default_value) {
    ParamSpec spec;
    spec.name = std::move(name);
    spec.type = type;
    spec.default_value = std::move(default_value);
    return spec;
}

ParamSchema make_schema() {
    ParamSchema schema;

    ParamSpec speed = make_spec("chassis.max_speed", ParamType::kDouble, ParamValue(3.0));
    speed.minimum = ParamValue(0.0);
    speed.maximum = ParamValue(10.0);
    EXPECT_TRUE(schema.add(std::move(speed)).ok());

    ParamSpec shots = make_spec("shooter.burst", ParamType::kInt, ParamValue(3));
    shots.minimum = ParamValue(1);
    shots.maximum = ParamValue(10);
    EXPECT_TRUE(schema.add(std::move(shots)).ok());

    // 与 burst 始终在同一批更新中写入，用于验证快照的原子性。
    ParamSpec magazine = make_spec("shooter.magazine", ParamType::kInt, ParamValue(3));
    magazine.minimum = ParamValue(1);
    magazine.maximum = ParamValue(10);
    EXPECT_TRUE(schema.add(std::move(magazine)).ok());

    ParamSpec enabled = make_spec("chassis.enabled", ParamType::kBool, ParamValue(true));
    EXPECT_TRUE(schema.add(std::move(enabled)).ok());

    ParamSpec mode = make_spec("mode", ParamType::kString, ParamValue("idle"));
    mode.allowed_values = {ParamValue("idle"), ParamValue("run")};
    EXPECT_TRUE(schema.add(std::move(mode)).ok());

    ParamSpec serial_port =
        make_spec("serial.port", ParamType::kString, ParamValue("/dev/ttyUSB0"));
    serial_port.hot_reloadable = false;
    EXPECT_TRUE(schema.add(std::move(serial_port)).ok());

    return schema;
}

TEST(ParamStoreTest, InitializesSnapshotWithDefaults) {
    const ParamStore store(make_schema());
    const auto snapshot = store.snapshot();

    ASSERT_NE(snapshot, nullptr);
    EXPECT_EQ(snapshot->generation(), 0U);
    EXPECT_DOUBLE_EQ(snapshot->get_double("chassis.max_speed", -1.0), 3.0);
    EXPECT_EQ(snapshot->get_int("shooter.burst", -1), 3);
    EXPECT_TRUE(snapshot->get_bool("chassis.enabled", false));
    EXPECT_EQ(snapshot->get_string("mode", ""), "idle");
    EXPECT_EQ(snapshot->values().size(), 6U);
}

TEST(ParamStoreTest, AppliesValidUpdates) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("chassis.max_speed", ParamValue(4.5));
    updates.emplace("shooter.burst", ParamValue(5));
    updates.emplace("mode", ParamValue("run"));

    ASSERT_TRUE(store.apply(updates, ParamUpdateOrigin::kHotReload).ok());

    const auto snapshot = store.snapshot();
    EXPECT_EQ(snapshot->generation(), 1U);
    EXPECT_DOUBLE_EQ(snapshot->get_double("chassis.max_speed", 0.0), 4.5);
    EXPECT_EQ(snapshot->get_int("shooter.burst", 0), 5);
    EXPECT_EQ(snapshot->get_string("mode", ""), "run");
}

TEST(ParamStoreTest, WidensIntToDouble) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("chassis.max_speed", ParamValue(2));
    ASSERT_TRUE(store.apply(updates, ParamUpdateOrigin::kHotReload).ok());

    const ParamValue* value = store.snapshot()->find("chassis.max_speed");
    ASSERT_NE(value, nullptr);
    EXPECT_EQ(value->type(), ParamType::kDouble);
    EXPECT_DOUBLE_EQ(value->as_double(), 2.0);
}

TEST(ParamStoreTest, RejectsUnknownParameter) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("chassis.typo", ParamValue(1));
    const ParamResult result = store.apply(updates, ParamUpdateOrigin::kHotReload);

    ASSERT_FALSE(result.ok());
    ASSERT_EQ(result.errors().size(), 1U);
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kUnknownParam);
}

TEST(ParamStoreTest, RejectsTypeMismatch) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("shooter.burst", ParamValue("many"));
    const ParamResult result = store.apply(updates, ParamUpdateOrigin::kHotReload);

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kTypeMismatch);
}

TEST(ParamStoreTest, RejectsOutOfRange) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("shooter.burst", ParamValue(100));
    const ParamResult result = store.apply(updates, ParamUpdateOrigin::kHotReload);

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kOutOfRange);
}

TEST(ParamStoreTest, RejectsValueOutsideAllowedSet) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("mode", ParamValue("fly"));
    const ParamResult result = store.apply(updates, ParamUpdateOrigin::kHotReload);

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kValueNotAllowed);
}

TEST(ParamStoreTest, FailedBatchLeavesSnapshotUntouched) {
    ParamStore store(make_schema());
    const auto before = store.snapshot();

    ParamValueMap updates;
    updates.emplace("chassis.max_speed", ParamValue(5.0)); // 合法
    updates.emplace("shooter.burst", ParamValue(999));     // 非法

    ASSERT_FALSE(store.apply(updates, ParamUpdateOrigin::kHotReload).ok());

    const auto after = store.snapshot();
    EXPECT_EQ(after, before);
    EXPECT_DOUBLE_EQ(after->get_double("chassis.max_speed", 0.0), 3.0);
    EXPECT_EQ(after->generation(), 0U);
}

TEST(ParamStoreTest, CollectsAllErrorsInOneBatch) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("unknown.a", ParamValue(1));
    updates.emplace("unknown.b", ParamValue(2));

    const ParamResult result = store.apply(updates, ParamUpdateOrigin::kHotReload);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().size(), 2U);
}

TEST(ParamStoreTest, RejectsHotReloadOfNonReloadableParameter) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("serial.port", ParamValue("/dev/ttyACM0"));
    const ParamResult result = store.apply(updates, ParamUpdateOrigin::kHotReload);

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kNotHotReloadable);
    EXPECT_EQ(store.snapshot()->get_string("serial.port", ""), "/dev/ttyUSB0");
}

TEST(ParamStoreTest, AllowsInitialLoadOfNonReloadableParameter) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("serial.port", ParamValue("/dev/ttyACM0"));
    ASSERT_TRUE(store.apply(updates, ParamUpdateOrigin::kInitialLoad).ok());
    EXPECT_EQ(store.snapshot()->get_string("serial.port", ""), "/dev/ttyACM0");
}

TEST(ParamStoreTest, HotReloadWithUnchangedValueIsAllowed) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("serial.port", ParamValue("/dev/ttyUSB0"));
    EXPECT_TRUE(store.apply(updates, ParamUpdateOrigin::kHotReload).ok());
    EXPECT_EQ(store.snapshot()->generation(), 0U);
}

TEST(ParamStoreTest, GenerationOnlyAdvancesOnRealChange) {
    ParamStore store(make_schema());

    ParamValueMap updates;
    updates.emplace("shooter.burst", ParamValue(3));
    ASSERT_TRUE(store.apply(updates, ParamUpdateOrigin::kHotReload).ok());
    EXPECT_EQ(store.snapshot()->generation(), 0U);

    updates["shooter.burst"] = ParamValue(4);
    ASSERT_TRUE(store.apply(updates, ParamUpdateOrigin::kHotReload).ok());
    EXPECT_EQ(store.snapshot()->generation(), 1U);
}

TEST(ParamStoreTest, ReadersAlwaysSeeConsistentSnapshot) {
    ParamStore store(make_schema());

    std::atomic<bool> stop{false};
    std::atomic<int> inconsistent{0};

    std::vector<std::thread> readers;
    readers.reserve(4);
    for (int i = 0; i < 4; ++i) {
        readers.emplace_back([&store, &stop, &inconsistent] {
            while (!stop.load(std::memory_order_acquire)) {
                const auto snapshot = store.snapshot();
                // burst 与 magazine 总是同批写入，读者不应看到二者不一致。
                if (snapshot->get_int("shooter.burst", -1) !=
                    snapshot->get_int("shooter.magazine", -2)) {
                    inconsistent.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });
    }

    for (int i = 0; i < 2000; ++i) {
        const int value = 1 + (i % 10);
        ParamValueMap updates;
        updates.emplace("shooter.burst", ParamValue(value));
        updates.emplace("shooter.magazine", ParamValue(value));
        updates.emplace("chassis.max_speed", ParamValue(0.1 * static_cast<double>(i % 100)));
        ASSERT_TRUE(store.apply(updates, ParamUpdateOrigin::kHotReload).ok());
    }

    stop.store(true, std::memory_order_release);
    for (auto& reader : readers) {
        reader.join();
    }

    EXPECT_EQ(inconsistent.load(std::memory_order_relaxed), 0);
}

} // namespace
} // namespace foray::platform::param
