#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "foray/platform/param/manager.hpp"
#include "foray/platform/param/source.hpp"
#include "foray/platform/param/yaml_source.hpp"
#include "temp_yaml.hpp"

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
    EXPECT_TRUE(
        schema.add(make_spec("chassis.max_speed", ParamType::kDouble, ParamValue(1.0))).ok());

    ParamSpec burst = make_spec("shooter.burst", ParamType::kInt, ParamValue(3));
    burst.minimum = ParamValue(1);
    burst.maximum = ParamValue(10);
    EXPECT_TRUE(schema.add(std::move(burst)).ok());

    ParamSpec port = make_spec("serial.port", ParamType::kString, ParamValue("/dev/ttyUSB0"));
    port.hot_reloadable = false;
    EXPECT_TRUE(schema.add(std::move(port)).ok());
    return schema;
}

/**
 * @brief 可控失败的参数源，用于验证错误传播。
 */
class FailingSource final : public ParamSource {
  public:
    std::string name() const override {
        return "failing";
    }

    ParamResult load(const ParamSchema& /*schema*/, ParamValueMap& /*out*/) const override {
        return ParamResult::failure(
            ParamError{ParamErrorCode::kSourceFailure, "failing", "intentional failure"});
    }
};

TEST(ParamManagerTest, LoadsWithoutSourcesUsingDefaults) {
    ParamManager manager(make_schema(), {});
    ASSERT_TRUE(manager.load_initial().ok());

    const auto snapshot = manager.snapshot();
    EXPECT_DOUBLE_EQ(snapshot->get_double("chassis.max_speed", 0.0), 1.0);
    EXPECT_EQ(snapshot->get_int("shooter.burst", 0), 3);
    EXPECT_TRUE(manager.initialized());
}

TEST(ParamManagerTest, LaterSourceOverridesEarlierSource) {
    auto shared = std::make_shared<MapParamSource>(ParamValueMap{
        {"chassis.max_speed", ParamValue(2.0)},
        {"shooter.burst", ParamValue(4)},
    });
    auto robot = std::make_shared<MapParamSource>(ParamValueMap{
        {"shooter.burst", ParamValue(6)},
    });

    ParamManager manager(make_schema(), {shared, robot});
    ASSERT_TRUE(manager.load_initial().ok());

    const auto snapshot = manager.snapshot();
    EXPECT_DOUBLE_EQ(snapshot->get_double("chassis.max_speed", 0.0), 2.0);
    EXPECT_EQ(snapshot->get_int("shooter.burst", 0), 6);
}

TEST(ParamManagerTest, ReloadPicksUpChangedSource) {
    auto source =
        std::make_shared<MapParamSource>(ParamValueMap{{"chassis.max_speed", ParamValue(2.0)}});

    ParamManager manager(make_schema(), {source});
    ASSERT_TRUE(manager.load_initial().ok());
    const std::uint64_t generation = manager.snapshot()->generation();

    source->set(ParamValueMap{{"chassis.max_speed", ParamValue(5.0)}});
    ASSERT_TRUE(manager.reload().ok());

    const auto snapshot = manager.snapshot();
    EXPECT_DOUBLE_EQ(snapshot->get_double("chassis.max_speed", 0.0), 5.0);
    EXPECT_EQ(snapshot->generation(), generation + 1);
}

TEST(ParamManagerTest, FailedReloadKeepsPreviousSnapshot) {
    auto source = std::make_shared<MapParamSource>(ParamValueMap{{"shooter.burst", ParamValue(4)}});

    ParamManager manager(make_schema(), {source});
    ASSERT_TRUE(manager.load_initial().ok());
    const auto before = manager.snapshot();

    source->set(ParamValueMap{{"shooter.burst", ParamValue(999)}});
    const ParamResult result = manager.reload();

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kOutOfRange);
    EXPECT_EQ(manager.snapshot(), before);
}

TEST(ParamManagerTest, ReloadBeforeInitialLoadFails) {
    ParamManager manager(make_schema(), {});
    const ParamResult result = manager.reload();

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kNotInitialized);
}

TEST(ParamManagerTest, SourceFailureIsReported) {
    ParamManager manager(make_schema(), {std::make_shared<FailingSource>()});
    const ParamResult result = manager.load_initial();

    ASSERT_FALSE(result.ok());
    EXPECT_FALSE(manager.initialized());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kSourceFailure);
}

TEST(ParamManagerTest, InitialLoadMaySetNonReloadableParameter) {
    auto source = std::make_shared<MapParamSource>(
        ParamValueMap{{"serial.port", ParamValue("/dev/ttyACM0")}});

    ParamManager manager(make_schema(), {source});
    ASSERT_TRUE(manager.load_initial().ok());
    EXPECT_EQ(manager.snapshot()->get_string("serial.port", ""), "/dev/ttyACM0");
}

TEST(ParamManagerTest, HotReloadCannotChangeNonReloadableParameter) {
    auto source = std::make_shared<MapParamSource>(
        ParamValueMap{{"serial.port", ParamValue("/dev/ttyACM0")}});

    ParamManager manager(make_schema(), {source});
    ASSERT_TRUE(manager.load_initial().ok());

    source->set(ParamValueMap{{"serial.port", ParamValue("/dev/ttyUSB1")}});
    const ParamResult result = manager.reload();

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kNotHotReloadable);
    EXPECT_EQ(manager.snapshot()->get_string("serial.port", ""), "/dev/ttyACM0");
}

TEST(ParamManagerTest, SecondInitialLoadBehavesLikeHotReload) {
    auto source = std::make_shared<MapParamSource>(
        ParamValueMap{{"serial.port", ParamValue("/dev/ttyACM0")}});

    ParamManager manager(make_schema(), {source});
    ASSERT_TRUE(manager.load_initial().ok());

    source->set(ParamValueMap{{"serial.port", ParamValue("/dev/ttyUSB1")}});
    const ParamResult result = manager.load_initial();

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kNotHotReloadable);
}

TEST(ParamManagerTest, YamlSourceReloadIntegration) {
    test::TempYaml file(R"(
chassis:
  max_speed: 2.0
shooter:
  burst: 5
)");

    auto source = std::make_shared<YamlFileParamSource>(file.path());
    ParamManager manager(make_schema(), {source});

    ASSERT_TRUE(manager.load_initial().ok());
    EXPECT_DOUBLE_EQ(manager.snapshot()->get_double("chassis.max_speed", 0.0), 2.0);
    EXPECT_EQ(manager.snapshot()->get_int("shooter.burst", 0), 5);

    file.write(R"(
chassis:
  max_speed: 4.0
shooter:
  burst: 6
)");
    ASSERT_TRUE(manager.reload().ok());
    EXPECT_DOUBLE_EQ(manager.snapshot()->get_double("chassis.max_speed", 0.0), 4.0);
    EXPECT_EQ(manager.snapshot()->get_int("shooter.burst", 0), 6);
}

} // namespace
} // namespace foray::platform::param
