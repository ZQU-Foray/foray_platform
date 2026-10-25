#include <gtest/gtest.h>

#include <filesystem>
#include <string>

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
        schema.add(make_spec("chassis.max_speed", ParamType::kDouble, ParamValue(3.0))).ok());
    EXPECT_TRUE(schema.add(make_spec("chassis.enabled", ParamType::kBool, ParamValue(false))).ok());
    EXPECT_TRUE(schema.add(make_spec("shooter.burst", ParamType::kInt, ParamValue(3))).ok());
    EXPECT_TRUE(
        schema.add(make_spec("robot.name", ParamType::kString, ParamValue("unknown"))).ok());

    ParamSpec port = make_spec("serial.port", ParamType::kString, ParamValue("/dev/ttyUSB0"));
    port.hot_reloadable = false;
    EXPECT_TRUE(schema.add(std::move(port)).ok());
    return schema;
}

TEST(YamlSourceTest, ParsesNestedAndFlatKeys) {
    const test::TempYaml file(R"(
chassis:
  max_speed: 3.5
  enabled: true
shooter.burst: 5
robot:
  name: sentry
)");

    const ParamSchema schema = make_schema();
    const YamlFileParamSource source(file.path());
    ParamValueMap values;

    ASSERT_TRUE(source.load(schema, values).ok());
    ASSERT_EQ(values.size(), 4U);
    EXPECT_DOUBLE_EQ(values.at("chassis.max_speed").as_double(), 3.5);
    EXPECT_TRUE(values.at("chassis.enabled").as_bool());
    EXPECT_EQ(values.at("shooter.burst").as_int(), 5);
    EXPECT_EQ(values.at("robot.name").as_string(), "sentry");
}

TEST(YamlSourceTest, ParsesIntegerAsDouble) {
    const test::TempYaml file("chassis:\n  max_speed: 2\n");

    const ParamSchema schema = make_schema();
    const YamlFileParamSource source(file.path());
    ParamValueMap values;

    ASSERT_TRUE(source.load(schema, values).ok());
    EXPECT_DOUBLE_EQ(values.at("chassis.max_speed").as_double(), 2.0);
}

TEST(YamlSourceTest, EmptyFileIsValid) {
    const test::TempYaml file("");

    const ParamSchema schema = make_schema();
    const YamlFileParamSource source(file.path());
    ParamValueMap values;
    values.emplace("stale", ParamValue(1));

    ASSERT_TRUE(source.load(schema, values).ok());
    EXPECT_TRUE(values.empty());
}

TEST(YamlSourceTest, MissingFileFails) {
    const ParamSchema schema = make_schema();
    const YamlFileParamSource source(
        (std::filesystem::temp_directory_path() / "foray_platform_missing.yaml").string());
    ParamValueMap values;

    const ParamResult result = source.load(schema, values);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kSourceFailure);
}

TEST(YamlSourceTest, UnknownKeyFails) {
    const test::TempYaml file("chassis:\n  max_speeed: 3.5\n");

    const ParamSchema schema = make_schema();
    const YamlFileParamSource source(file.path());
    ParamValueMap values;

    const ParamResult result = source.load(schema, values);
    ASSERT_FALSE(result.ok());
    ASSERT_EQ(result.errors().size(), 1U);
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kUnknownParam);
    EXPECT_EQ(result.errors().front().name, "chassis.max_speeed");
}

TEST(YamlSourceTest, TypeMismatchFails) {
    const test::TempYaml file("shooter:\n  burst: many\n");

    const ParamSchema schema = make_schema();
    const YamlFileParamSource source(file.path());
    ParamValueMap values;

    const ParamResult result = source.load(schema, values);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kTypeMismatch);
}

TEST(YamlSourceTest, NonFiniteDoubleFails) {
    const test::TempYaml file("chassis:\n  max_speed: .nan\n");

    const ParamSchema schema = make_schema();
    const YamlFileParamSource source(file.path());
    ParamValueMap values;

    const ParamResult result = source.load(schema, values);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kTypeMismatch);
}

TEST(YamlSourceTest, SequenceFails) {
    const test::TempYaml file("shooter:\n  burst:\n    - 1\n    - 2\n");

    const ParamSchema schema = make_schema();
    const YamlFileParamSource source(file.path());
    ParamValueMap values;

    const ParamResult result = source.load(schema, values);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kTypeMismatch);
}

TEST(YamlSourceTest, RootMustBeMapping) {
    const test::TempYaml file("- 1\n- 2\n");

    const ParamSchema schema = make_schema();
    const YamlFileParamSource source(file.path());
    ParamValueMap values;

    const ParamResult result = source.load(schema, values);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kTypeMismatch);
}

TEST(YamlSourceTest, ReloadSeesRewrittenFile) {
    test::TempYaml file("chassis:\n  max_speed: 1.0\n");

    ParamSchema schema;
    ASSERT_TRUE(
        schema.add(make_spec("chassis.max_speed", ParamType::kDouble, ParamValue(0.0))).ok());

    const YamlFileParamSource source(file.path());
    ParamValueMap first;
    ASSERT_TRUE(source.load(schema, first).ok());
    EXPECT_DOUBLE_EQ(first.at("chassis.max_speed").as_double(), 1.0);

    file.write("chassis:\n  max_speed: 2.0\n");
    ParamValueMap second;
    ASSERT_TRUE(source.load(schema, second).ok());
    EXPECT_DOUBLE_EQ(second.at("chassis.max_speed").as_double(), 2.0);
}

} // namespace
} // namespace foray::platform::param
