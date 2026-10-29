#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "foray/platform/config/config.hpp"
#include "temp_config.hpp"

namespace foray::platform::config {
namespace {

param::ParamSpec make_spec(std::string name, param::ParamType type,
                           param::ParamValue default_value) {
    param::ParamSpec spec;
    spec.name = std::move(name);
    spec.type = type;
    spec.default_value = std::move(default_value);
    return spec;
}

param::ParamSchema make_schema() {
    param::ParamSchema schema;

    param::ParamSpec width =
        make_spec("camera.width", param::ParamType::kInt, param::ParamValue(640));
    width.minimum = param::ParamValue(1);
    width.maximum = param::ParamValue(4096);
    EXPECT_TRUE(schema.add(std::move(width)).ok());

    EXPECT_TRUE(
        schema.add(make_spec("camera.serial", param::ParamType::kString, param::ParamValue("")))
            .ok());

    param::ParamSpec exposure =
        make_spec("camera.exposure_us", param::ParamType::kDouble, param::ParamValue(1000.0));
    exposure.minimum = param::ParamValue(1.0);
    exposure.maximum = param::ParamValue(1000000.0);
    EXPECT_TRUE(schema.add(std::move(exposure)).ok());

    EXPECT_TRUE(
        schema.add(make_spec("camera.enabled", param::ParamType::kBool, param::ParamValue(true)))
            .ok());

    EXPECT_TRUE(schema
                    .add(make_spec("lidar.host_ip", param::ParamType::kString,
                                   param::ParamValue("127.0.0.1")))
                    .ok());

    param::ParamSpec port =
        make_spec("lidar.port", param::ParamType::kInt, param::ParamValue(56301));
    port.minimum = param::ParamValue(1);
    port.maximum = param::ParamValue(65535);
    EXPECT_TRUE(schema.add(std::move(port)).ok());

    param::ParamSpec mode =
        make_spec("camera.mode", param::ParamType::kString, param::ParamValue("idle"));
    mode.allowed_values = {param::ParamValue("idle"), param::ParamValue("run")};
    EXPECT_TRUE(schema.add(std::move(mode)).ok());

    return schema;
}

std::string filename_of(const std::string* source) {
    return std::filesystem::path(*source).filename().string();
}

TEST(ConfigResolverTest, LoadsSingleLayerAndFillsDefaults) {
    test::TempConfigDir directory;
    const auto file = directory.write("shared.yaml", R"(
camera:
  width: 1280
  serial: "DA123"
lidar:
  host_ip: "192.168.1.5"
)");

    ConfigDocument document;
    document.add_layer("shared", file);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    const param::ParamResult result = resolver.load(document, env, snapshot);

    ASSERT_TRUE(result.ok()) << result.to_string();
    EXPECT_EQ(snapshot.values.at("camera.width").as_int(), 1280);
    EXPECT_EQ(snapshot.values.at("camera.serial").as_string(), "DA123");
    EXPECT_EQ(snapshot.values.at("lidar.host_ip").as_string(), "192.168.1.5");
    EXPECT_DOUBLE_EQ(snapshot.values.at("camera.exposure_us").as_double(), 1000.0);
    EXPECT_TRUE(snapshot.values.at("camera.enabled").as_bool());
    EXPECT_EQ(snapshot.values.at("lidar.port").as_int(), 56301);

    ASSERT_NE(snapshot.source("camera.width"), nullptr);
    EXPECT_EQ(filename_of(snapshot.source("camera.width")), "shared.yaml");
    EXPECT_EQ(snapshot.source("camera.exposure_us"), nullptr);
}

TEST(ConfigResolverTest, LaterLayerOverridesEarlier) {
    test::TempConfigDir directory;
    const auto shared = directory.write("shared.yaml", "camera:\n  width: 640\n");
    const auto robot = directory.write("robot.yaml", "camera:\n  width: 1920\n");

    ConfigDocument document;
    document.add_layer("shared", shared);
    document.add_layer("robot", robot);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    ASSERT_TRUE(resolver.load(document, env, snapshot).ok());

    EXPECT_EQ(snapshot.values.at("camera.width").as_int(), 1920);
    ASSERT_NE(snapshot.source("camera.width"), nullptr);
    EXPECT_EQ(filename_of(snapshot.source("camera.width")), "robot.yaml");
}

TEST(ConfigResolverTest, IncludeResolvesRelativePaths) {
    test::TempConfigDir directory;
    directory.write("base.yaml",
                    "camera:\n  width: 800\n  serial: \"base\"\nlidar:\n  port: 12345\n");
    const auto root = directory.write("root.yaml", "include: base.yaml\ncamera:\n  width: 1024\n");

    ConfigDocument document;
    document.add_layer("root", root);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    ASSERT_TRUE(resolver.load(document, env, snapshot).ok());

    EXPECT_EQ(snapshot.values.at("camera.width").as_int(), 1024);
    EXPECT_EQ(snapshot.values.at("camera.serial").as_string(), "base");
    EXPECT_EQ(snapshot.values.at("lidar.port").as_int(), 12345);
    EXPECT_EQ(filename_of(snapshot.source("camera.width")), "root.yaml");
    EXPECT_EQ(filename_of(snapshot.source("camera.serial")), "base.yaml");
}

TEST(ConfigResolverTest, IncludeSequenceIsAppliedInOrder) {
    test::TempConfigDir directory;
    directory.write("a.yaml", "camera:\n  width: 700\n");
    directory.write("b.yaml", "camera:\n  width: 800\n");
    const auto root = directory.write("root.yaml", "include:\n  - a.yaml\n  - b.yaml\n");

    ConfigDocument document;
    document.add_layer("root", root);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    ASSERT_TRUE(resolver.load(document, env, snapshot).ok());

    EXPECT_EQ(snapshot.values.at("camera.width").as_int(), 800);
    EXPECT_EQ(filename_of(snapshot.source("camera.width")), "b.yaml");
}

TEST(ConfigResolverTest, DetectsIncludeCycle) {
    test::TempConfigDir directory;
    directory.write("a.yaml", "include: b.yaml\n");
    const auto a = directory.write("a.yaml", "include: b.yaml\n");
    directory.write("b.yaml", "include: a.yaml\n");

    ConfigDocument document;
    document.add_layer("a", a);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    const param::ParamResult result = resolver.load(document, env, snapshot);

    ASSERT_FALSE(result.ok());
    bool found_cycle = false;
    for (const auto& error : result.errors()) {
        if (error.message.find("cycle") != std::string::npos) {
            found_cycle = true;
        }
    }
    EXPECT_TRUE(found_cycle);
}

TEST(ConfigResolverTest, SubstitutesEnvironmentVariables) {
    test::TempConfigDir directory;
    const auto file = directory.write(
        "shared.yaml", "camera:\n  serial: \"${CAM_SERIAL}\"\n  width: \"${CAM_WIDTH}\"\n");

    MapEnvProvider env;
    env.set("CAM_SERIAL", "DA999");
    env.set("CAM_WIDTH", "1280");

    ConfigDocument document;
    document.add_layer("shared", file);

    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    ASSERT_TRUE(resolver.load(document, env, snapshot).ok());

    EXPECT_EQ(snapshot.values.at("camera.serial").as_string(), "DA999");
    EXPECT_EQ(snapshot.values.at("camera.width").as_int(), 1280);
}

TEST(ConfigResolverTest, UndefinedEnvironmentVariableFails) {
    test::TempConfigDir directory;
    const auto file = directory.write("shared.yaml", "camera:\n  serial: \"${MISSING_VAR}\"\n");

    const MapEnvProvider env;
    ConfigDocument document;
    document.add_layer("shared", file);

    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    const param::ParamResult result = resolver.load(document, env, snapshot);

    ASSERT_FALSE(result.ok());
    bool found_variable = false;
    for (const auto& error : result.errors()) {
        if (error.message.find("MISSING_VAR") != std::string::npos) {
            found_variable = true;
        }
    }
    EXPECT_TRUE(found_variable);
}

TEST(ConfigResolverTest, AppliesEnvironmentOverrideLayer) {
    test::TempConfigDir directory;
    const auto file = directory.write("shared.yaml", "camera:\n  width: 640\n");

    MapEnvProvider env;
    env.set("FORAY_CAMERA_WIDTH", "1920");

    ConfigDocument document;
    document.add_layer("shared", file);

    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    ASSERT_TRUE(resolver.load(document, env, snapshot).ok());

    EXPECT_EQ(snapshot.values.at("camera.width").as_int(), 1920);
    ASSERT_NE(snapshot.source("camera.width"), nullptr);
    EXPECT_EQ(*snapshot.source("camera.width"), "env:FORAY_CAMERA_WIDTH");

    ConfigDocument without_env = document;
    without_env.enable_env_layer = false;
    ConfigSnapshot no_env_snapshot;
    ASSERT_TRUE(resolver.load(without_env, env, no_env_snapshot).ok());
    EXPECT_EQ(no_env_snapshot.values.at("camera.width").as_int(), 640);
}

TEST(ConfigResolverTest, RejectsUnknownKey) {
    test::TempConfigDir directory;
    const auto file = directory.write("shared.yaml", "camera:\n  unknown: 1\n");

    ConfigDocument document;
    document.add_layer("shared", file);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    const param::ParamResult result = resolver.load(document, env, snapshot);

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, param::ParamErrorCode::kUnknownParam);
}

TEST(ConfigResolverTest, RejectsTypeMismatch) {
    test::TempConfigDir directory;
    const auto file = directory.write("shared.yaml", "camera:\n  width: wide\n");

    ConfigDocument document;
    document.add_layer("shared", file);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    const param::ParamResult result = resolver.load(document, env, snapshot);

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, param::ParamErrorCode::kTypeMismatch);
}

TEST(ConfigResolverTest, RequiredLayerMissingFailsButOptionalLayerIsSkipped) {
    test::TempConfigDir directory;
    const auto missing = directory.path() / "missing.yaml";

    ConfigDocument required_document;
    required_document.add_layer("missing", missing, true);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    EXPECT_FALSE(resolver.load(required_document, env, snapshot).ok());

    ConfigDocument optional_document;
    optional_document.add_layer("missing", missing, false);
    ConfigSnapshot optional_snapshot;
    EXPECT_TRUE(resolver.load(optional_document, env, optional_snapshot).ok());
    EXPECT_EQ(optional_snapshot.values.at("camera.width").as_int(), 640);
}

TEST(ConfigResolverTest, DetectsDuplicateFlatAndNestedKeys) {
    test::TempConfigDir directory;
    const auto file = directory.write("shared.yaml", "camera:\n  width: 640\ncamera.width: 1280\n");

    ConfigDocument document;
    document.add_layer("shared", file);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    const param::ParamResult result = resolver.load(document, env, snapshot);

    ASSERT_FALSE(result.ok());
    bool found_duplicate = false;
    for (const auto& error : result.errors()) {
        if (error.code == param::ParamErrorCode::kDuplicateParam) {
            found_duplicate = true;
        }
    }
    EXPECT_TRUE(found_duplicate);
}

TEST(ConfigResolverTest, WidensIntToDouble) {
    test::TempConfigDir directory;
    const auto file = directory.write("shared.yaml", "camera:\n  exposure_us: 2000\n");

    ConfigDocument document;
    document.add_layer("shared", file);

    const MapEnvProvider env;
    const ConfigResolver resolver(make_schema());
    ConfigSnapshot snapshot;
    ASSERT_TRUE(resolver.load(document, env, snapshot).ok());

    EXPECT_EQ(snapshot.values.at("camera.exposure_us").type(), param::ParamType::kDouble);
    EXPECT_DOUBLE_EQ(snapshot.values.at("camera.exposure_us").as_double(), 2000.0);
}

TEST(ConfigToolTest, ValidateReturnsSnapshot) {
    test::TempConfigDir directory;
    const auto file = directory.write("shared.yaml", "camera:\n  width: 1280\n");

    ConfigDocument document;
    document.add_layer("shared", file);

    const MapEnvProvider env;
    const auto validation = ConfigTool::validate(make_schema(), document, env);

    ASSERT_TRUE(validation.result.ok());
    EXPECT_EQ(validation.snapshot.values.at("camera.width").as_int(), 1280);
}

} // namespace
} // namespace foray::platform::config
