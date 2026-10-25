#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "foray/platform/config/config.hpp"

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
    width.description = "image width";
    width.unit = "px";
    EXPECT_TRUE(schema.add(std::move(width)).ok());

    EXPECT_TRUE(
        schema.add(make_spec("camera.serial", param::ParamType::kString, param::ParamValue("")))
            .ok());

    EXPECT_TRUE(
        schema.add(make_spec("camera.enabled", param::ParamType::kBool, param::ParamValue(true)))
            .ok());

    param::ParamSpec mode =
        make_spec("camera.mode", param::ParamType::kString, param::ParamValue("idle"));
    mode.allowed_values = {param::ParamValue("idle"), param::ParamValue("run")};
    mode.hot_reloadable = false;
    EXPECT_TRUE(schema.add(std::move(mode)).ok());

    return schema;
}

ConfigSnapshot make_snapshot() {
    ConfigSnapshot snapshot;
    snapshot.values.emplace("camera.width", param::ParamValue(1280));
    snapshot.values.emplace("camera.serial", param::ParamValue("DA123"));
    snapshot.values.emplace("camera.enabled", param::ParamValue(true));
    snapshot.values.emplace("camera.mode", param::ParamValue("run"));
    snapshot.provenance.emplace("camera.width", "shared.yaml");
    return snapshot;
}

TEST(ConfigExporterTest, DumpContainsValuesAndSources) {
    const std::string dump = ConfigExporter::dump(make_snapshot());

    EXPECT_NE(dump.find("camera.width: 1280  # shared.yaml"), std::string::npos);
    EXPECT_NE(dump.find("camera.serial: \"DA123\""), std::string::npos);
    EXPECT_NE(dump.find("camera.enabled: true"), std::string::npos);
}

TEST(ConfigExporterTest, MarkdownContainsSchemaDetails) {
    const std::string markdown = ConfigExporter::schema_markdown(make_schema());

    EXPECT_NE(markdown.find("`camera.width`"), std::string::npos);
    EXPECT_NE(markdown.find("int"), std::string::npos);
    EXPECT_NE(markdown.find("[1, 4096]"), std::string::npos);
    EXPECT_NE(markdown.find("image width [px]"), std::string::npos);
    EXPECT_NE(markdown.find("否"), std::string::npos);
}

TEST(ConfigExporterTest, JsonSchemaContainsTypesBoundsAndEnum) {
    const std::string json = ConfigExporter::schema_json(make_schema());

    EXPECT_NE(json.find("\"$schema\""), std::string::npos);
    EXPECT_NE(json.find("\"additionalProperties\": false"), std::string::npos);
    EXPECT_NE(json.find("\"camera.width\""), std::string::npos);
    EXPECT_NE(json.find("\"type\": \"integer\""), std::string::npos);
    EXPECT_NE(json.find("\"minimum\": 1"), std::string::npos);
    EXPECT_NE(json.find("\"maximum\": 4096"), std::string::npos);
    EXPECT_NE(json.find("\"enum\": [\"idle\", \"run\"]"), std::string::npos);
    EXPECT_NE(json.find("\"type\": \"boolean\""), std::string::npos);
}

TEST(ConfigToolTest, ExportsThroughFacade) {
    const ConfigSnapshot snapshot = make_snapshot();

    EXPECT_EQ(ConfigTool::dump(snapshot), ConfigExporter::dump(snapshot));
    EXPECT_EQ(ConfigTool::schema_markdown(make_schema()),
              ConfigExporter::schema_markdown(make_schema()));
    EXPECT_EQ(ConfigTool::schema_json(make_schema()), ConfigExporter::schema_json(make_schema()));
}

} // namespace
} // namespace foray::platform::config
