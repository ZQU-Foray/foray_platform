#include <gtest/gtest.h>

#include <string>

#include "foray/platform/param/schema.hpp"

namespace foray::platform::param {
namespace {

ParamSpec make_spec(std::string name, ParamType type, ParamValue default_value) {
    ParamSpec spec;
    spec.name = std::move(name);
    spec.type = type;
    spec.default_value = std::move(default_value);
    return spec;
}

TEST(ParamSchemaTest, AddsValidSpecsAndFindsThem) {
    ParamSchema schema;

    EXPECT_TRUE(
        schema.add(make_spec("chassis.max_speed", ParamType::kDouble, ParamValue(3.5))).ok());
    EXPECT_TRUE(schema.add(make_spec("gimbal.fire", ParamType::kBool, ParamValue(false))).ok());
    EXPECT_TRUE(schema.add(make_spec("mode", ParamType::kString, ParamValue("idle"))).ok());

    EXPECT_EQ(schema.size(), 3U);
    ASSERT_NE(schema.find("chassis.max_speed"), nullptr);
    EXPECT_EQ(schema.find("chassis.max_speed")->type, ParamType::kDouble);
    EXPECT_EQ(schema.find("missing"), nullptr);

    // string_view 查询不需要构造临时 string。
    const std::string name = "mode";
    ASSERT_NE(schema.find(std::string_view(name)), nullptr);
}

TEST(ParamSchemaTest, RejectsDuplicateNames) {
    ParamSchema schema;
    ASSERT_TRUE(schema.add(make_spec("a", ParamType::kInt, ParamValue(1))).ok());

    const ParamResult result = schema.add(make_spec("a", ParamType::kDouble, ParamValue(2.0)));
    ASSERT_FALSE(result.ok());
    ASSERT_EQ(result.errors().size(), 1U);
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kDuplicateParam);
}

TEST(ParamSchemaTest, RejectsInvalidNames) {
    const char* invalid_names[] = {"", ".a", "a.", "a..b", "1a", "a b", "a-b", "a/b"};

    for (const char* name : invalid_names) {
        ParamSchema schema;
        const ParamResult result = schema.add(make_spec(name, ParamType::kInt, ParamValue(1)));
        EXPECT_FALSE(result.ok()) << "name should be rejected: '" << name << "'";
        EXPECT_EQ(schema.size(), 0U);
    }
}

TEST(ParamSchemaTest, AcceptsDottedNames) {
    ParamSchema schema;
    EXPECT_TRUE(schema.add(make_spec("a.b.c_d1", ParamType::kInt, ParamValue(0))).ok());
}

TEST(ParamSchemaTest, RejectsDefaultTypeMismatch) {
    ParamSchema schema;
    const ParamResult result = schema.add(make_spec("a", ParamType::kInt, ParamValue("1")));

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kTypeMismatch);
}

TEST(ParamSchemaTest, AllowsIntDefaultForDoubleSpec) {
    ParamSchema schema;
    EXPECT_TRUE(schema.add(make_spec("a", ParamType::kDouble, ParamValue(1))).ok());
    EXPECT_EQ(schema.find("a")->default_value.type(), ParamType::kInt);
}

TEST(ParamSchemaTest, RejectsDoubleDefaultForIntSpec) {
    ParamSchema schema;
    const ParamResult result = schema.add(make_spec("a", ParamType::kInt, ParamValue(1.0)));
    EXPECT_FALSE(result.ok());
}

TEST(ParamSchemaTest, RejectsBoundsOnNonNumericTypes) {
    ParamSchema schema;
    ParamSpec spec = make_spec("flag", ParamType::kBool, ParamValue(false));
    spec.minimum = ParamValue(0);

    const ParamResult result = schema.add(std::move(spec));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kInvalidSchema);
}

TEST(ParamSchemaTest, RejectsInvertedBounds) {
    ParamSchema schema;
    ParamSpec spec = make_spec("a", ParamType::kInt, ParamValue(5));
    spec.minimum = ParamValue(10);
    spec.maximum = ParamValue(1);

    const ParamResult result = schema.add(std::move(spec));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kInvalidSchema);
}

TEST(ParamSchemaTest, RejectsDoubleBoundsForIntSpec) {
    ParamSchema schema;
    ParamSpec spec = make_spec("a", ParamType::kInt, ParamValue(1));
    spec.minimum = ParamValue(0.5);

    const ParamResult result = schema.add(std::move(spec));
    ASSERT_FALSE(result.ok());
}

TEST(ParamSchemaTest, RejectsDefaultOutsideBounds) {
    ParamSchema schema;
    ParamSpec spec = make_spec("a", ParamType::kInt, ParamValue(11));
    spec.minimum = ParamValue(0);
    spec.maximum = ParamValue(10);

    const ParamResult result = schema.add(std::move(spec));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kOutOfRange);
}

TEST(ParamSchemaTest, AcceptsDefaultOnBoundary) {
    ParamSchema schema;
    ParamSpec spec = make_spec("a", ParamType::kDouble, ParamValue(10));
    spec.minimum = ParamValue(0);
    spec.maximum = ParamValue(10);

    EXPECT_TRUE(schema.add(std::move(spec)).ok());
}

TEST(ParamSchemaTest, RejectsDefaultOutsideAllowedSet) {
    ParamSchema schema;
    ParamSpec spec = make_spec("mode", ParamType::kString, ParamValue("idle"));
    spec.allowed_values = {ParamValue("run"), ParamValue("stop")};

    const ParamResult result = schema.add(std::move(spec));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kValueNotAllowed);
}

TEST(ParamSchemaTest, RejectsMistypedAllowedValues) {
    ParamSchema schema;
    ParamSpec spec = make_spec("a", ParamType::kInt, ParamValue(1));
    spec.allowed_values = {ParamValue(1), ParamValue("two")};

    const ParamResult result = schema.add(std::move(spec));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kInvalidSchema);
}

TEST(ParamSchemaTest, RejectsDuplicateAllowedValuesAfterWidening) {
    ParamSchema schema;
    ParamSpec spec = make_spec("a", ParamType::kDouble, ParamValue(1.0));
    spec.allowed_values = {ParamValue(1), ParamValue(1.0)};

    const ParamResult result = schema.add(std::move(spec));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.errors().front().code, ParamErrorCode::kInvalidSchema);
}

} // namespace
} // namespace foray::platform::param
