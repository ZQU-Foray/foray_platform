#include <gtest/gtest.h>

#include <stdexcept>

#include "foray/platform/param/value.hpp"

namespace foray::platform::param {
namespace {

TEST(ParamValueTest, DefaultIsInvalid) {
    const ParamValue value;
    EXPECT_FALSE(value.is_valid());
    EXPECT_STREQ(value.type_name(), "invalid");
    EXPECT_EQ(value.to_string(), "<invalid>");
    EXPECT_THROW((void)value.type(), std::logic_error);
}

TEST(ParamValueTest, ReportsType) {
    EXPECT_EQ(ParamValue(true).type(), ParamType::kBool);
    EXPECT_EQ(ParamValue(1).type(), ParamType::kInt);
    EXPECT_EQ(ParamValue(std::int64_t{1}).type(), ParamType::kInt);
    EXPECT_EQ(ParamValue(1.0).type(), ParamType::kDouble);
    EXPECT_EQ(ParamValue("text").type(), ParamType::kString);
    EXPECT_EQ(ParamValue(std::string("text")).type(), ParamType::kString);
}

TEST(ParamValueTest, TypedAccessors) {
    EXPECT_TRUE(ParamValue(true).as_bool());
    EXPECT_EQ(ParamValue(7).as_int(), 7);
    EXPECT_DOUBLE_EQ(ParamValue(2.5).as_double(), 2.5);
    // int 可宽化为 double。
    EXPECT_DOUBLE_EQ(ParamValue(3).as_double(), 3.0);
    EXPECT_EQ(ParamValue("abc").as_string(), "abc");

    EXPECT_THROW((void)ParamValue(1.0).as_int(), std::bad_variant_access);
    EXPECT_THROW((void)ParamValue("abc").as_double(), std::bad_variant_access);
}

TEST(ParamValueTest, NumericPredicate) {
    EXPECT_TRUE(ParamValue(1).is_numeric());
    EXPECT_TRUE(ParamValue(1.0).is_numeric());
    EXPECT_FALSE(ParamValue(true).is_numeric());
    EXPECT_FALSE(ParamValue("1").is_numeric());
}

TEST(ParamValueTest, StringRepresentation) {
    EXPECT_EQ(ParamValue(true).to_string(), "true");
    EXPECT_EQ(ParamValue(false).to_string(), "false");
    EXPECT_EQ(ParamValue(42).to_string(), "42");
    EXPECT_EQ(ParamValue("hello").to_string(), "hello");
    EXPECT_EQ(ParamValue(1.5).to_string(), "1.5");
}

TEST(ParamValueTest, Equality) {
    EXPECT_EQ(ParamValue(1), ParamValue(1));
    EXPECT_NE(ParamValue(1), ParamValue(1.0));
    EXPECT_NE(ParamValue("1"), ParamValue(1));
    EXPECT_EQ(ParamValue(), ParamValue());
}

TEST(ParamTypeTest, Names) {
    EXPECT_STREQ(to_string(ParamType::kBool), "bool");
    EXPECT_STREQ(to_string(ParamType::kInt), "int");
    EXPECT_STREQ(to_string(ParamType::kDouble), "double");
    EXPECT_STREQ(to_string(ParamType::kString), "string");
}

} // namespace
} // namespace foray::platform::param
