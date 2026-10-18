#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "foray/platform/result/result.hpp"

namespace foray::platform {
namespace {

TEST(ResultTest, HoldsValue) {
    const Result<int> result(42);

    EXPECT_TRUE(result.ok());
    EXPECT_TRUE(static_cast<bool>(result));
    EXPECT_EQ(result.value(), 42);
}

TEST(ResultTest, HoldsError) {
    const Result<int> result = Result<int>::failure(make_error(ErrorCode::kTimeout, "timed out"));

    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(static_cast<bool>(result));
    EXPECT_EQ(result.error().code, ErrorCode::kTimeout);
    EXPECT_EQ(result.error().message, "timed out");
}

TEST(ResultTest, AccessingWrongAlternativeThrows) {
    const Result<int> failure = Result<int>::failure(make_error(ErrorCode::kInternal, "boom"));
    EXPECT_THROW((void)failure.value(), std::bad_variant_access);

    const Result<int> success(1);
    EXPECT_THROW((void)success.error(), std::bad_variant_access);
}

TEST(ResultTest, SupportsMoveOnlyValue) {
    Result<std::unique_ptr<int>> result =
        Result<std::unique_ptr<int>>::success(std::make_unique<int>(7));

    ASSERT_TRUE(result.ok());
    ASSERT_NE(result.value(), nullptr);
    EXPECT_EQ(*result.value(), 7);

    std::unique_ptr<int> moved = std::move(result).value();
    EXPECT_EQ(*moved, 7);
}

TEST(ResultVoidTest, SuccessAndFailure) {
    const Result<void> success = Result<void>::success();
    EXPECT_TRUE(success.ok());
    EXPECT_THROW((void)success.error(), std::logic_error);

    const Result<void> failure = Result<void>::failure(make_error(ErrorCode::kNoData, "no data"));
    EXPECT_FALSE(failure.ok());
    EXPECT_EQ(failure.error().code, ErrorCode::kNoData);
    EXPECT_EQ(failure.error().message, "no data");
}

TEST(ErrorTest, KeepsVendorCode) {
    const Error error =
        make_error(ErrorCode::kSdkError, "MVS failed", static_cast<std::int32_t>(0x80000001));

    EXPECT_EQ(error.code, ErrorCode::kSdkError);
    EXPECT_EQ(error.vendor_code, static_cast<std::int32_t>(0x80000001));
    EXPECT_FALSE(error.ok());
}

TEST(ErrorTest, Names) {
    EXPECT_STREQ(to_string(ErrorCode::kOk), "ok");
    EXPECT_STREQ(to_string(ErrorCode::kSdkUnavailable), "sdk_unavailable");
    EXPECT_STREQ(to_string(ErrorCode::kDeviceAmbiguous), "device_ambiguous");
    EXPECT_STREQ(to_string(ErrorCode::kUnsupportedFormat), "unsupported_format");
}

} // namespace
} // namespace foray::platform
