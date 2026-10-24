#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "hikvision/mvs_error.hpp"

namespace foray::platform::driver::hikvision::detail {
namespace {

/**
 * @brief `docs/driver_hikvision.md` §8 的映射表逐项验证。
 *
 * 这些码决定了上层看到的是「超时」还是「断链」还是「参数错」，错一个就会导致
 * 状态机走错分支（例如把 `MV_E_NODATA` 当致命错误会让相机每 100 ms 掉线一次）。
 */
TEST(HikvisionError, MapsDocumentedCodes) {
    EXPECT_EQ(map_mvs_error(mvs_error::kNoData), ErrorCode::kNoData);
    EXPECT_EQ(map_mvs_error(mvs_error::kGcTimeout), ErrorCode::kTimeout);
    EXPECT_EQ(map_mvs_error(mvs_error::kAccessDenied), ErrorCode::kPermissionDenied);
    EXPECT_EQ(map_mvs_error(mvs_error::kParameter), ErrorCode::kInvalidArgument);
    EXPECT_EQ(map_mvs_error(mvs_error::kGcArgument), ErrorCode::kInvalidArgument);
    EXPECT_EQ(map_mvs_error(mvs_error::kSupport), ErrorCode::kNotSupported);
    EXPECT_EQ(map_mvs_error(mvs_error::kNotImplemented), ErrorCode::kNotSupported);
    EXPECT_EQ(map_mvs_error(mvs_error::kHandle), ErrorCode::kInternal);
    EXPECT_EQ(map_mvs_error(mvs_error::kBufOver), ErrorCode::kFrameTooLarge);
    EXPECT_EQ(map_mvs_error(mvs_error::kNoEnoughBuf), ErrorCode::kFrameTooLarge);
    EXPECT_EQ(map_mvs_error(mvs_error::kBusy), ErrorCode::kBusy);
    EXPECT_EQ(map_mvs_error(mvs_error::kLoadLibrary), ErrorCode::kSdkUnavailable);
    EXPECT_EQ(map_mvs_error(mvs_error::kUdpRecvData), ErrorCode::kDisconnected);
}

/// @brief 损坏帧是 SDK 错误而非断链：驱动要计数但不能据此把相机下线。
TEST(HikvisionError, AbnormalImageIsSdkError) {
    EXPECT_EQ(map_mvs_error(mvs_error::kAbnormalImage), ErrorCode::kSdkError);
    EXPECT_EQ(map_mvs_error(mvs_error::kResource), ErrorCode::kSdkError);
}

/// @brief 未收录的码一律归一为 `kSdkError`，不得落到 `kOk`。
TEST(HikvisionError, UnknownCodeBecomesSdkError) {
    EXPECT_EQ(map_mvs_error(0x80009999U), ErrorCode::kSdkError);
    EXPECT_EQ(map_mvs_error(0U), ErrorCode::kSdkError);
}

/**
 * @brief `vendor_code` 按位保真，诊断时才能回查 SDK 手册。
 *
 * `MV_E_*` 都 ≥ 0x8000_0000，int32 下为负值；这里断言转回 uint32 后与原始码一致。
 */
TEST(HikvisionError, VendorCodeKeepsBitPattern) {
    const auto error = make_mvs_error(ErrorCode::kDisconnected, "链路断开", mvs_error::kUdpRecvData);

    EXPECT_EQ(static_cast<std::uint32_t>(error.vendor_code), mvs_error::kUdpRecvData);
    EXPECT_LT(error.vendor_code, 0);
}

/// @brief `mvs_failure()` 的消息要同时带上下文与十六进制原始码。
TEST(HikvisionError, FailureMessageCarriesContextAndVendorCode) {
    const auto error = mvs_failure("MV_CC_OpenDevice", mvs_error::kUdpRecvData);

    EXPECT_EQ(error.code, ErrorCode::kDisconnected);
    EXPECT_NE(error.message.find("MV_CC_OpenDevice"), std::string::npos);
    EXPECT_NE(error.message.find("0x80000214"), std::string::npos);
}

/// @brief 成功码（0）在没有 SDK 上下文时也要表现为失败，避免静默通过。
TEST(HikvisionError, FailureWithZeroCodeIsStillFailure) {
    const auto error = mvs_failure("MV_CC_StopGrabbing", 0U);

    EXPECT_FALSE(error.ok());
    EXPECT_EQ(error.code, ErrorCode::kSdkError);
}

} // namespace
} // namespace foray::platform::driver::hikvision::detail
