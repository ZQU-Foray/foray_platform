#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "foray/platform/result/error.hpp"

namespace foray::platform::driver::hikvision::detail {

/**
 * @brief MVS 错误码的本地副本。
 *
 * 刻意不 include `MvErrorDefine.h`：错误映射属于 core 层，mock 测试必须能在没装
 * SDK 的机器上验证它。数值来源 `MvErrorDefine.h`（SDK V4.8.0.3），只收录驱动会
 * 遇到的码。
 */
namespace mvs_error {

inline constexpr std::uint32_t kHandle = 0x80000000U;      // MV_E_HANDLE
inline constexpr std::uint32_t kSupport = 0x80000001U;     // MV_E_SUPPORT
inline constexpr std::uint32_t kBufOver = 0x80000002U;     // MV_E_BUFOVER
inline constexpr std::uint32_t kParameter = 0x80000004U;   // MV_E_PARAMETER
inline constexpr std::uint32_t kResource = 0x80000006U;    // MV_E_RESOURCE
inline constexpr std::uint32_t kNoData = 0x80000007U;      // MV_E_NODATA
inline constexpr std::uint32_t kNoEnoughBuf = 0x8000000AU; // MV_E_NOENOUGH_BUF
inline constexpr std::uint32_t kAbnormalImage = 0x8000000BU; // MV_E_ABNORMAL_IMAGE
inline constexpr std::uint32_t kLoadLibrary = 0x8000000CU;   // MV_E_LOAD_LIBRARY
inline constexpr std::uint32_t kGcArgument = 0x80000101U;    // MV_E_GC_ARGUMENT
inline constexpr std::uint32_t kGcTimeout = 0x80000107U;     // MV_E_GC_TIMEOUT
inline constexpr std::uint32_t kNotImplemented = 0x80000200U; // MV_E_NOT_IMPLEMENTED
inline constexpr std::uint32_t kAccessDenied = 0x80000203U;   // MV_E_ACCESS_DENIED
inline constexpr std::uint32_t kBusy = 0x80000204U;           // MV_E_BUSY
inline constexpr std::uint32_t kUdpRecvData = 0x80000214U;    // MV_E_UDP_RECV_DATA

} // namespace mvs_error

/**
 * @brief MVS 原始错误码 → HAL 错误码。
 *
 * 映射表见 `docs/driver_hikvision.md` §8；未列出的码一律 `kSdkError`。
 */
ErrorCode map_mvs_error(std::uint32_t code) noexcept;

/**
 * @brief 构造错误，`vendor_code` 按位保真保存 MVS 原始码。
 *
 * `MV_E_*` 都在 `0x8000_xxxx`，转 int32 会变成负数；诊断时按 uint32 打印。
 */
Error make_mvs_error(ErrorCode code, std::string message, std::uint32_t vendor_code);

/**
 * @brief 便捷包装：由上下文与 MVS 原始码直接构造 `Error`。
 *
 * 消息形如 `"MV_CC_OpenDevice 失败: 0x80000214"`，错误码经 `map_mvs_error` 归一。
 */
Error mvs_failure(std::string_view context, std::uint32_t code);

} // namespace foray::platform::driver::hikvision::detail
