#include "mvs_error.hpp"

#include <cstdio>

namespace foray::platform::driver::hikvision::detail {

namespace {

std::string format_code(std::uint32_t code) {
    char buffer[16] = {};
    std::snprintf(buffer, sizeof(buffer), "0x%08X", code);
    return std::string(buffer);
}

} // namespace

ErrorCode map_mvs_error(std::uint32_t code) noexcept {
    switch (code) {
    case mvs_error::kNoData:
        return ErrorCode::kNoData;
    case mvs_error::kGcTimeout:
        return ErrorCode::kTimeout;
    case mvs_error::kAccessDenied:
        return ErrorCode::kPermissionDenied;
    case mvs_error::kParameter:
    case mvs_error::kGcArgument:
        return ErrorCode::kInvalidArgument;
    case mvs_error::kSupport:
    case mvs_error::kNotImplemented:
        return ErrorCode::kNotSupported;
    case mvs_error::kHandle:
        return ErrorCode::kInternal;
    case mvs_error::kBufOver:
    case mvs_error::kNoEnoughBuf:
        return ErrorCode::kFrameTooLarge;
    case mvs_error::kBusy:
        return ErrorCode::kBusy;
    // 缺传输层插件或库路径不对时 MVS 报这个码，对上层等价于「SDK 用不了」。
    case mvs_error::kLoadLibrary:
        return ErrorCode::kSdkUnavailable;
    // 实测 GigE 打开相机时网段不通就报 MV_E_UDP_RECV_DATA，属于链路问题。
    case mvs_error::kUdpRecvData:
        return ErrorCode::kDisconnected;
    case mvs_error::kResource:
    case mvs_error::kAbnormalImage:
        return ErrorCode::kSdkError;
    default:
        return ErrorCode::kSdkError;
    }
}

Error make_mvs_error(ErrorCode code, std::string message, std::uint32_t vendor_code) {
    return make_error(code, std::move(message), static_cast<std::int32_t>(vendor_code));
}

Error mvs_failure(std::string_view context, std::uint32_t code) {
    std::string message;
    message.reserve(context.size() + 32);
    message.append(context);
    message.append(" 失败: ");
    message.append(format_code(code));
    return make_mvs_error(map_mvs_error(code), std::move(message), code);
}

} // namespace foray::platform::driver::hikvision::detail
