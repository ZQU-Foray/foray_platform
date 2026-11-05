#include "foray/platform/result/error.hpp"

#include <utility>

namespace foray::platform {

const char* to_string(ErrorCode code) noexcept {
    switch (code) {
    case ErrorCode::kOk:
        return "ok";
    case ErrorCode::kInvalidArgument:
        return "invalid_argument";
    case ErrorCode::kNotSupported:
        return "not_supported";
    case ErrorCode::kSdkUnavailable:
        return "sdk_unavailable";
    case ErrorCode::kSdkError:
        return "sdk_error";
    case ErrorCode::kDeviceNotFound:
        return "device_not_found";
    case ErrorCode::kDeviceAmbiguous:
        return "device_ambiguous";
    case ErrorCode::kPermissionDenied:
        return "permission_denied";
    case ErrorCode::kNotOpen:
        return "not_open";
    case ErrorCode::kAlreadyOpen:
        return "already_open";
    case ErrorCode::kBusy:
        return "busy";
    case ErrorCode::kTimeout:
        return "timeout";
    case ErrorCode::kNoData:
        return "no_data";
    case ErrorCode::kDisconnected:
        return "disconnected";
    case ErrorCode::kQueueFull:
        return "queue_full";
    case ErrorCode::kFrameTooLarge:
        return "frame_too_large";
    case ErrorCode::kUnsupportedFormat:
        return "unsupported_format";
    case ErrorCode::kInternal:
        return "internal";
    }
    return "unknown";
}

Error make_error(ErrorCode code, std::string message, std::int32_t vendor_code) {
    return Error{code, std::move(message), vendor_code};
}

} // namespace foray::platform
