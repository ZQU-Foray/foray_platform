#include "foray/platform/param/result.hpp"

#include <sstream>

namespace foray::platform::param {

const char* to_string(ParamErrorCode code) noexcept {
    switch (code) {
    case ParamErrorCode::kInvalidSchema:
        return "invalid_schema";
    case ParamErrorCode::kDuplicateParam:
        return "duplicate_param";
    case ParamErrorCode::kUnknownParam:
        return "unknown_param";
    case ParamErrorCode::kTypeMismatch:
        return "type_mismatch";
    case ParamErrorCode::kOutOfRange:
        return "out_of_range";
    case ParamErrorCode::kValueNotAllowed:
        return "value_not_allowed";
    case ParamErrorCode::kNotHotReloadable:
        return "not_hot_reloadable";
    case ParamErrorCode::kSourceFailure:
        return "source_failure";
    case ParamErrorCode::kNotInitialized:
        return "not_initialized";
    }
    return "unknown";
}

std::string ParamResult::to_string() const {
    if (ok()) {
        return "ok";
    }

    std::ostringstream stream;
    for (std::size_t i = 0; i < errors_.size(); ++i) {
        if (i != 0) {
            stream << '\n';
        }
        const auto& error = errors_[i];
        stream << param::to_string(error.code);
        if (!error.name.empty()) {
            stream << " [" << error.name << "]";
        }
        if (!error.message.empty()) {
            stream << ": " << error.message;
        }
    }
    return stream.str();
}

} // namespace foray::platform::param
