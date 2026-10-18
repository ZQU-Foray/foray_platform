#include "validation.hpp"

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace foray::platform::param::detail {

namespace {

bool is_identifier_start(char value) noexcept {
    return (value >= 'A' && value <= 'Z') || (value >= 'a' && value <= 'z') || value == '_';
}

bool is_identifier_char(char value) noexcept {
    return is_identifier_start(value) || (value >= '0' && value <= '9');
}

bool is_numeric_type(ParamType type) noexcept {
    return type == ParamType::kInt || type == ParamType::kDouble;
}

/// @brief 值类型是否可以无损地解释为声明类型。
bool is_compatible_type(ParamType expected, ParamType actual) noexcept {
    if (expected == actual) {
        return true;
    }
    // 只允许 int -> double 宽化；double -> int 会静默截断，禁止。
    return expected == ParamType::kDouble && actual == ParamType::kInt;
}

bool is_finite(const ParamValue& value) noexcept {
    if (value.type() != ParamType::kDouble) {
        return true;
    }
    return std::isfinite(value.as_double());
}

ParamError make_error(ParamErrorCode code, std::string name, std::string message) {
    return ParamError{code, std::move(name), std::move(message)};
}

bool value_in_allowed_set(const ParamSpec& spec, const ParamValue& value) {
    for (const auto& allowed : spec.allowed_values) {
        if (spec.type == ParamType::kDouble) {
            if (value.as_double() == allowed.as_double()) {
                return true;
            }
        } else if (spec.type == ParamType::kInt) {
            if (value.as_int() == allowed.as_int()) {
                return true;
            }
        } else if (value == allowed) {
            return true;
        }
    }
    return false;
}

} // namespace

bool is_valid_param_name(std::string_view name) noexcept {
    if (name.empty()) {
        return false;
    }

    bool at_segment_start = true;
    for (const char value : name) {
        if (value == '.') {
            if (at_segment_start) {
                return false;
            }
            at_segment_start = true;
            continue;
        }

        if (at_segment_start) {
            if (!is_identifier_start(value)) {
                return false;
            }
            at_segment_start = false;
        } else if (!is_identifier_char(value)) {
            return false;
        }
    }
    return !at_segment_start;
}

ParamResult validate_spec(const ParamSpec& spec) {
    std::vector<ParamError> errors;

    if (!is_valid_param_name(spec.name)) {
        errors.push_back(make_error(ParamErrorCode::kInvalidSchema, spec.name,
                                    "parameter name must be a dotted identifier"));
    }

    if (!spec.default_value.is_valid()) {
        errors.push_back(
            make_error(ParamErrorCode::kInvalidSchema, spec.name, "default value is invalid"));
    } else if (!is_compatible_type(spec.type, spec.default_value.type())) {
        errors.push_back(
            make_error(ParamErrorCode::kTypeMismatch, spec.name,
                       std::string("default value type '") + spec.default_value.type_name() +
                           "' does not match declared type '" + to_string(spec.type) + "'"));
    } else if (!is_finite(spec.default_value)) {
        errors.push_back(
            make_error(ParamErrorCode::kInvalidSchema, spec.name, "default value must be finite"));
    }

    if (!is_numeric_type(spec.type) && (spec.minimum.has_value() || spec.maximum.has_value())) {
        errors.push_back(make_error(ParamErrorCode::kInvalidSchema, spec.name,
                                    "minimum/maximum are only valid for numeric parameters"));
    }

    for (const auto* bound : {&spec.minimum, &spec.maximum}) {
        if (!bound->has_value()) {
            continue;
        }
        const ParamValue& value = bound->value();
        if (!value.is_valid() || !is_numeric_type(value.type())) {
            errors.push_back(
                make_error(ParamErrorCode::kInvalidSchema, spec.name, "bounds must be numeric"));
            continue;
        }
        if (!is_compatible_type(spec.type, value.type())) {
            errors.push_back(make_error(ParamErrorCode::kInvalidSchema, spec.name,
                                        "bound type does not match declared type"));
        } else if (!is_finite(value)) {
            errors.push_back(
                make_error(ParamErrorCode::kInvalidSchema, spec.name, "bounds must be finite"));
        } else if (spec.type == ParamType::kInt && value.type() != ParamType::kInt) {
            // int 参数的边界不能是 double，否则可能被静默截断。
            errors.push_back(make_error(ParamErrorCode::kInvalidSchema, spec.name,
                                        "int parameter bounds must be int"));
        }
    }

    if (errors.empty() && spec.minimum.has_value() && spec.maximum.has_value()) {
        const bool min_greater = spec.type == ParamType::kInt
                                     ? spec.minimum->as_int() > spec.maximum->as_int()
                                     : spec.minimum->as_double() > spec.maximum->as_double();
        if (min_greater) {
            errors.push_back(make_error(ParamErrorCode::kInvalidSchema, spec.name,
                                        "minimum must not be greater than maximum"));
        }
    }

    std::vector<ParamValue> coerced_allowed;
    for (const auto& allowed : spec.allowed_values) {
        if (!allowed.is_valid() || !is_compatible_type(spec.type, allowed.type())) {
            errors.push_back(make_error(ParamErrorCode::kInvalidSchema, spec.name,
                                        "allowed value has an incompatible type"));
            continue;
        }
        if (!is_finite(allowed)) {
            errors.push_back(make_error(ParamErrorCode::kInvalidSchema, spec.name,
                                        "allowed values must be finite"));
            continue;
        }
        const ParamValue coerced = coerce(spec, allowed);
        for (const auto& previous : coerced_allowed) {
            if (previous == coerced) {
                errors.push_back(make_error(ParamErrorCode::kInvalidSchema, spec.name,
                                            "duplicate allowed value"));
                break;
            }
        }
        coerced_allowed.push_back(coerced);
    }

    if (!errors.empty()) {
        return ParamResult::failure(std::move(errors));
    }

    // 结构合法后，复用运行期校验检查默认值的边界与枚举。
    return check_value(spec, spec.default_value);
}

ParamResult check_value(const ParamSpec& spec, const ParamValue& value) {
    if (!value.is_valid()) {
        return ParamResult::failure(
            make_error(ParamErrorCode::kTypeMismatch, spec.name, "value is invalid"));
    }

    if (!is_compatible_type(spec.type, value.type())) {
        return ParamResult::failure(make_error(ParamErrorCode::kTypeMismatch, spec.name,
                                               std::string("expected '") + to_string(spec.type) +
                                                   "', got '" + value.type_name() + "'"));
    }

    if (!is_finite(value)) {
        return ParamResult::failure(
            make_error(ParamErrorCode::kOutOfRange, spec.name, "value must be finite"));
    }

    if (spec.type == ParamType::kInt) {
        const std::int64_t numeric = value.as_int();
        if (spec.minimum.has_value() && numeric < spec.minimum->as_int()) {
            return ParamResult::failure(
                make_error(ParamErrorCode::kOutOfRange, spec.name,
                           "value is below minimum " + spec.minimum->to_string()));
        }
        if (spec.maximum.has_value() && numeric > spec.maximum->as_int()) {
            return ParamResult::failure(
                make_error(ParamErrorCode::kOutOfRange, spec.name,
                           "value is above maximum " + spec.maximum->to_string()));
        }
    } else if (spec.type == ParamType::kDouble) {
        const double numeric = value.as_double();
        if (spec.minimum.has_value() && numeric < spec.minimum->as_double()) {
            return ParamResult::failure(
                make_error(ParamErrorCode::kOutOfRange, spec.name,
                           "value is below minimum " + spec.minimum->to_string()));
        }
        if (spec.maximum.has_value() && numeric > spec.maximum->as_double()) {
            return ParamResult::failure(
                make_error(ParamErrorCode::kOutOfRange, spec.name,
                           "value is above maximum " + spec.maximum->to_string()));
        }
    }

    if (!spec.allowed_values.empty() && !value_in_allowed_set(spec, value)) {
        return ParamResult::failure(
            make_error(ParamErrorCode::kValueNotAllowed, spec.name, "value is not in allowed set"));
    }

    return ParamResult::success();
}

ParamValue coerce(const ParamSpec& spec, const ParamValue& value) {
    if (spec.type == ParamType::kDouble && value.type() == ParamType::kInt) {
        return ParamValue(value.as_double());
    }
    return value;
}

} // namespace foray::platform::param::detail
