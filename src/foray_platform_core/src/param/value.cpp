#include "foray/platform/param/value.hpp"

#include <sstream>
#include <stdexcept>

namespace foray::platform::param {

ParamType ParamValue::type() const {
    if (std::holds_alternative<bool>(value_)) {
        return ParamType::kBool;
    }
    if (std::holds_alternative<std::int64_t>(value_)) {
        return ParamType::kInt;
    }
    if (std::holds_alternative<double>(value_)) {
        return ParamType::kDouble;
    }
    if (std::holds_alternative<std::string>(value_)) {
        return ParamType::kString;
    }
    throw std::logic_error("ParamValue::type() called on an invalid value");
}

double ParamValue::as_double() const {
    if (const auto* value = std::get_if<double>(&value_)) {
        return *value;
    }
    if (const auto* value = std::get_if<std::int64_t>(&value_)) {
        return static_cast<double>(*value);
    }
    throw std::bad_variant_access();
}

const char* ParamValue::type_name() const noexcept {
    if (std::holds_alternative<bool>(value_)) {
        return "bool";
    }
    if (std::holds_alternative<std::int64_t>(value_)) {
        return "int";
    }
    if (std::holds_alternative<double>(value_)) {
        return "double";
    }
    if (std::holds_alternative<std::string>(value_)) {
        return "string";
    }
    return "invalid";
}

std::string ParamValue::to_string() const {
    if (const auto* value = std::get_if<bool>(&value_)) {
        return *value ? "true" : "false";
    }
    if (const auto* value = std::get_if<std::int64_t>(&value_)) {
        return std::to_string(*value);
    }
    if (const auto* value = std::get_if<double>(&value_)) {
        std::ostringstream stream;
        // 保留足够位数，避免日志中的 1.0 显示为 1，误导排查。
        stream.precision(17);
        stream << *value;
        return stream.str();
    }
    if (const auto* value = std::get_if<std::string>(&value_)) {
        return *value;
    }
    return "<invalid>";
}

} // namespace foray::platform::param
