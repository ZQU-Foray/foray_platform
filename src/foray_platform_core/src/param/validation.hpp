#pragma once

#include <string_view>

#include "foray/platform/param/result.hpp"
#include "foray/platform/param/schema.hpp"
#include "foray/platform/param/value.hpp"

namespace foray::platform::param::detail {

/// @brief 参数名是否为合法的点分标识符。
bool is_valid_param_name(std::string_view name) noexcept;

/// @brief 校验 schema 声明本身。
ParamResult validate_spec(const ParamSpec& spec);

/**
 * @brief 校验值是否符合声明。
 *
 * 类型允许 `int -> double` 宽化；检查边界与允许值集合。
 */
ParamResult check_value(const ParamSpec& spec, const ParamValue& value);

/// @brief 将值转换为声明类型（目前只做 `int -> double`）。
ParamValue coerce(const ParamSpec& spec, const ParamValue& value);

} // namespace foray::platform::param::detail
