#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <variant>

namespace foray::platform::param {

/**
 * @brief 参数类型。
 */
enum class ParamType {
    kBool,
    kInt,
    kDouble,
    kString,
};

/// @brief 返回类型名称，用于日志与错误信息。
constexpr const char* to_string(ParamType type) noexcept {
    switch (type) {
    case ParamType::kBool:
        return "bool";
    case ParamType::kInt:
        return "int";
    case ParamType::kDouble:
        return "double";
    case ParamType::kString:
        return "string";
    }
    return "unknown";
}

/**
 * @brief 类型化参数值。
 *
 * 内部使用 `std::variant` 保存四种标量。默认构造为「无效值」，
 * 用于表示 YAML 中的空值等异常情况；schema 校验会拒绝无效值。
 *
 * 类型转换规则（仅在校验阶段使用）：
 * - `int` 可宽化为 `double`；
 * - `double` **不能**窄化为 `int`；
 * - 其余跨类型转换一律拒绝，避免配置中的隐式错误。
 */
class ParamValue {
  public:
    /// @brief 无效值。
    ParamValue() noexcept = default;

    ParamValue(bool value) noexcept : value_(value) {
    }
    ParamValue(int value) noexcept : value_(static_cast<std::int64_t>(value)) {
    }
    ParamValue(std::int64_t value) noexcept : value_(value) {
    }
    ParamValue(double value) noexcept : value_(value) {
    }
    ParamValue(std::string value) : value_(std::move(value)) {
    }
    ParamValue(const char* value) : value_(std::string(value)) {
    }

    /// @brief 是否持有有效值。
    bool is_valid() const noexcept {
        return !std::holds_alternative<std::monostate>(value_);
    }

    /// @brief 返回类型。无效值抛出 `std::logic_error`。
    ParamType type() const;

    /// @brief 是否为数值类型（`kInt` 或 `kDouble`）。
    bool is_numeric() const noexcept {
        return std::holds_alternative<std::int64_t>(value_) ||
               std::holds_alternative<double>(value_);
    }

    /// @brief 读取 bool。类型不符时抛出 `std::bad_variant_access`。
    bool as_bool() const {
        return std::get<bool>(value_);
    }

    /// @brief 读取 int64。类型不符时抛出 `std::bad_variant_access`。
    std::int64_t as_int() const {
        return std::get<std::int64_t>(value_);
    }

    /**
     * @brief 读取 double。
     *
     * `int` 会自动宽化为 `double`；其他类型抛出 `std::bad_variant_access`。
     */
    double as_double() const;

    /// @brief 读取 string。类型不符时抛出 `std::bad_variant_access`。
    const std::string& as_string() const {
        return std::get<std::string>(value_);
    }

    /// @brief 返回类型名称；无效值返回 `"invalid"`。
    const char* type_name() const noexcept;

    /// @brief 返回人类可读的字符串表示，用于日志与错误信息。
    std::string to_string() const;

    friend bool operator==(const ParamValue& lhs, const ParamValue& rhs) {
        return lhs.value_ == rhs.value_;
    }

    friend bool operator!=(const ParamValue& lhs, const ParamValue& rhs) {
        return !(lhs == rhs);
    }

  private:
    std::variant<std::monostate, bool, std::int64_t, double, std::string> value_;
};

} // namespace foray::platform::param
