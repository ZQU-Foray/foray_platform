#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "foray/platform/param/result.hpp"
#include "foray/platform/param/value.hpp"

namespace foray::platform::param {

/**
 * @brief 单个参数的声明。
 *
 * 参数名使用点分层次结构（例如 `chassis.max_speed`），每段只允许
 * `[A-Za-z_][A-Za-z0-9_]*`。点分名称与 YAML 嵌套映射一一对应。
 *
 * `source_note` 用于记录魔法数字的来源（规则手册页码、标定日期、赛季），
 * 与仓库规范中的「魔法数字必须注释来源」对应。
 */
struct ParamSpec {
    /// @brief 参数名，必须唯一。
    std::string name;

    /// @brief 期望类型。
    ParamType type = ParamType::kBool;

    /// @brief 默认值，类型必须与 `type` 兼容。
    ParamValue default_value;

    /// @brief 公共 API 文档：参数含义。
    std::string description;

    /// @brief 单位（物理量建议填写，例如 `m/s`、`rad`）。
    std::string unit;

    /// @brief 是否允许热加载修改。启动期初始加载不受此限制。
    bool hot_reloadable = true;

    /// @brief 数值下界（含）。仅对 `kInt` / `kDouble` 有效。
    std::optional<ParamValue> minimum;

    /// @brief 数值上界（含）。仅对 `kInt` / `kDouble` 有效。
    std::optional<ParamValue> maximum;

    /// @brief 允许值集合。非空时值必须属于该集合。
    std::vector<ParamValue> allowed_values;

    /// @brief 魔法数字来源说明。
    std::string source_note;
};

/**
 * @brief 参数 schema。
 *
 * 负责声明参数、校验声明本身是否合法，并提供按名称查询。
 * `add()` 失败时不会修改 schema。
 */
class ParamSchema {
  public:
    ParamSchema() = default;

    /**
     * @brief 添加一个参数声明。
     *
     * 校验内容：名称格式、名称唯一、默认值与类型一致、默认值满足边界与枚举、
     * 边界类型合法且下界不大于上界、枚举元素类型一致。
     */
    ParamResult add(ParamSpec spec);

    /// @brief 按名称查找；不存在返回 `nullptr`。
    const ParamSpec* find(std::string_view name) const noexcept;

    const std::vector<ParamSpec>& specs() const noexcept {
        return specs_;
    }

    std::size_t size() const noexcept {
        return specs_.size();
    }

    bool empty() const noexcept {
        return specs_.empty();
    }

  private:
    std::vector<ParamSpec> specs_;
    /// @brief 透明比较器允许用 `string_view` 查询，避免热路径分配。
    std::map<std::string, std::size_t, std::less<>> index_;
};

} // namespace foray::platform::param
