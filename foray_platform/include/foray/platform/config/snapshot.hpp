#pragma once

#include <map>
#include <string>
#include <string_view>

#include "foray/platform/param/snapshot.hpp"

namespace foray::platform::config {

/**
 * @brief 配置加载结果：已校验的值 + 每个值的来源。
 *
 * `values` 包含 schema 的全部参数（未显式配置的项使用默认值）；
 * `provenance` 只记录**显式提供过**的参数，值为来源文件路径或
 * `env:<变量名>`。
 */
struct ConfigSnapshot {
    param::ParamValueMap values;
    std::map<std::string, std::string, std::less<>> provenance;

    /// @brief 按名称查找值；不存在返回 `nullptr`。
    const param::ParamValue* find(std::string_view name) const noexcept {
        const auto found = values.find(name);
        return found == values.end() ? nullptr : &found->second;
    }

    /// @brief 按名称查找来源；默认值返回 `nullptr`。
    const std::string* source(std::string_view name) const noexcept {
        const auto found = provenance.find(name);
        return found == provenance.end() ? nullptr : &found->second;
    }
};

} // namespace foray::platform::config
