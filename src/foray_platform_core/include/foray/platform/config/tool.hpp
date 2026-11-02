#pragma once

#include <string>

#include "document.hpp"
#include "exporter.hpp"
#include "foray/platform/param/schema.hpp"
#include "resolver.hpp"

namespace foray::platform::config {

/**
 * @brief 配置栈的便捷门面，供 CLI 与测试复用。
 *
 * 注意：这里不包含可执行文件入口。`foray_config_tool` 可执行文件在
 * HAL/driver 提供真实 schema 后再落地（见 plan.md）。
 */
class ConfigTool {
  public:
    struct Validation {
        param::ParamResult result;
        ConfigSnapshot snapshot;
    };

    static Validation validate(const param::ParamSchema& schema, const ConfigDocument& document,
                               const EnvProvider& env);

    static std::string dump(const ConfigSnapshot& snapshot);
    static std::string schema_markdown(const param::ParamSchema& schema);
    static std::string schema_json(const param::ParamSchema& schema);
};

} // namespace foray::platform::config
