#pragma once

#include <string>

#include "foray/platform/param/schema.hpp"
#include "snapshot.hpp"

namespace foray::platform::config {

/**
 * @brief 配置导出工具：合并结果、schema 文档与 JSON Schema。
 *
 * 所有函数只做字符串生成，不读写文件，可用于 CLI 与 CI。
 */
class ConfigExporter {
  public:
    /// @brief 导出已合并的值；每行附带来源注释。
    static std::string dump(const ConfigSnapshot& snapshot);

    /// @brief 由 schema 生成 Markdown 表格，供文档/评审使用。
    static std::string schema_markdown(const param::ParamSchema& schema);

    /// @brief 由 schema 生成 JSON Schema（draft 2020-12 风格）。
    static std::string schema_json(const param::ParamSchema& schema);
};

} // namespace foray::platform::config
