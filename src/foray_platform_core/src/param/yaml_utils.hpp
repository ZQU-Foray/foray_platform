#pragma once

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "foray/platform/param/result.hpp"
#include "foray/platform/param/schema.hpp"
#include "foray/platform/param/value.hpp"

namespace foray::platform::param::detail {

/// @brief 构造 YAML 来源错误。
ParamError make_yaml_error(ParamErrorCode code, std::string name, std::string message);

/**
 * @brief 把嵌套映射展开为点分名称的叶子列表。
 *
 * - 只接受映射与标量；序列/空值报错；
 * - `prefix` 为空时从根开始；
 * - `source_name` 用于错误与来源标识。
 */
void flatten_yaml_node(const YAML::Node& node, const std::string& prefix,
                       std::vector<std::pair<std::string, YAML::Node>>& leaves,
                       std::vector<ParamError>& errors, const std::string& source_name);

/**
 * @brief 按 schema 的期望类型解析标量文本。
 *
 * 供普通 YAML 来源与配置栈共用；不读取文件、不做环境变量替换。
 */
ParamResult parse_scalar_text(const ParamSpec& spec, std::string_view text, ParamValue& out);

} // namespace foray::platform::param::detail
