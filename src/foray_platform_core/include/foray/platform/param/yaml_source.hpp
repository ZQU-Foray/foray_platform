#pragma once

#include <string>

#include "foray/platform/param/source.hpp"

namespace foray::platform::param {

/**
 * @brief 从 YAML 文件读取参数。
 *
 * 文件格式：
 * ```yaml
 * chassis:
 *   max_speed: 3.5      # 解析为 chassis.max_speed
 *   wheel_radius: 0.15
 * ```
 * 只支持映射与标量，不支持序列。标量按 schema 中声明的类型解析；
 * 类型解析失败会报 `kTypeMismatch`。
 *
 * 严格性：
 * - 文件中出现 schema 未声明的键 -> `kUnknownParam`（防止拼写错误被静默忽略）；
 * - 文件不存在或 YAML 解析失败 -> `kSourceFailure`；
 * - 空文件合法，表示不覆盖任何参数。
 *
 * 类型解析规则：bool 只接受 `true` / `false`；int 要求完整的十进制整数；
 * double 接受整数与小数；string 接受任意标量。
 */
class YamlFileParamSource final : public ParamSource {
  public:
    /// @param path YAML 文件路径。每次 `load()` 都会重新读取，支持外部修改后热加载。
    explicit YamlFileParamSource(std::string path);

    std::string name() const override;

    ParamResult load(const ParamSchema& schema, ParamValueMap& out) const override;

  private:
    std::string path_;
};

} // namespace foray::platform::param
