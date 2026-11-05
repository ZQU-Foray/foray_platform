#pragma once

#include <map>
#include <optional>
#include <string>
#include <string_view>

#include "document.hpp"
#include "foray/platform/param/result.hpp"
#include "foray/platform/param/schema.hpp"
#include "snapshot.hpp"

namespace foray::platform::config {

/**
 * @brief 环境变量提供者。
 *
 * 允许测试注入受控环境；生产使用 `ProcessEnvProvider`。
 */
class EnvProvider {
  public:
    virtual ~EnvProvider() = default;

    /// @brief 读取变量；不存在返回 `std::nullopt`。
    virtual std::optional<std::string> get(std::string_view name) const = 0;
};

/// @brief 读取当前进程环境（`std::getenv`）。
class ProcessEnvProvider final : public EnvProvider {
  public:
    std::optional<std::string> get(std::string_view name) const override;
};

/// @brief 内存环境变量表，用于测试。
class MapEnvProvider final : public EnvProvider {
  public:
    MapEnvProvider() = default;
    explicit MapEnvProvider(std::map<std::string, std::string, std::less<>> values);

    void set(std::string name, std::string value);
    std::optional<std::string> get(std::string_view name) const override;

  private:
    std::map<std::string, std::string, std::less<>> values_;
};

/**
 * @brief 统一配置解析器。
 *
 * 功能：
 * - 按 `ConfigDocument` 的层顺序加载 YAML；
 * - 支持文件内顶层 `include:`（字符串或字符串序列，路径相对当前文件）；
 * - 标量支持 `${ENV_VAR}` 替换；未定义变量报错；
 * - 文件层之后可选应用 `FORAY_<PARAM>` 环境变量覆盖；
 * - 使用 `param::ParamSchema` 做类型/范围/枚举校验；
 * - 记录每个显式值的来源文件（provenance）。
 *
 * 不负责：热加载、设备能力探测、机器人特定配置值。
 */
class ConfigResolver {
  public:
    explicit ConfigResolver(param::ParamSchema schema);

    /// @brief 使用当前进程环境加载。
    param::ParamResult load(const ConfigDocument& document, ConfigSnapshot& out) const;

    /// @brief 使用指定环境加载（测试/嵌入场景）。
    param::ParamResult load(const ConfigDocument& document, const EnvProvider& env,
                            ConfigSnapshot& out) const;

    const param::ParamSchema& schema() const noexcept {
        return schema_;
    }

  private:
    param::ParamSchema schema_;
};

} // namespace foray::platform::config
