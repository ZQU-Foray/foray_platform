#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace foray::platform::config {

/**
 * @brief 配置文件中一层。
 *
 * 典型顺序：`shared` → `<robot>` → `local`。
 * 后加载的层覆盖先加载的同名参数。
 */
struct ConfigLayer {
    /// @brief 层名称，用于日志与诊断（如 `shared` / `sentry` / `local`）。
    std::string name;
    /// @brief YAML 文件路径。
    std::filesystem::path path;
    /// @brief 文件不存在时是否报错；可选层缺失时跳过。
    bool required = true;
};

/**
 * @brief 配置文档：有序的层集合 + 环境变量覆盖策略。
 *
 * 该类型只描述“加载什么”，不读取文件；解析与校验由 `ConfigResolver` 完成。
 */
struct ConfigDocument {
    std::vector<ConfigLayer> layers;
    /// @brief 是否在所有文件层之后应用 `FORAY_<PARAM>` 环境变量覆盖。
    bool enable_env_layer = true;
    /// @brief 环境变量前缀；默认 `FORAY_`。
    std::string env_prefix = "FORAY_";

    void add_layer(std::string name, std::filesystem::path path, bool required = true) {
        layers.push_back(ConfigLayer{std::move(name), std::move(path), required});
    }
};

} // namespace foray::platform::config
