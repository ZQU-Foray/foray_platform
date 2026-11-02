#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "foray/platform/param/result.hpp"
#include "foray/platform/param/schema.hpp"
#include "foray/platform/param/snapshot.hpp"

namespace foray::platform::param {

/**
 * @brief 参数来源抽象。
 *
 * 实现可以是 YAML 文件、内存映射、ROS 参数服务等。`load()` 必须是
 * **只读且可重复调用**的：热加载会在运行期再次调用它。
 */
class ParamSource {
  public:
    virtual ~ParamSource() = default;

    /// @brief 来源名称，用于日志与错误信息（文件路径、`"memory"` 等）。
    virtual std::string name() const = 0;

    /**
     * @brief 读取全部参数。
     *
     * @param schema 用于按期望类型解析（YAML 等无类型来源需要）。
     * @param out 成功时写入解析结果；失败时不得修改。
     *
     * 只需要返回该来源**显式提供**的参数；未出现的参数保持默认值。
     */
    virtual ParamResult load(const ParamSchema& schema, ParamValueMap& out) const = 0;
};

/**
 * @brief 内存参数源，主要用于测试与程序化配置。
 *
 * `set()` 与 `load()` 可并发调用。
 */
class MapParamSource final : public ParamSource {
  public:
    MapParamSource() = default;

    explicit MapParamSource(ParamValueMap values);

    /// @brief 整体替换当前值。
    void set(ParamValueMap values);

    std::string name() const override;

    ParamResult load(const ParamSchema& schema, ParamValueMap& out) const override;

  private:
    mutable std::mutex mutex_;
    ParamValueMap values_;
};

} // namespace foray::platform::param
