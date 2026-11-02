#pragma once

#include <memory>
#include <mutex>

#include "foray/platform/param/schema.hpp"
#include "foray/platform/param/snapshot.hpp"

namespace foray::platform::param {

/**
 * @brief 本次更新来自哪个阶段。
 *
 * - `kInitialLoad`：启动期首次加载，允许写入不可热加载参数；
 * - `kHotReload`：运行期热加载，不可热加载参数只能保持原值。
 */
enum class ParamUpdateOrigin {
    kInitialLoad,
    kHotReload,
};

/**
 * @brief 类型化参数存储。
 *
 * 线程模型：
 * - **读**：`snapshot()` 返回不可变快照的 `shared_ptr`，读者无锁；
 * - **写**：`apply()` 串行化（内部互斥），在写线程中复制映射、校验并
 *   原子替换快照；读者要么看到旧快照，要么看到新快照，不会看到中间状态；
 * - 写入会分配内存，**必须在非实时线程调用**（与 X4「调参不影响实时」一致）。
 *
 * 原子性：一次 `apply()` 中任意一条更新失败，整批更新都不生效。
 */
class ParamStore {
  public:
    /**
     * @brief 以 schema 构造，并用默认值初始化快照。
     * @param schema 已完成校验的 schema。
     */
    explicit ParamStore(ParamSchema schema);

    /// @brief 获取当前快照。线程安全、无锁读取。
    std::shared_ptr<const ParamSnapshot> snapshot() const noexcept;

    /**
     * @brief 原子地应用一批更新。
     *
     * 规则：
     * - 未知参数、类型不符、越界、枚举不允许 -> 整批失败；
     * - `kHotReload` 下修改 `hot_reloadable == false` 的参数 -> 整批失败
     *   （写入相同值视为无变化，允许）；
     * - 未提供值的参数保持当前值；
     * - 值没有实际变化时不递增代号。
     *
     * @return 成功或包含全部错误的失败结果。
     */
    ParamResult apply(const ParamValueMap& updates, ParamUpdateOrigin origin);

    const ParamSchema& schema() const noexcept {
        return schema_;
    }

  private:
    ParamSchema schema_;
    mutable std::mutex write_mutex_;
    std::shared_ptr<const ParamSnapshot> snapshot_;
};

} // namespace foray::platform::param
