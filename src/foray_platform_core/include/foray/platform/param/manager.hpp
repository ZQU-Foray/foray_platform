#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "foray/platform/param/snapshot.hpp"
#include "foray/platform/param/source.hpp"
#include "foray/platform/param/store.hpp"
#include "foray/platform/param/value.hpp"

namespace foray::platform::param {

/**
 * @brief 参数系统门面：schema + 多来源分层 + 热加载。
 *
 * 分层覆盖：`sources` 按顺序加载，后出现的来源覆盖先出现的同名参数。
 * 这对应架构约定的两级配置：
 *
 * ```
 * config/shared.yaml       <- 全兵种共享
 * config/<robot_type>.yaml <- 兵种 / 场景差异
 * ```
 *
 * 生命周期：
 * 1. 构造时只建立 schema 与默认值，不读取任何来源；
 * 2. 启动期调用 `load_initial()`，允许写入不可热加载参数；
 * 3. 运行期在**非实时线程**调用 `reload()`；整批加载与校验要么全部成功，
 *    要么保持原快照不变。
 */
class ParamManager {
  public:
    /**
     * @param schema 参数声明。
     * @param sources 按覆盖顺序排列的参数源，可为空。
     */
    ParamManager(ParamSchema schema, std::vector<std::shared_ptr<ParamSource>> sources);

    /// @brief 当前参数快照。可在实时线程无锁读取。
    std::shared_ptr<const ParamSnapshot> snapshot() const noexcept;

    /**
     * @brief 启动期初始加载。
     *
     * 依次加载全部来源、合并后一次性校验并应用。失败时保持默认值。
     * 若已经初始化过，则退化为一次热加载（不会修改不可热加载参数）。
     */
    ParamResult load_initial();

    /**
     * @brief 运行期热加载。
     *
     * 重新读取全部来源并整体应用。未初始化时返回 `kNotInitialized`。
     * 不可热加载参数若值发生变化，整批失败。
     */
    ParamResult reload();

    const ParamSchema& schema() const noexcept {
        return store_.schema();
    }

    bool initialized() const noexcept;

    /// @brief 变更监听器句柄；0 表示无效。
    using ListenerId = std::uint64_t;

    /// @brief 单个参数的一次变更记录。
    struct ParamChange {
        std::string name;
        ParamValue old_value;
        ParamValue new_value;
    };

    /**
     * @brief 变更监听器。
     *
     * 在 `load_initial()` / `reload()` 成功且值确实变化的返回前，于**调用方线程**
     * 同步执行；只包含真正变化的键。监听器抛出的异常会被捕获并跳过该监听器，
     * 不影响 `load_initial()` / `reload()` 的结果。
     *
     * 监听器内**不得**再次调用 `load_initial()` / `reload()`（会死锁）。
     */
    using ParamChangeListener = std::function<void(const std::vector<ParamChange>&)>;

    /**
     * @brief 注册变更监听器。
     * @return 句柄，用于 `remove_change_listener()`。
     */
    ListenerId add_change_listener(ParamChangeListener listener);

    /**
     * @brief 注销变更监听器。
     * @return 存在并移除返回 `true`。
     */
    bool remove_change_listener(ListenerId id);

  private:
    ParamResult load_and_apply(ParamUpdateOrigin origin);

    /// @brief 在锁外通知所有监听器。
    void notify_listeners(const std::vector<ParamChange>& changes);

    ParamStore store_;
    std::vector<std::shared_ptr<ParamSource>> sources_;
    mutable std::mutex reload_mutex_;
    std::atomic<bool> initialized_{false};

    std::map<ListenerId, ParamChangeListener> listeners_;
    std::mutex listener_mutex_;
    ListenerId next_listener_id_ = 1;
};

} // namespace foray::platform::param
