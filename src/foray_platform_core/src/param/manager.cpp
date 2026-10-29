#include "foray/platform/param/manager.hpp"

#include <exception>
#include <utility>
#include <vector>

namespace foray::platform::param {

namespace {

/// @brief 计算两个快照之间的差异；只保留值真正变化的键。
std::vector<ParamManager::ParamChange> diff_snapshots(const ParamSnapshot& before,
                                                      const ParamSnapshot& after) {
    std::vector<ParamManager::ParamChange> changes;
    for (const auto& entry : after.values()) {
        const ParamValue* old_value = before.find(entry.first);
        if (old_value != nullptr && *old_value == entry.second) {
            continue;
        }
        ParamManager::ParamChange change;
        change.name = entry.first;
        if (old_value != nullptr) {
            change.old_value = *old_value;
        }
        change.new_value = entry.second;
        changes.push_back(std::move(change));
    }
    return changes;
}

} // namespace

ParamManager::ParamManager(ParamSchema schema, std::vector<std::shared_ptr<ParamSource>> sources)
    : store_(std::move(schema)), sources_(std::move(sources)) {
}

std::shared_ptr<const ParamSnapshot> ParamManager::snapshot() const noexcept {
    return store_.snapshot();
}

bool ParamManager::initialized() const noexcept {
    return initialized_.load(std::memory_order_acquire);
}

ParamManager::ListenerId ParamManager::add_change_listener(ParamChangeListener listener) {
    if (!listener) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(listener_mutex_);
    const ListenerId id = next_listener_id_++;
    listeners_.emplace(id, std::move(listener));
    return id;
}

bool ParamManager::remove_change_listener(ListenerId id) {
    std::lock_guard<std::mutex> lock(listener_mutex_);
    return listeners_.erase(id) > 0;
}

void ParamManager::notify_listeners(const std::vector<ParamChange>& changes) {
    if (changes.empty()) {
        return;
    }

    // 在锁内复制监听器列表，锁外执行，允许监听器安全地注册/注销自身。
    std::vector<ParamChangeListener> callbacks;
    {
        std::lock_guard<std::mutex> lock(listener_mutex_);
        callbacks.reserve(listeners_.size());
        for (const auto& entry : listeners_) {
            callbacks.push_back(entry.second);
        }
    }

    for (const auto& callback : callbacks) {
        try {
            callback(changes);
        } catch (...) {
            // 监听器异常不影响参数加载结果；保持继续通知其余监听器。
        }
    }
}

ParamResult ParamManager::load_initial() {
    std::vector<ParamChange> changes;
    ParamResult result;
    {
        std::lock_guard<std::mutex> lock(reload_mutex_);

        // 重复调用时按热加载处理，避免第二次启动加载改写不可热加载参数。
        const ParamUpdateOrigin origin = initialized_.load(std::memory_order_acquire)
                                             ? ParamUpdateOrigin::kHotReload
                                             : ParamUpdateOrigin::kInitialLoad;
        const std::shared_ptr<const ParamSnapshot> before = store_.snapshot();
        result = load_and_apply(origin);
        if (result.ok()) {
            initialized_.store(true, std::memory_order_release);
            const std::shared_ptr<const ParamSnapshot> after = store_.snapshot();
            if (after->generation() != before->generation()) {
                changes = diff_snapshots(*before, *after);
            }
        }
    }

    notify_listeners(changes);
    return result;
}

ParamResult ParamManager::reload() {
    std::vector<ParamChange> changes;
    ParamResult result;
    {
        std::lock_guard<std::mutex> lock(reload_mutex_);

        if (!initialized_.load(std::memory_order_acquire)) {
            return ParamResult::failure(ParamError{ParamErrorCode::kNotInitialized, "",
                                                   "load_initial() has not succeeded yet"});
        }

        const std::shared_ptr<const ParamSnapshot> before = store_.snapshot();
        result = load_and_apply(ParamUpdateOrigin::kHotReload);
        if (result.ok()) {
            const std::shared_ptr<const ParamSnapshot> after = store_.snapshot();
            if (after->generation() != before->generation()) {
                changes = diff_snapshots(*before, *after);
            }
        }
    }

    notify_listeners(changes);
    return result;
}

ParamResult ParamManager::load_and_apply(ParamUpdateOrigin origin) {
    ParamValueMap merged;
    std::vector<ParamError> errors;

    for (const auto& source : sources_) {
        ParamValueMap loaded;
        ParamResult result = source->load(store_.schema(), loaded);
        if (!result.ok()) {
            errors.push_back(ParamError{ParamErrorCode::kSourceFailure, source->name(),
                                        "failed to load parameter source"});
            errors.insert(errors.end(), result.errors().begin(), result.errors().end());
            continue;
        }

        for (const auto& [name, value] : loaded) {
            merged[name] = value;
        }
    }

    if (!errors.empty()) {
        return ParamResult::failure(std::move(errors));
    }

    return store_.apply(merged, origin);
}

} // namespace foray::platform::param
