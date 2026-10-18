#include "foray/platform/param/manager.hpp"

#include <utility>
#include <vector>

namespace foray::platform::param {

ParamManager::ParamManager(ParamSchema schema, std::vector<std::shared_ptr<ParamSource>> sources)
    : store_(std::move(schema)), sources_(std::move(sources)) {
}

std::shared_ptr<const ParamSnapshot> ParamManager::snapshot() const noexcept {
    return store_.snapshot();
}

bool ParamManager::initialized() const noexcept {
    return initialized_.load(std::memory_order_acquire);
}

ParamResult ParamManager::load_initial() {
    std::lock_guard<std::mutex> lock(reload_mutex_);

    // 重复调用时按热加载处理，避免第二次启动加载改写不可热加载参数。
    const ParamUpdateOrigin origin = initialized_.load(std::memory_order_acquire)
                                         ? ParamUpdateOrigin::kHotReload
                                         : ParamUpdateOrigin::kInitialLoad;
    ParamResult result = load_and_apply(origin);
    if (result.ok()) {
        initialized_.store(true, std::memory_order_release);
    }
    return result;
}

ParamResult ParamManager::reload() {
    std::lock_guard<std::mutex> lock(reload_mutex_);

    if (!initialized_.load(std::memory_order_acquire)) {
        return ParamResult::failure(ParamError{ParamErrorCode::kNotInitialized, "",
                                               "load_initial() has not succeeded yet"});
    }

    return load_and_apply(ParamUpdateOrigin::kHotReload);
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
