#include "foray/platform/param/store.hpp"

#include <atomic>
#include <utility>
#include <vector>

#include "validation.hpp"

namespace foray::platform::param {

ParamStore::ParamStore(ParamSchema schema) : schema_(std::move(schema)) {
    ParamValueMap defaults;
    for (const auto& spec : schema_.specs()) {
        defaults.emplace(spec.name, spec.default_value);
    }
    snapshot_ = std::make_shared<const ParamSnapshot>(std::move(defaults), 0);
}

std::shared_ptr<const ParamSnapshot> ParamStore::snapshot() const noexcept {
    return std::atomic_load_explicit(&snapshot_, std::memory_order_acquire);
}

ParamResult ParamStore::apply(const ParamValueMap& updates, ParamUpdateOrigin origin) {
    std::lock_guard<std::mutex> lock(write_mutex_);

    const std::shared_ptr<const ParamSnapshot> current =
        std::atomic_load_explicit(&snapshot_, std::memory_order_acquire);
    ParamValueMap values = current->values();

    std::vector<ParamError> errors;
    bool changed = false;

    for (const auto& [name, raw_value] : updates) {
        const ParamSpec* spec = schema_.find(name);
        if (spec == nullptr) {
            errors.push_back(ParamError{ParamErrorCode::kUnknownParam, name,
                                        "parameter is not declared in schema"});
            continue;
        }

        ParamResult check = detail::check_value(*spec, raw_value);
        if (!check.ok()) {
            errors.insert(errors.end(), check.errors().begin(), check.errors().end());
            continue;
        }

        const ParamValue value = detail::coerce(*spec, raw_value);
        const auto current_value = values.find(name);
        const bool value_changed = current_value == values.end() || current_value->second != value;

        if (value_changed && origin == ParamUpdateOrigin::kHotReload && !spec->hot_reloadable) {
            errors.push_back(ParamError{ParamErrorCode::kNotHotReloadable, name,
                                        "parameter cannot be changed by hot reload"});
            continue;
        }

        if (value_changed) {
            values[name] = value;
            changed = true;
        }
    }

    if (!errors.empty()) {
        // 整批回滚：不替换快照。
        return ParamResult::failure(std::move(errors));
    }

    if (!changed) {
        return ParamResult::success();
    }

    auto next = std::make_shared<const ParamSnapshot>(std::move(values), current->generation() + 1);
    std::atomic_store_explicit(&snapshot_, std::move(next), std::memory_order_release);
    return ParamResult::success();
}

} // namespace foray::platform::param
