#include "foray/platform/param/schema.hpp"

#include <utility>

#include "validation.hpp"

namespace foray::platform::param {

ParamResult ParamSchema::add(ParamSpec spec) {
    if (index_.find(spec.name) != index_.end()) {
        return ParamResult::failure(
            ParamError{ParamErrorCode::kDuplicateParam, spec.name, "parameter is already defined"});
    }

    ParamResult validation = detail::validate_spec(spec);
    if (!validation.ok()) {
        return validation;
    }

    const std::size_t position = specs_.size();
    index_.emplace(spec.name, position);
    specs_.push_back(std::move(spec));
    return ParamResult::success();
}

const ParamSpec* ParamSchema::find(std::string_view name) const noexcept {
    const auto found = index_.find(name);
    if (found == index_.end()) {
        return nullptr;
    }
    return &specs_[found->second];
}

} // namespace foray::platform::param
