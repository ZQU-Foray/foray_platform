#include "foray/platform/param/snapshot.hpp"

#include <utility>

namespace foray::platform::param {

ParamSnapshot::ParamSnapshot(ParamValueMap values, std::uint64_t generation)
    : values_(std::move(values)), generation_(generation) {
}

const ParamValue* ParamSnapshot::find(std::string_view name) const noexcept {
    const auto found = values_.find(name);
    if (found == values_.end()) {
        return nullptr;
    }
    return &found->second;
}

bool ParamSnapshot::get_bool(std::string_view name, bool default_value) const noexcept {
    const ParamValue* value = find(name);
    if (value == nullptr || value->type() != ParamType::kBool) {
        return default_value;
    }
    return value->as_bool();
}

std::int64_t ParamSnapshot::get_int(std::string_view name,
                                    std::int64_t default_value) const noexcept {
    const ParamValue* value = find(name);
    if (value == nullptr || value->type() != ParamType::kInt) {
        return default_value;
    }
    return value->as_int();
}

double ParamSnapshot::get_double(std::string_view name, double default_value) const noexcept {
    const ParamValue* value = find(name);
    if (value == nullptr || !value->is_numeric()) {
        return default_value;
    }
    return value->as_double();
}

std::string ParamSnapshot::get_string(std::string_view name, std::string default_value) const {
    const ParamValue* value = find(name);
    if (value == nullptr || value->type() != ParamType::kString) {
        return default_value;
    }
    return value->as_string();
}

} // namespace foray::platform::param
