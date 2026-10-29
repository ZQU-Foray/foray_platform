#include "foray/platform/param/source.hpp"

#include <utility>

namespace foray::platform::param {

MapParamSource::MapParamSource(ParamValueMap values) : values_(std::move(values)) {
}

void MapParamSource::set(ParamValueMap values) {
    std::lock_guard<std::mutex> lock(mutex_);
    values_ = std::move(values);
}

std::string MapParamSource::name() const {
    return "memory";
}

ParamResult MapParamSource::load(const ParamSchema& /*schema*/, ParamValueMap& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    out = values_;
    return ParamResult::success();
}

} // namespace foray::platform::param
