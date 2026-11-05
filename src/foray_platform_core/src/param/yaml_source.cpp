#include "foray/platform/param/yaml_source.hpp"

#include <yaml-cpp/yaml.h>

#include <string>
#include <utility>
#include <vector>

#include "yaml_utils.hpp"

namespace foray::platform::param {

YamlFileParamSource::YamlFileParamSource(std::string path) : path_(std::move(path)) {
}

std::string YamlFileParamSource::name() const {
    return path_;
}

ParamResult YamlFileParamSource::load(const ParamSchema& schema, ParamValueMap& out) const {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path_);
    } catch (const YAML::BadFile&) {
        return ParamResult::failure(detail::make_yaml_error(ParamErrorCode::kSourceFailure, path_,
                                                            "cannot open YAML file"));
    } catch (const YAML::Exception& error) {
        return ParamResult::failure(
            detail::make_yaml_error(ParamErrorCode::kSourceFailure, path_, error.what()));
    }

    if (root.IsNull()) {
        // 空文件表示不覆盖任何参数。
        out.clear();
        return ParamResult::success();
    }

    if (!root.IsMap()) {
        return ParamResult::failure(detail::make_yaml_error(
            ParamErrorCode::kTypeMismatch, path_, "root of the YAML document must be a mapping"));
    }

    std::vector<std::pair<std::string, YAML::Node>> leaves;
    std::vector<ParamError> errors;
    detail::flatten_yaml_node(root, "", leaves, errors, path_);
    if (!errors.empty()) {
        return ParamResult::failure(std::move(errors));
    }

    ParamValueMap parsed;
    for (const auto& [name, node] : leaves) {
        const ParamSpec* spec = schema.find(name);
        if (spec == nullptr) {
            errors.push_back(detail::make_yaml_error(ParamErrorCode::kUnknownParam, name,
                                                     "parameter is not declared in schema"));
            continue;
        }

        ParamValue value;
        ParamResult parse = detail::parse_scalar_text(*spec, node.Scalar(), value);
        if (!parse.ok()) {
            errors.insert(errors.end(), parse.errors().begin(), parse.errors().end());
            continue;
        }

        const auto [position, inserted] = parsed.emplace(name, std::move(value));
        (void)position;
        if (!inserted) {
            errors.push_back(detail::make_yaml_error(ParamErrorCode::kDuplicateParam, name,
                                                     "parameter appears more than once"));
        }
    }

    if (!errors.empty()) {
        return ParamResult::failure(std::move(errors));
    }

    out = std::move(parsed);
    return ParamResult::success();
}

} // namespace foray::platform::param
