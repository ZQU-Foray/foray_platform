#include "foray/platform/param/yaml_source.hpp"

#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace foray::platform::param {

namespace {

ParamError make_error(ParamErrorCode code, std::string name, std::string message) {
    return ParamError{code, std::move(name), std::move(message)};
}

/**
 * @brief 把嵌套映射展开为点分名称的叶子列表。
 */
void flatten_node(const YAML::Node& node, const std::string& prefix,
                  std::vector<std::pair<std::string, YAML::Node>>& leaves,
                  std::vector<ParamError>& errors, const std::string& source_name) {
    if (node.IsMap()) {
        for (const auto& item : node) {
            const YAML::Node& key = item.first;
            if (!key.IsScalar() || key.Scalar().empty()) {
                errors.push_back(make_error(ParamErrorCode::kSourceFailure, source_name,
                                            "mapping keys must be non-empty scalars"));
                continue;
            }

            const std::string key_text = key.Scalar();
            const std::string full_name = prefix.empty() ? key_text : prefix + "." + key_text;
            flatten_node(item.second, full_name, leaves, errors, source_name);
        }
        return;
    }

    if (node.IsSequence()) {
        errors.push_back(make_error(ParamErrorCode::kTypeMismatch,
                                    prefix.empty() ? source_name : prefix,
                                    "sequences are not supported"));
        return;
    }

    if (node.IsNull()) {
        errors.push_back(make_error(ParamErrorCode::kTypeMismatch,
                                    prefix.empty() ? source_name : prefix,
                                    "null values are not supported"));
        return;
    }

    if (!node.IsScalar()) {
        errors.push_back(make_error(ParamErrorCode::kTypeMismatch,
                                    prefix.empty() ? source_name : prefix,
                                    "unsupported YAML node"));
        return;
    }

    leaves.emplace_back(prefix, node);
}

ParamResult parse_scalar(const ParamSpec& spec, const YAML::Node& node, ParamValue& out) {
    const std::string text = node.Scalar();

    switch (spec.type) {
    case ParamType::kBool: {
        if (text == "true") {
            out = ParamValue(true);
            return ParamResult::success();
        }
        if (text == "false") {
            out = ParamValue(false);
            return ParamResult::success();
        }
        return ParamResult::failure(
            make_error(ParamErrorCode::kTypeMismatch, spec.name,
                       "expected bool literal 'true' or 'false', got '" + text + "'"));
    }

    case ParamType::kInt: {
        std::int64_t value = 0;
        const char* begin = text.data();
        const char* end = begin + text.size();
        const auto [parsed_to, error] = std::from_chars(begin, end, value);
        if (error != std::errc() || parsed_to != end) {
            return ParamResult::failure(make_error(ParamErrorCode::kTypeMismatch, spec.name,
                                                   "expected integer, got '" + text + "'"));
        }
        out = ParamValue(value);
        return ParamResult::success();
    }

    case ParamType::kDouble: {
        if (text.empty()) {
            return ParamResult::failure(
                make_error(ParamErrorCode::kTypeMismatch, spec.name, "empty double value"));
        }
        char* parsed_to = nullptr;
        errno = 0;
        const double value = std::strtod(text.c_str(), &parsed_to);
        if (parsed_to != text.c_str() + text.size() || errno == ERANGE || !std::isfinite(value)) {
            return ParamResult::failure(make_error(ParamErrorCode::kTypeMismatch, spec.name,
                                                   "expected finite number, got '" + text + "'"));
        }
        out = ParamValue(value);
        return ParamResult::success();
    }

    case ParamType::kString:
        out = ParamValue(text);
        return ParamResult::success();
    }

    return ParamResult::failure(
        make_error(ParamErrorCode::kTypeMismatch, spec.name, "unsupported parameter type"));
}

} // namespace

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
        return ParamResult::failure(
            make_error(ParamErrorCode::kSourceFailure, path_, "cannot open YAML file"));
    } catch (const YAML::Exception& error) {
        return ParamResult::failure(
            make_error(ParamErrorCode::kSourceFailure, path_, error.what()));
    }

    if (root.IsNull()) {
        // 空文件表示不覆盖任何参数。
        out.clear();
        return ParamResult::success();
    }

    if (!root.IsMap()) {
        return ParamResult::failure(make_error(ParamErrorCode::kTypeMismatch, path_,
                                               "root of the YAML document must be a mapping"));
    }

    std::vector<std::pair<std::string, YAML::Node>> leaves;
    std::vector<ParamError> errors;
    flatten_node(root, "", leaves, errors, path_);
    if (!errors.empty()) {
        return ParamResult::failure(std::move(errors));
    }

    ParamValueMap parsed;
    for (const auto& [name, node] : leaves) {
        const ParamSpec* spec = schema.find(name);
        if (spec == nullptr) {
            errors.push_back(make_error(ParamErrorCode::kUnknownParam, name,
                                        "parameter is not declared in schema"));
            continue;
        }

        ParamValue value;
        ParamResult parse = parse_scalar(*spec, node, value);
        if (!parse.ok()) {
            errors.insert(errors.end(), parse.errors().begin(), parse.errors().end());
            continue;
        }

        const auto [position, inserted] = parsed.emplace(name, std::move(value));
        (void)position;
        if (!inserted) {
            errors.push_back(make_error(ParamErrorCode::kDuplicateParam, name,
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
