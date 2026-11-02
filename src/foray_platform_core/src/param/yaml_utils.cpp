#include "yaml_utils.hpp"

#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <string>
#include <system_error>
#include <utility>

namespace foray::platform::param::detail {

ParamError make_yaml_error(ParamErrorCode code, std::string name, std::string message) {
    return ParamError{code, std::move(name), std::move(message)};
}

void flatten_yaml_node(const YAML::Node& node, const std::string& prefix,
                       std::vector<std::pair<std::string, YAML::Node>>& leaves,
                       std::vector<ParamError>& errors, const std::string& source_name) {
    if (node.IsMap()) {
        for (const auto& item : node) {
            const YAML::Node& key = item.first;
            if (!key.IsScalar() || key.Scalar().empty()) {
                errors.push_back(make_yaml_error(ParamErrorCode::kSourceFailure, source_name,
                                                 "mapping keys must be non-empty scalars"));
                continue;
            }

            const std::string key_text = key.Scalar();
            const std::string full_name = prefix.empty() ? key_text : prefix + "." + key_text;
            flatten_yaml_node(item.second, full_name, leaves, errors, source_name);
        }
        return;
    }

    if (node.IsSequence()) {
        errors.push_back(make_yaml_error(ParamErrorCode::kTypeMismatch,
                                         prefix.empty() ? source_name : prefix,
                                         "sequences are not supported"));
        return;
    }

    if (node.IsNull()) {
        errors.push_back(make_yaml_error(ParamErrorCode::kTypeMismatch,
                                         prefix.empty() ? source_name : prefix,
                                         "null values are not supported"));
        return;
    }

    if (!node.IsScalar()) {
        errors.push_back(make_yaml_error(ParamErrorCode::kTypeMismatch,
                                         prefix.empty() ? source_name : prefix,
                                         "unsupported YAML node"));
        return;
    }

    leaves.emplace_back(prefix, node);
}

ParamResult parse_scalar_text(const ParamSpec& spec, std::string_view text, ParamValue& out) {
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
        return ParamResult::failure(make_yaml_error(
            ParamErrorCode::kTypeMismatch, spec.name,
            "expected bool literal 'true' or 'false', got '" + std::string(text) + "'"));
    }

    case ParamType::kInt: {
        std::int64_t value = 0;
        const char* begin = text.data();
        const char* end = begin + text.size();
        const auto [parsed_to, error] = std::from_chars(begin, end, value);
        if (error != std::errc() || parsed_to != end) {
            return ParamResult::failure(
                make_yaml_error(ParamErrorCode::kTypeMismatch, spec.name,
                                "expected integer, got '" + std::string(text) + "'"));
        }
        out = ParamValue(value);
        return ParamResult::success();
    }

    case ParamType::kDouble: {
        if (text.empty()) {
            return ParamResult::failure(
                make_yaml_error(ParamErrorCode::kTypeMismatch, spec.name, "empty double value"));
        }
        std::string owned(text);
        char* parsed_to = nullptr;
        errno = 0;
        const double value = std::strtod(owned.c_str(), &parsed_to);
        if (parsed_to != owned.c_str() + owned.size() || errno == ERANGE || !std::isfinite(value)) {
            return ParamResult::failure(
                make_yaml_error(ParamErrorCode::kTypeMismatch, spec.name,
                                "expected finite number, got '" + owned + "'"));
        }
        out = ParamValue(value);
        return ParamResult::success();
    }

    case ParamType::kString:
        out = ParamValue(std::string(text));
        return ParamResult::success();
    }

    return ParamResult::failure(
        make_yaml_error(ParamErrorCode::kTypeMismatch, spec.name, "unsupported parameter type"));
}

} // namespace foray::platform::param::detail
