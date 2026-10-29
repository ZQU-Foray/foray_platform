#include "foray/platform/config/resolver.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "foray/platform/param/store.hpp"
#include "param/yaml_utils.hpp"

namespace foray::platform::config {

namespace {

namespace fs = std::filesystem;

std::string normalize_env_key(std::string_view name) {
    std::string normalized;
    normalized.reserve(name.size());
    for (const char value : name) {
        if (value >= 'a' && value <= 'z') {
            normalized.push_back(static_cast<char>(value - 'a' + 'A'));
        } else if ((value >= 'A' && value <= 'Z') || (value >= '0' && value <= '9')) {
            normalized.push_back(value);
        } else {
            normalized.push_back('_');
        }
    }
    return normalized;
}

fs::path canonical_path(const fs::path& path) {
    std::error_code error;
    const fs::path canonical = fs::weakly_canonical(path, error);
    return error ? path : canonical;
}

fs::path resolve_include_path(const fs::path& including_file, const std::string& include) {
    fs::path resolved(include);
    if (resolved.is_relative()) {
        resolved = including_file.parent_path() / resolved;
    }
    return canonical_path(resolved);
}

std::optional<std::string> substitute_env(std::string_view text, const EnvProvider& env,
                                          const std::string& parameter_name,
                                          std::vector<param::ParamError>& errors) {
    std::string result;
    result.reserve(text.size());

    std::size_t position = 0;
    while (position < text.size()) {
        const std::size_t start = text.find("${", position);
        if (start == std::string_view::npos) {
            result.append(text.substr(position));
            break;
        }

        result.append(text.substr(position, start - position));

        const std::size_t end = text.find('}', start + 2);
        if (end == std::string_view::npos) {
            errors.push_back(param::detail::make_yaml_error(
                param::ParamErrorCode::kSourceFailure, parameter_name,
                "unterminated ${...} environment reference"));
            return std::nullopt;
        }

        const std::string_view variable = text.substr(start + 2, end - (start + 2));
        if (variable.empty()) {
            errors.push_back(param::detail::make_yaml_error(param::ParamErrorCode::kSourceFailure,
                                                            parameter_name,
                                                            "empty ${} environment reference"));
            return std::nullopt;
        }

        const std::optional<std::string> value = env.get(variable);
        if (!value.has_value()) {
            errors.push_back(param::detail::make_yaml_error(
                param::ParamErrorCode::kSourceFailure, parameter_name,
                "environment variable '" + std::string(variable) + "' is not defined"));
            return std::nullopt;
        }

        result.append(*value);
        position = end + 1;
    }

    return result;
}

class StackGuard {
  public:
    StackGuard(std::set<std::string>& stack, std::string key)
        : stack_(stack), key_(std::move(key)) {
    }

    ~StackGuard() {
        stack_.erase(key_);
    }

    StackGuard(const StackGuard&) = delete;
    StackGuard& operator=(const StackGuard&) = delete;

  private:
    std::set<std::string>& stack_;
    std::string key_;
};

void load_file(const fs::path& file_path, const param::ParamSchema& schema, const EnvProvider& env,
               param::ParamValueMap& values,
               std::map<std::string, std::string, std::less<>>& provenance,
               std::set<std::string>& stack, std::vector<param::ParamError>& errors) {
    const fs::path canonical = canonical_path(file_path);
    const std::string source = canonical.string();

    if (stack.count(source) != 0) {
        errors.push_back(param::detail::make_yaml_error(param::ParamErrorCode::kSourceFailure,
                                                        source, "include cycle detected"));
        return;
    }

    std::error_code exists_error;
    if (!fs::is_regular_file(canonical, exists_error) || exists_error) {
        errors.push_back(param::detail::make_yaml_error(param::ParamErrorCode::kSourceFailure,
                                                        source, "config file does not exist"));
        return;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(canonical.string());
    } catch (const YAML::Exception& error) {
        errors.push_back(param::detail::make_yaml_error(param::ParamErrorCode::kSourceFailure,
                                                        source, error.what()));
        return;
    }

    if (root.IsNull()) {
        return;
    }

    if (!root.IsMap()) {
        errors.push_back(
            param::detail::make_yaml_error(param::ParamErrorCode::kTypeMismatch, source,
                                           "root of the YAML document must be a mapping"));
        return;
    }

    stack.insert(source);
    const StackGuard guard(stack, source);

    if (root["include"]) {
        const YAML::Node include = root["include"];
        std::vector<std::string> includes;

        if (include.IsScalar()) {
            includes.push_back(include.Scalar());
        } else if (include.IsSequence()) {
            for (const auto& entry : include) {
                if (!entry.IsScalar()) {
                    errors.push_back(param::detail::make_yaml_error(
                        param::ParamErrorCode::kSourceFailure, source,
                        "include entries must be scalar paths"));
                    continue;
                }
                includes.push_back(entry.Scalar());
            }
        } else {
            errors.push_back(param::detail::make_yaml_error(
                param::ParamErrorCode::kSourceFailure, source,
                "include must be a scalar path or a sequence of paths"));
        }

        for (const auto& include_path : includes) {
            if (include_path.empty()) {
                errors.push_back(param::detail::make_yaml_error(
                    param::ParamErrorCode::kSourceFailure, source, "include path is empty"));
                continue;
            }
            load_file(resolve_include_path(canonical, include_path), schema, env, values,
                      provenance, stack, errors);
        }
    }

    std::set<std::string> seen_in_file;
    for (const auto& item : root) {
        const YAML::Node& key_node = item.first;
        if (!key_node.IsScalar() || key_node.Scalar().empty()) {
            errors.push_back(
                param::detail::make_yaml_error(param::ParamErrorCode::kSourceFailure, source,
                                               "mapping keys must be non-empty scalars"));
            continue;
        }

        const std::string key = key_node.Scalar();
        if (key == "include") {
            continue;
        }

        std::vector<std::pair<std::string, YAML::Node>> leaves;
        param::detail::flatten_yaml_node(item.second, key, leaves, errors, source);

        for (const auto& [name, node] : leaves) {
            const param::ParamSpec* spec = schema.find(name);
            if (spec == nullptr) {
                errors.push_back(
                    param::detail::make_yaml_error(param::ParamErrorCode::kUnknownParam, name,
                                                   "parameter is not declared in schema"));
                continue;
            }

            if (!seen_in_file.insert(name).second) {
                errors.push_back(param::detail::make_yaml_error(
                    param::ParamErrorCode::kDuplicateParam, name,
                    "parameter appears more than once in the same file"));
                continue;
            }

            const std::optional<std::string> substituted =
                substitute_env(node.Scalar(), env, name, errors);
            if (!substituted.has_value()) {
                continue;
            }

            param::ParamValue value;
            param::ParamResult parsed =
                param::detail::parse_scalar_text(*spec, *substituted, value);
            if (!parsed.ok()) {
                errors.insert(errors.end(), parsed.errors().begin(), parsed.errors().end());
                continue;
            }

            values[name] = value;
            provenance[name] = source;
        }
    }
}

void apply_env_layer(const param::ParamSchema& schema, const EnvProvider& env,
                     const std::string& prefix, param::ParamValueMap& values,
                     std::map<std::string, std::string, std::less<>>& provenance,
                     std::vector<param::ParamError>& errors) {
    for (const auto& spec : schema.specs()) {
        const std::string variable = prefix + normalize_env_key(spec.name);
        const std::optional<std::string> raw = env.get(variable);
        if (!raw.has_value()) {
            continue;
        }

        param::ParamValue value;
        param::ParamResult parsed = param::detail::parse_scalar_text(spec, *raw, value);
        if (!parsed.ok()) {
            for (auto error : parsed.errors()) {
                error.message = variable + ": " + error.message;
                errors.push_back(std::move(error));
            }
            continue;
        }

        values[spec.name] = value;
        provenance[spec.name] = "env:" + variable;
    }
}

} // namespace

std::optional<std::string> ProcessEnvProvider::get(std::string_view name) const {
    const std::string owned(name);
    const char* value = std::getenv(owned.c_str());
    if (value == nullptr) {
        return std::nullopt;
    }
    return std::string(value);
}

MapEnvProvider::MapEnvProvider(std::map<std::string, std::string, std::less<>> values)
    : values_(std::move(values)) {
}

void MapEnvProvider::set(std::string name, std::string value) {
    values_[std::move(name)] = std::move(value);
}

std::optional<std::string> MapEnvProvider::get(std::string_view name) const {
    const auto found = values_.find(name);
    if (found == values_.end()) {
        return std::nullopt;
    }
    return found->second;
}

ConfigResolver::ConfigResolver(param::ParamSchema schema) : schema_(std::move(schema)) {
}

param::ParamResult ConfigResolver::load(const ConfigDocument& document, ConfigSnapshot& out) const {
    const ProcessEnvProvider env;
    return load(document, env, out);
}

param::ParamResult ConfigResolver::load(const ConfigDocument& document, const EnvProvider& env,
                                        ConfigSnapshot& out) const {
    out = ConfigSnapshot{};

    std::vector<param::ParamError> errors;
    for (const auto& layer : document.layers) {
        if (layer.path.empty()) {
            continue;
        }

        std::error_code exists_error;
        const bool exists = fs::is_regular_file(layer.path, exists_error);
        if (exists_error || !exists) {
            if (layer.required) {
                errors.push_back(param::detail::make_yaml_error(
                    param::ParamErrorCode::kSourceFailure, layer.name,
                    "required config file not found: " + layer.path.string()));
            }
            continue;
        }

        std::set<std::string> stack;
        load_file(layer.path, schema_, env, out.values, out.provenance, stack, errors);
    }

    if (!errors.empty()) {
        return param::ParamResult::failure(std::move(errors));
    }

    if (document.enable_env_layer) {
        apply_env_layer(schema_, env, document.env_prefix, out.values, out.provenance, errors);
        if (!errors.empty()) {
            return param::ParamResult::failure(std::move(errors));
        }
    }

    // 复用参数存储完成类型/范围/枚举校验，并做 int -> double 宽化。
    param::ParamStore store(schema_);
    param::ParamResult applied = store.apply(out.values, param::ParamUpdateOrigin::kInitialLoad);
    if (!applied.ok()) {
        return applied;
    }

    out.values = store.snapshot()->values();
    return param::ParamResult::success();
}

} // namespace foray::platform::config
