#include "foray/platform/config/exporter.hpp"

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace foray::platform::config {

namespace {

std::string escape_json(std::string_view text) {
    std::string escaped;
    escaped.reserve(text.size());

    for (const char value : text) {
        switch (value) {
        case '"':
            escaped += "\\\"";
            break;
        case '\\':
            escaped += "\\\\";
            break;
        case '\n':
            escaped += "\\n";
            break;
        case '\r':
            escaped += "\\r";
            break;
        case '\t':
            escaped += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(value) < 0x20) {
                constexpr char kHex[] = "0123456789abcdef";
                escaped += "\\u00";
                escaped.push_back(kHex[(static_cast<unsigned char>(value) >> 4) & 0x0F]);
                escaped.push_back(kHex[static_cast<unsigned char>(value) & 0x0F]);
            } else {
                escaped.push_back(value);
            }
            break;
        }
    }
    return escaped;
}

std::string escape_markdown(std::string_view text) {
    std::string escaped;
    escaped.reserve(text.size());
    for (const char value : text) {
        if (value == '|') {
            escaped += "\\|";
        } else if (value == '\n' || value == '\r') {
            escaped.push_back(' ');
        } else {
            escaped.push_back(value);
        }
    }
    return escaped;
}

std::string quote_string(std::string_view text) {
    return "\"" + escape_json(text) + "\"";
}

std::string format_value(const param::ParamValue& value) {
    if (value.type() == param::ParamType::kString) {
        return quote_string(value.as_string());
    }
    return value.to_string();
}

std::string json_value(const param::ParamValue& value) {
    return format_value(value);
}

std::string json_type(param::ParamType type) {
    switch (type) {
    case param::ParamType::kBool:
        return "boolean";
    case param::ParamType::kInt:
        return "integer";
    case param::ParamType::kDouble:
        return "number";
    case param::ParamType::kString:
        return "string";
    }
    return "string";
}

std::string describe(const param::ParamSpec& spec) {
    std::string description = spec.description;
    if (!spec.unit.empty()) {
        if (!description.empty()) {
            description += " ";
        }
        description += "[" + spec.unit + "]";
    }
    if (!spec.source_note.empty()) {
        if (!description.empty()) {
            description += " ";
        }
        description += "(" + spec.source_note + ")";
    }
    return description;
}

std::string format_range(const param::ParamSpec& spec) {
    if (spec.minimum.has_value() && spec.maximum.has_value()) {
        return "[" + spec.minimum->to_string() + ", " + spec.maximum->to_string() + "]";
    }
    if (spec.minimum.has_value()) {
        return ">= " + spec.minimum->to_string();
    }
    if (spec.maximum.has_value()) {
        return "<= " + spec.maximum->to_string();
    }
    return "";
}

std::string format_allowed(const param::ParamSpec& spec) {
    std::string allowed;
    for (std::size_t index = 0; index < spec.allowed_values.size(); ++index) {
        if (index != 0) {
            allowed += ", ";
        }
        allowed += format_value(spec.allowed_values[index]);
    }
    return allowed;
}

} // namespace

std::string ConfigExporter::dump(const ConfigSnapshot& snapshot) {
    std::string output;
    for (const auto& [name, value] : snapshot.values) {
        output += name;
        output += ": ";
        output += format_value(value);
        if (const std::string* source = snapshot.source(name); source != nullptr) {
            output += "  # ";
            output += *source;
        }
        output.push_back('\n');
    }
    return output;
}

std::string ConfigExporter::schema_markdown(const param::ParamSchema& schema) {
    std::string output;
    output += "| 名称 | 类型 | 默认值 | 范围 | 枚举 | 热加载 | 说明 |\n";
    output += "|---|---|---|---|---|---|---|\n";

    for (const auto& spec : schema.specs()) {
        output += "| `";
        output += escape_markdown(spec.name);
        output += "` | ";
        output += param::to_string(spec.type);
        output += " | ";
        output += escape_markdown(format_value(spec.default_value));
        output += " | ";
        output += escape_markdown(format_range(spec));
        output += " | ";
        output += escape_markdown(format_allowed(spec));
        output += " | ";
        output += spec.hot_reloadable ? "是" : "否";
        output += " | ";
        output += escape_markdown(describe(spec));
        output += " |\n";
    }
    return output;
}

std::string ConfigExporter::schema_json(const param::ParamSchema& schema) {
    std::vector<std::string> properties;
    properties.reserve(schema.specs().size());

    for (const auto& spec : schema.specs()) {
        std::string property;
        property += "    \"";
        property += escape_json(spec.name);
        property += "\": {\n";
        property += "      \"type\": \"";
        property += json_type(spec.type);
        property += "\"";
        property += ",\n      \"default\": ";
        property += json_value(spec.default_value);

        if (spec.minimum.has_value()) {
            property += ",\n      \"minimum\": ";
            property += json_value(*spec.minimum);
        }
        if (spec.maximum.has_value()) {
            property += ",\n      \"maximum\": ";
            property += json_value(*spec.maximum);
        }
        if (!spec.allowed_values.empty()) {
            property += ",\n      \"enum\": [";
            for (std::size_t index = 0; index < spec.allowed_values.size(); ++index) {
                if (index != 0) {
                    property += ", ";
                }
                property += json_value(spec.allowed_values[index]);
            }
            property += "]";
        }

        const std::string description = describe(spec);
        if (!description.empty()) {
            property += ",\n      \"description\": \"";
            property += escape_json(description);
            property += "\"";
        }

        property += "\n    }";
        properties.push_back(std::move(property));
    }

    std::string output;
    output += "{\n";
    output += "  \"$schema\": \"https://json-schema.org/draft/2020-12/schema\",\n";
    output += "  \"title\": \"foray_platform configuration\",\n";
    output += "  \"type\": \"object\",\n";
    output += "  \"additionalProperties\": false,\n";
    output += "  \"properties\": {\n";

    for (std::size_t index = 0; index < properties.size(); ++index) {
        output += properties[index];
        if (index + 1 != properties.size()) {
            output += ",";
        }
        output.push_back('\n');
    }

    output += "  }\n";
    output += "}\n";
    return output;
}

} // namespace foray::platform::config
