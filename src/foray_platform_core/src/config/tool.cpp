#include "foray/platform/config/tool.hpp"

#include <utility>

namespace foray::platform::config {

ConfigTool::Validation ConfigTool::validate(const param::ParamSchema& schema,
                                            const ConfigDocument& document,
                                            const EnvProvider& env) {
    ConfigResolver resolver(schema);
    Validation validation;
    validation.result = resolver.load(document, env, validation.snapshot);
    return validation;
}

std::string ConfigTool::dump(const ConfigSnapshot& snapshot) {
    return ConfigExporter::dump(snapshot);
}

std::string ConfigTool::schema_markdown(const param::ParamSchema& schema) {
    return ConfigExporter::schema_markdown(schema);
}

std::string ConfigTool::schema_json(const param::ParamSchema& schema) {
    return ConfigExporter::schema_json(schema);
}

} // namespace foray::platform::config
