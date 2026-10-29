#include "foray/platform/net/nm_backend.hpp"

#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <utility>
#include <vector>

namespace foray::platform::net {
namespace {

std::string trim(std::string text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

/// `nmcli -t` 用反斜杠转义字段内的冒号。
std::vector<std::string> split_escaped(const std::string& line) {
    std::vector<std::string> fields;
    std::string current;
    for (std::size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '\\' && i + 1 < line.size()) {
            current.push_back(line[++i]);
        } else if (line[i] == ':') {
            fields.push_back(current);
            current.clear();
        } else {
            current.push_back(line[i]);
        }
    }
    fields.push_back(current);
    return fields;
}

struct ConnectionRow {
    std::string name;
    std::string uuid;
    std::string type;
    std::string device;
};

Result<std::string> nmcli(ProcessRunner& runner, std::vector<std::string> args) {
    std::vector<std::string> argv{"nmcli", "--colors", "no"};
    argv.insert(argv.end(), std::make_move_iterator(args.begin()),
                std::make_move_iterator(args.end()));
    auto output = runner.run(argv);
    if (!output.ok()) {
        return output.error();
    }
    const ProcessOutput& result = output.value();
    if (result.exit_code != 0) {
        const std::string detail = trim(result.err.empty() ? result.out : result.err);
        return Error{ErrorCode::kInternal, "nmcli 执行失败: " + detail};
    }
    return result.out;
}

Result<std::vector<ConnectionRow>> list_connections(ProcessRunner& runner) {
    auto text = nmcli(runner, {"-t", "-f", "NAME,UUID,TYPE,DEVICE", "connection", "show"});
    if (!text.ok()) {
        return text.error();
    }
    std::vector<ConnectionRow> rows;
    std::istringstream stream(text.value());
    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty()) {
            continue;
        }
        const auto fields = split_escaped(line);
        if (fields.size() < 4) {
            continue;
        }
        rows.push_back({fields[0], fields[1], fields[2], fields[3]});
    }
    return rows;
}

Result<std::string> bridge_connection(ProcessRunner& runner, const std::string& bridge,
                                      bool create_if_missing) {
    auto rows = list_connections(runner);
    if (!rows.ok()) {
        return rows.error();
    }
    for (const auto& row : rows.value()) {
        if (row.type == "bridge" && (row.device == bridge || row.name == bridge)) {
            return row.name;
        }
    }
    if (!create_if_missing) {
        return Error{ErrorCode::kDeviceNotFound, "NetworkManager 中没有桥连接: " + bridge};
    }
    auto added = nmcli(runner, {"connection", "add", "type", "bridge", "ifname", bridge, "con-name",
                                bridge, "autoconnect", "yes"});
    if (!added.ok()) {
        return added.error();
    }
    return bridge;
}

Result<void> activate(ProcessRunner& runner, const std::string& connection) {
    const auto out = nmcli(runner, {"connection", "up", connection});
    return out.ok() ? Result<void>() : Result<void>(out.error());
}

/// 找到接口对应的 NM 连接：桥自身，或本工具创建的 foray-<iface>-slave。
Result<std::string> connection_for(ProcessRunner& runner, const std::string& name) {
    auto rows = list_connections(runner);
    if (!rows.ok()) {
        return rows.error();
    }
    for (const auto& row : rows.value()) {
        if (row.type == "bridge" && (row.device == name || row.name == name)) {
            return row.name;
        }
    }
    const std::string slave_id = "foray-" + name + "-slave";
    for (const auto& row : rows.value()) {
        if (row.type == "bridge-slave" && row.device == name && row.name == slave_id) {
            return row.name;
        }
    }
    for (const auto& row : rows.value()) {
        if (row.type == "bridge-slave" && row.device == name) {
            return row.name;
        }
    }
    return Error{ErrorCode::kDeviceNotFound,
                 "找不到 " + name + " 的 NetworkManager 连接"};
}

/// 用户自建 profile 只停用、不删除；已停用时 nmcli 会报错，这里视为成功。
Result<void> deactivate(ProcessRunner& runner, const std::string& connection) {
    (void)nmcli(runner, {"connection", "down", connection});
    return Result<void>();
}

std::vector<std::string> current_ipv4(ProcessRunner& runner, const std::string& bridge) {
    auto out = nmcli(runner, {"-t", "-f", "IP4.ADDRESS", "device", "show", bridge});
    std::vector<std::string> addresses;
    if (!out.ok()) {
        return addresses;
    }
    std::istringstream stream(out.value());
    std::string line;
    while (std::getline(stream, line)) {
        // 形如 `IP4.ADDRESS[1]:192.168.1.51/24`；-t 模式下取最后一个冒号之后。
        const auto colon = line.find_last_of(':');
        const std::string value = colon == std::string::npos ? line : line.substr(colon + 1);
        if (!value.empty()) {
            addresses.push_back(value);
        }
    }
    return addresses;
}

class NetworkManagerBackend final : public NetBackend {
  public:
    explicit NetworkManagerBackend(ProcessRunner& runner) : runner_(runner) {
    }

    BackendKind kind() const override {
        return BackendKind::kNetworkManager;
    }

    Result<void> ensure_bridge(const std::string& bridge) override {
        auto connection = bridge_connection(runner_, bridge, true);
        if (!connection.ok()) {
            return connection.error();
        }
        auto out = nmcli(runner_, {"connection", "up", connection.value()});
        return out.ok() ? Result<void>() : Result<void>(out.error());
    }

    Result<void> set_slave(const std::string& iface, const std::string& bridge) override {
        auto connection = bridge_connection(runner_, bridge, true);
        if (!connection.ok()) {
            return connection.error();
        }
        const std::string slave_id = "foray-" + iface + "-slave";
        auto rows = list_connections(runner_);
        if (!rows.ok()) {
            return rows.error();
        }
        const auto* existing = [&rows, &slave_id]() -> const ConnectionRow* {
            for (const auto& row : rows.value()) {
                if (row.name == slave_id) {
                    return &row;
                }
            }
            return nullptr;
        }();
        if (existing != nullptr && existing->device == iface) {
            return activate(runner_, slave_id);
        }
        const auto added = nmcli(runner_, {"connection", "add", "type", "bridge-slave", "ifname",
                                           iface, "master", bridge, "con-name", slave_id,
                                           "autoconnect", "yes"});
        if (!added.ok()) {
            return added.error();
        }
        return activate(runner_, slave_id);
    }

    Result<void> clear_slave(const std::string& iface) override {
        auto rows = list_connections(runner_);
        if (!rows.ok()) {
            return rows.error();
        }
        for (const auto& row : rows.value()) {
            if (row.device != iface || row.type != "bridge-slave") {
                continue;
            }
            // 只删除本工具创建的 profile，用户自建的只做 down。
            if (row.name.rfind("foray-", 0) == 0) {
                auto deleted = nmcli(runner_, {"connection", "delete", row.name});
                return deleted.ok() ? Result<void>() : Result<void>(deleted.error());
            }
            return deactivate(runner_, row.name);
        }
        return Result<void>();
    }

    Result<void> set_up(const std::string& name) override {
        auto connection = bridge_connection(runner_, name, false);
        if (!connection.ok()) {
            return connection.error();
        }
        return activate(runner_, connection.value());
    }

    Result<void> set_mtu(const std::string& name, std::uint32_t mtu) override {
        if (mtu < 576 || mtu > 65535) {
            return Error{ErrorCode::kInvalidArgument,
                         "MTU 超出 576–65535: " + std::to_string(mtu)};
        }
        auto connection = connection_for(runner_, name);
        if (!connection.ok()) {
            return connection.error();
        }
        auto modified = nmcli(runner_, {"connection", "modify", connection.value(),
                                        "802-3-ethernet.mtu", std::to_string(mtu)});
        if (!modified.ok()) {
            return Error{modified.error().code, "设置 " + name + " 的 MTU 失败（profile " +
                                                    connection.value() + "）: " +
                                                    modified.error().message};
        }
        return activate(runner_, connection.value());
    }

    Result<void> add_ipv4(const std::string& name, const Ipv4& ip, std::uint8_t prefix) override {
        auto connection = bridge_connection(runner_, name, false);
        if (!connection.ok()) {
            return connection.error();
        }
        const std::string cidr = ip.to_string() + "/" + std::to_string(prefix);
        const auto existing = current_ipv4(runner_, name);
        if (std::find(existing.begin(), existing.end(), cidr) != existing.end()) {
            return Result<void>();
        }
        auto added = nmcli(runner_, {"connection", "modify", connection.value(), "+ipv4.addresses",
                                     cidr});
        if (!added.ok()) {
            return added.error();
        }
        auto manual = nmcli(runner_, {"connection", "modify", connection.value(), "ipv4.method",
                                      "manual"});
        if (!manual.ok()) {
            return manual.error();
        }
        return activate(runner_, connection.value());
    }

    Result<void> remove_ipv4(const std::string& name, const Ipv4& ip, std::uint8_t prefix) override {
        auto connection = bridge_connection(runner_, name, false);
        if (!connection.ok()) {
            return connection.error();
        }
        const std::string cidr = ip.to_string() + "/" + std::to_string(prefix);
        auto removed = nmcli(runner_, {"connection", "modify", connection.value(), "-ipv4.addresses",
                                       cidr});
        if (!removed.ok()) {
            return removed.error();
        }
        return activate(runner_, connection.value());
    }

  private:
    ProcessRunner& runner_;
};

} // namespace

Result<bool> network_manager_available(ProcessRunner& runner) {
    auto version = runner.run({"nmcli", "--version"});
    if (!version.ok() || version.value().exit_code != 0) {
        return false;
    }
    auto state = runner.run({"nmcli", "-t", "-f", "RUNNING", "general"});
    return state.ok() && state.value().exit_code == 0 && trim(state.value().out) == "running";
}

Result<std::unique_ptr<NetBackend>> make_network_manager_backend(ProcessRunner& runner) {
    auto available = network_manager_available(runner);
    if (!available.ok()) {
        return available.error();
    }
    if (!available.value()) {
        return Error{ErrorCode::kNotSupported, "NetworkManager 不可用"};
    }
    return std::unique_ptr<NetBackend>(new NetworkManagerBackend(runner));
}

} // namespace foray::platform::net
