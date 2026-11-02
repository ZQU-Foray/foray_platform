#include "livox/config_json.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/stat.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace foray::platform::driver::livox::detail {
namespace {

// Livox-SDK2 v1.3.1 sample 默认端口。
constexpr std::uint16_t kLidarCmdPort = 56100;
constexpr std::uint16_t kLidarPushMsgPort = 56200;
constexpr std::uint16_t kLidarPointPort = 56300;
constexpr std::uint16_t kLidarImuPort = 56400;
constexpr std::uint16_t kLidarLogPort = 56500;

constexpr std::uint16_t kHostCmdPort = 56101;
constexpr std::uint16_t kHostPushMsgPort = 56201;
constexpr std::uint16_t kHostPointPort = 56301;
constexpr std::uint16_t kHostImuPort = 56401;
constexpr std::uint16_t kHostLogPort = 56501;

constexpr mode_t kPrivateFileMode = 0600;

std::string sanitize(const std::string& ip) {
    std::string result = ip;
    for (char& character : result) {
        if (character == '.') {
            character = '_';
        }
    }
    return result;
}

} // namespace

bool is_valid_ipv4(const std::string& ip) noexcept {
    unsigned a = 0;
    unsigned b = 0;
    unsigned c = 0;
    unsigned d = 0;
    const int parsed = std::sscanf(ip.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d);
    return parsed == 4 && a <= 255 && b <= 255 && c <= 255 && d <= 255;
}

bool is_local_ipv4(const std::string& ip) noexcept {
    if (!is_valid_ipv4(ip)) {
        return false;
    }

    ifaddrs* addresses = nullptr;
    if (getifaddrs(&addresses) != 0) {
        // 无法枚举网卡时不阻塞启动，交由 SDK 报告实际绑定错误。
        return true;
    }

    bool found = false;
    for (ifaddrs* entry = addresses; entry != nullptr; entry = entry->ifa_next) {
        if (entry->ifa_addr == nullptr || entry->ifa_addr->sa_family != AF_INET) {
            continue;
        }

        char host[NI_MAXHOST] = {};
        const socklen_t length = sizeof(sockaddr_in);
        if (getnameinfo(entry->ifa_addr, length, host, sizeof(host), nullptr, 0, NI_NUMERICHOST) !=
            0) {
            continue;
        }
        if (ip == host) {
            found = true;
            break;
        }
    }

    freeifaddrs(addresses);
    return found;
}

std::string build_config_json(const std::string& host_ip) {
    std::ostringstream json;
    json << "{\n";

    // SDK v1.3.1 用 "MID360" 表示 Mid-360（dev_type 9），
    // 用大小写敏感的 "Mid360s" 表示 Mid-360s（dev_type 35）。
    // 两个段都写入同一份网络配置：设备只匹配自己的类型，多余段会被忽略。
    const auto append_section = [&json, &host_ip](const char* key) {
        json << "  \"" << key << "\": {\n";
        json << "    \"lidar_net_info\": {\n";
        json << "      \"cmd_data_port\": " << kLidarCmdPort << ",\n";
        json << "      \"push_msg_port\": " << kLidarPushMsgPort << ",\n";
        json << "      \"point_data_port\": " << kLidarPointPort << ",\n";
        json << "      \"imu_data_port\": " << kLidarImuPort << ",\n";
        json << "      \"log_data_port\": " << kLidarLogPort << "\n";
        json << "    },\n";
        json << "    \"host_net_info\": [\n";
        json << "      {\n";
        json << "        \"host_ip\": \"" << host_ip << "\",\n";
        json << "        \"cmd_data_port\": " << kHostCmdPort << ",\n";
        json << "        \"push_msg_port\": " << kHostPushMsgPort << ",\n";
        json << "        \"point_data_port\": " << kHostPointPort << ",\n";
        json << "        \"imu_data_port\": " << kHostImuPort << ",\n";
        json << "        \"log_data_port\": " << kHostLogPort << "\n";
        json << "      }\n";
        json << "    ]\n";
        json << "  }";
    };

    append_section("MID360");
    json << ",\n";
    append_section("Mid360s");
    json << "\n}\n";
    return json.str();
}

Result<void> write_config_json(const std::string& path, const std::string& host_ip) {
    if (path.empty()) {
        return make_error(ErrorCode::kInvalidArgument, "Livox 配置文件路径为空");
    }
    if (!is_valid_ipv4(host_ip)) {
        return make_error(ErrorCode::kInvalidArgument, "Livox host_ip 不是合法 IPv4 地址");
    }

    const std::filesystem::path file_path(path);
    if (file_path.has_parent_path()) {
        std::error_code error;
        std::filesystem::create_directories(file_path.parent_path(), error);
        if (error) {
            return make_error(ErrorCode::kInternal, "创建 Livox 配置目录失败: " + error.message());
        }
    }

    std::ofstream stream(file_path, std::ios::out | std::ios::trunc);
    if (!stream) {
        return make_error(ErrorCode::kPermissionDenied, "无法写入 Livox 配置文件: " + path);
    }
    stream << build_config_json(host_ip);
    stream.close();
    if (!stream) {
        return make_error(ErrorCode::kInternal, "写入 Livox 配置文件失败: " + path);
    }

    // 配置里包含主机网络信息，限制为仅所有者可读写。
    if (::chmod(path.c_str(), kPrivateFileMode) != 0) {
        return make_error(ErrorCode::kPermissionDenied, "无法设置 Livox 配置文件权限: " + path);
    }
    return Result<void>();
}

std::string default_config_path(const std::string& runtime_dir, const std::string& host_ip) {
    std::filesystem::path directory;
    if (!runtime_dir.empty()) {
        directory = runtime_dir;
    } else if (const char* xdg_runtime = std::getenv("XDG_RUNTIME_DIR");
               xdg_runtime != nullptr && xdg_runtime[0] != '\0') {
        directory = std::filesystem::path(xdg_runtime) / "foray_livox";
    } else {
        directory = std::filesystem::temp_directory_path() / "foray_livox";
    }

    const std::string name = "livox_config_" + sanitize(host_ip) + ".json";
    return (directory / name).string();
}

} // namespace foray::platform::driver::livox::detail
