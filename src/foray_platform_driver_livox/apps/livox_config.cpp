/**
 * @file livox_config.cpp
 * @brief 雷达参数注入工具：读回 → 下发（IP / 安装姿态 / 主机端口 / PPS）→ 再读回校验。
 *
 * 只通过 `detail::LivoxApi` 与设备对话，SDK 细节留在 `livox_api_real.cpp`；
 * 因此没有真实 SDK 时（stub）会统一返回 `kSdkUnavailable`。
 *
 * 退出码：0 成功 / 1 用法错误 / 2 SDK 初始化失败 / 3 没等到设备 / 4 下发失败 / 5 读回校验失败。
 */

#include <chrono>
#include <filesystem>
#include <fstream>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <mutex>
#include <optional>
#include <string>

#include "livox/config_json.hpp"
#include "livox/livox_api.hpp"
#include "livox/livox_control.hpp"

namespace {

namespace detail = foray::platform::driver::livox::detail;

/// @brief 等待设备上线的上限：Mid-360s 上电到 InfoChange 一般 < 5 s。
constexpr std::chrono::seconds kDeviceTimeout{15};

struct Options {
    std::string json_path;
    std::string host_ip;
    std::string lidar_ip;
    std::string net_mask;
    std::string gateway;
    std::string host_port_ip;
    std::optional<float> roll;
    std::optional<float> pitch;
    std::optional<float> yaw;
    std::optional<std::int32_t> x;
    std::optional<std::int32_t> y;
    std::optional<std::int32_t> z;
    std::optional<std::uint8_t> pps_mode;
    std::optional<std::uint16_t> host_port;
    std::optional<std::uint16_t> lidar_port;
    bool dry_run = false;
};

void print_usage() {
    std::fputs(
        "用法: foray_lidar_config --host-ip <IPv4> [--json <配置文件>] [选项]\n"
        "  省略 --json 时自动生成临时配置（含 host_ip）\n"
        "  --set-lidar-ip <IPv4>   改写雷达 IP（危险：改错会失联）\n"
        "  --net-mask <IPv4>       与 --set-lidar-ip 搭配的掩码\n"
        "  --gateway <IPv4>        与 --set-lidar-ip 搭配的网关\n"
        "  --roll/--pitch/--yaw <度>  --x/--y/--z <mm>   安装姿态\n"
        "  --pps <0|1>             PPS 同步模式（Mid-360s 不支持，会失败）\n"
        "  --host-port <端口> --lidar-port <端口>  点云端口对\n"
        "  --dry-run               只读回，不下发\n",
        stderr);
}

template <typename T>
bool parse_number(const std::string& text, T& value) {
    try {
        std::size_t consumed = 0;
        const long long parsed = std::stoll(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        value = static_cast<T>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_options(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next_value = [&](std::string& target) {
            if (i + 1 >= argc) {
                return false;
            }
            target = argv[++i];
            return true;
        };
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        } else if (arg == "--json") {
            if (!next_value(options.json_path)) {
                return false;
            }
        } else if (arg == "--host-ip") {
            if (!next_value(options.host_ip)) {
                return false;
            }
        } else if (arg == "--set-lidar-ip") {
            if (!next_value(options.lidar_ip)) {
                return false;
            }
        } else if (arg == "--net-mask") {
            if (!next_value(options.net_mask)) {
                return false;
            }
        } else if (arg == "--gateway") {
            if (!next_value(options.gateway)) {
                return false;
            }
        } else if (arg == "--host-port-ip") {
            if (!next_value(options.host_port_ip)) {
                return false;
            }
        } else if (arg == "--roll" || arg == "--pitch" || arg == "--yaw") {
            std::string text;
            float value = 0.0F;
            if (!next_value(text) || !parse_number(text, value)) {
                return false;
            }
            (arg == "--roll" ? options.roll : arg == "--pitch" ? options.pitch : options.yaw) =
                value;
        } else if (arg == "--x" || arg == "--y" || arg == "--z") {
            std::string text;
            std::int32_t value = 0;
            if (!next_value(text) || !parse_number(text, value)) {
                return false;
            }
            (arg == "--x" ? options.x : arg == "--y" ? options.y : options.z) = value;
        } else if (arg == "--pps") {
            std::string text;
            std::uint8_t value = 0;
            if (!next_value(text) || !parse_number(text, value)) {
                return false;
            }
            options.pps_mode = value;
        } else if (arg == "--host-port" || arg == "--lidar-port") {
            std::string text;
            std::uint16_t value = 0;
            if (!next_value(text) || !parse_number(text, value)) {
                return false;
            }
            (arg == "--host-port" ? options.host_port : options.lidar_port) = value;
        } else if (arg == "--dry-run") {
            options.dry_run = true;
        } else {
            std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
            return false;
        }
    }
    if (options.host_ip.empty()) {
        std::fputs("--host-ip 必填\n", stderr);
        return false;
    }
    return true;
}

/// @brief 设备发现等待器：第一个 InfoChange 回调提供 handle 与 IP。
struct DeviceWait {
    std::mutex mutex;
    std::condition_variable cv;
    bool found = false;
    detail::InfoView info;
};

void on_info(void* user, const detail::InfoView& info) {
    auto& wait = *static_cast<DeviceWait*>(user);
    std::lock_guard<std::mutex> lock(wait.mutex);
    if (!wait.found) {
        wait.found = true;
        wait.info = info;
        wait.cv.notify_one();
    }
}

bool has_attitude(const Options& options) {
    // 全默认时不下发姿态，避免把标定值覆盖成 0。
    return options.roll.has_value() || options.pitch.has_value() || options.yaw.has_value() ||
           options.x.has_value() || options.y.has_value() || options.z.has_value();
}

int run(const Options& options) {
    std::string config_path = options.json_path;
    std::string temp_path;
    if (config_path.empty()) {
        temp_path = (std::filesystem::temp_directory_path() /
                     ("foray-lidar-config-" + std::to_string(::getpid()) + ".json"))
                        .string();
        const auto written = detail::write_config_json(temp_path, options.host_ip);
        if (!written.ok()) {
            std::fprintf(stderr, "生成临时配置失败: %s\n", written.error().message.c_str());
            return 2;
        }
        config_path = temp_path;
    }

    // SDK 打不开 JSON 时只打日志、仍返回成功，这里提前拦下来。
    {
        std::ifstream probe(config_path);
        if (!probe) {
            std::fprintf(stderr, "配置文件不存在或不可读: %s\n", config_path.c_str());
            return 2;
        }
    }

    const auto api = detail::make_livox_api();
    const auto init = api->init(config_path, options.host_ip);
    if (!init.ok()) {
        std::fprintf(stderr, "SDK 初始化失败: %s\n", init.error().message.c_str());
        return 2;
    }

    DeviceWait wait;
    api->set_info_change_callback(&on_info, &wait);
    api->start();

    {
        std::unique_lock<std::mutex> lock(wait.mutex);
        if (!wait.cv.wait_for(lock, kDeviceTimeout, [&wait] { return wait.found; })) {
            std::fputs("在超时时间内没有发现雷达（检查网线、IP 与 JSON 配置）\n", stderr);
            api->uninit();
            return 3;
        }
    }

    const std::uint32_t handle = wait.info.handle;
    std::printf("handle=%u dev_type=%u serial=%s ip=%s\n", handle, wait.info.dev_type,
                wait.info.serial != nullptr ? wait.info.serial : "?",
                wait.info.ip_address != nullptr ? wait.info.ip_address : "?");
    if (wait.info.ip_address != nullptr) {
        std::printf("reported-ip: %s\n", wait.info.ip_address);
    }

    detail::LivoxControl control(*api);
    if (options.dry_run) {
        api->uninit();
        return 0;
    }

    int exit_code = 0;

    if (!options.lidar_ip.empty()) {
        const auto applied = control.set_lidar_ip(handle, options.lidar_ip, options.net_mask,
                                                   options.gateway);
        if (!applied.ok()) {
            std::fprintf(stderr, "改写雷达 IP 失败: %s\n", applied.error().message.c_str());
            exit_code = 4;
        } else {
            std::printf("applied: lidar-ip=%s\n", options.lidar_ip.c_str());
            const auto readback = control.verify_lidar_ip(handle, options.lidar_ip);
            if (!readback.ok()) {
                std::fprintf(stderr, "雷达 IP 读回校验失败: %s\n",
                             readback.error().message.c_str());
                exit_code = 5;
            }
        }
    }

    if (has_attitude(options)) {
        const auto applied = control.set_install_attitude(
            handle, options.roll.value_or(0.0F), options.pitch.value_or(0.0F),
            options.yaw.value_or(0.0F), options.x.value_or(0), options.y.value_or(0),
            options.z.value_or(0));
        if (!applied.ok()) {
            std::fprintf(stderr, "下发安装姿态失败: %s\n", applied.error().message.c_str());
            exit_code = 4;
        } else {
            std::printf("applied: install-attitude\n");
        }
    }

    if (options.pps_mode.has_value()) {
        const auto applied = control.set_pps_sync_mode(handle, *options.pps_mode);
        if (!applied.ok()) {
            std::fprintf(stderr, "下发 PPS 模式失败: %s\n", applied.error().message.c_str());
            exit_code = 4;
        } else {
            std::printf("applied: pps=%u\n", static_cast<unsigned>(*options.pps_mode));
        }
    }

    if (options.host_port.has_value() || options.lidar_port.has_value()) {
        const std::string host_ip =
            options.host_port_ip.empty() ? options.host_ip : options.host_port_ip;
        if (!options.host_port.has_value() || !options.lidar_port.has_value()) {
            std::fputs("--host-port 与 --lidar-port 需要成对给出\n", stderr);
            api->uninit();
            return 1;
        }
        const auto applied = control.set_host_port_cfg(handle, host_ip, *options.host_port,
                                                       *options.lidar_port);
        if (!applied.ok()) {
            std::fprintf(stderr, "下发主机端口失败: %s\n", applied.error().message.c_str());
            exit_code = 4;
        } else {
            std::printf("applied: host-port %u -> lidar-port %u\n",
                        static_cast<unsigned>(*options.host_port),
                        static_cast<unsigned>(*options.lidar_port));
        }
    }

    const auto info = api->query_internal_info(handle);
    if (info.ok()) {
        std::printf("readback: ip=%s params=%u\n", info.value().ip_address.c_str(),
                    static_cast<unsigned>(info.value().param_count));
    } else {
        std::fprintf(stderr, "读回内部信息失败: %s\n", info.error().message.c_str());
    }

    api->uninit();
    if (!temp_path.empty()) {
        std::error_code ignored;
        std::filesystem::remove(temp_path, ignored);
    }
    return exit_code;
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_options(argc, argv, options)) {
        print_usage();
        return 1;
    }
    return run(options);
}
