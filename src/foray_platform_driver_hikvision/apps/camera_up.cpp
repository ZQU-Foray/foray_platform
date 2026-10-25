#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "foray/platform/driver/hikvision/camera.hpp"
#include "foray/platform/hal/camera.hpp"
#include "foray/platform/hal/frame_lease.hpp"
#include "foray/platform/net/bootstrap.hpp"
#include "foray/platform/net/net_types.hpp"
#include "foray/platform/net/process.hpp"
#include "foray/platform/result/error.hpp"
#include "foray/platform/time/duration.hpp"
#include "hikvision/camera_bootstrap.hpp"
#include "hikvision/mvs_api.hpp"

namespace {

using foray::platform::ErrorCode;
using foray::platform::hal::Camera;
using foray::platform::hal::CameraConfig;
using foray::platform::hal::FrameLease;
using foray::platform::hal::ImageFrame;
using foray::platform::time::Duration;

/// @brief 退出码：2 用法 / 3 环境与网络 / 4 没挑到相机 / 5 打开或启动失败 / 6 没取到帧。
constexpr int kExitUsage = 2;
constexpr int kExitEnvironment = 3;
constexpr int kExitNoDevice = 4;
constexpr int kExitDriver = 5;
constexpr int kExitNoFrames = 6;

/// @brief 巨帧推荐值：GigE 相机 720p 以上分辨率下避免分片丢包。
constexpr int kDefaultMtu = 9014;
constexpr int kMinMtu = 576;
constexpr int kMaxMtu = 65535;
constexpr int kDefaultFrames = 3;
constexpr int kDefaultTimeoutMs = 1000;

struct Options {
    std::string iface;
    std::string bridge = "br-camera";
    std::string host_ip;
    int prefix = 16;
    int mtu = kDefaultMtu;
    std::string helper = "foray-net-helper";
    bool dry_run = false;
    bool no_network = false;
    bool allow_reassign_master = false;
    std::string transport = "gige";
    std::string serial;
    std::string model;
    std::string ip;
    std::string name;
    int frames = kDefaultFrames;
    int timeout_ms = kDefaultTimeoutMs;
};

void print_usage(const char* program) {
    std::fprintf(
        stderr,
        "用法: %s [选项]\n"
        "  --iface <网口>            相机所在物理网口（GigE 自举必填，如 enp0s13f0u1）\n"
        "  --bridge <网桥>           相机独占的网桥（默认 br-camera）\n"
        "  --host-ip <IPv4>          桥上主机地址（默认按相机地址自动规划 LLA/同网段地址）\n"
        "  --prefix <1-32>           主机地址前缀（默认 16）\n"
        "  --mtu <576-65535>         物理口与网桥的 MTU（默认 9014，失败只警告）\n"
        "  --helper <路径>           特权 helper（默认 foray-net-helper）\n"
        "  --allow-reassign-master   允许把已属于另一个网桥的物理口抢过来\n"
        "  --serial <序列号>         按序列号选择相机\n"
        "  --model <型号>            按型号选择（需保证唯一）\n"
        "  --name <名称>             按用户自定义名选择\n"
        "  --ip <地址>               按 IP 选择（GigE）\n"
        "  --frames <n>              取多少帧后退出（默认 3，0 = 不限）\n"
        "  --timeout-ms <n>          单帧读超时毫秒（默认 1000，必须 > 0）\n"
        "  --transport gige|usb3     传输层（默认 gige；usb3 目前只做可见性检测）\n"
        "  --no-network              跳过全部网络自举步骤（链路已手工配好时使用）\n"
        "  --dry-run                 只打印网络计划，不修改系统\n",
        program);
}

bool take_value(int argc, char** argv, int& index, std::string& out) {
    if (index + 1 >= argc) {
        return false;
    }
    out = argv[++index];
    return true;
}

bool parse_int(const std::string& text, int& out) {
    try {
        std::size_t consumed = 0;
        const int value = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        out = value;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

void clear_selectors(Options& options) {
    options.serial.clear();
    options.model.clear();
    options.ip.clear();
    options.name.clear();
}

bool parse_options(int argc, char** argv, Options& options, bool& help) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            help = true;
            return true;
        }
        if (arg == "--iface") {
            if (!take_value(argc, argv, i, options.iface)) {
                return false;
            }
        } else if (arg == "--bridge") {
            if (!take_value(argc, argv, i, options.bridge)) {
                return false;
            }
        } else if (arg == "--host-ip") {
            if (!take_value(argc, argv, i, options.host_ip)) {
                return false;
            }
        } else if (arg == "--prefix") {
            std::string text;
            if (!take_value(argc, argv, i, text) || !parse_int(text, options.prefix) ||
                options.prefix < 1 || options.prefix > 32) {
                return false;
            }
        } else if (arg == "--mtu") {
            std::string text;
            if (!take_value(argc, argv, i, text) || !parse_int(text, options.mtu) ||
                options.mtu < kMinMtu || options.mtu > kMaxMtu) {
                return false;
            }
        } else if (arg == "--helper") {
            if (!take_value(argc, argv, i, options.helper)) {
                return false;
            }
        } else if (arg == "--allow-reassign-master") {
            options.allow_reassign_master = true;
        } else if (arg == "--serial") {
            clear_selectors(options);
            if (!take_value(argc, argv, i, options.serial)) {
                return false;
            }
        } else if (arg == "--model") {
            clear_selectors(options);
            if (!take_value(argc, argv, i, options.model)) {
                return false;
            }
        } else if (arg == "--name") {
            clear_selectors(options);
            if (!take_value(argc, argv, i, options.name)) {
                return false;
            }
        } else if (arg == "--ip") {
            clear_selectors(options);
            if (!take_value(argc, argv, i, options.ip)) {
                return false;
            }
        } else if (arg == "--frames") {
            std::string text;
            if (!take_value(argc, argv, i, text) || !parse_int(text, options.frames) ||
                options.frames < 0) {
                return false;
            }
        } else if (arg == "--timeout-ms") {
            std::string text;
            if (!take_value(argc, argv, i, text) || !parse_int(text, options.timeout_ms) ||
                options.timeout_ms <= 0) {
                return false;
            }
        } else if (arg == "--transport") {
            if (!take_value(argc, argv, i, options.transport) ||
                (options.transport != "gige" && options.transport != "usb3")) {
                return false;
            }
        } else if (arg == "--no-network") {
            options.no_network = true;
        } else if (arg == "--dry-run") {
            options.dry_run = true;
        } else {
            std::fprintf(stderr, "未知选项: %s\n", arg.c_str());
            return false;
        }
    }
    return true;
}

std::string resolve_helper(const std::string& helper) {
#ifdef FORAY_NET_HELPER_DEFAULT
    if (helper == "foray-net-helper") {
        return FORAY_NET_HELPER_DEFAULT;
    }
#endif
    return helper;
}

std::int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void print_stats(const Camera& camera) {
    const auto stats = camera.stats();
    std::printf("统计: 收 %llu 帧 / 交付 %llu / 丢弃 %llu / 超时 %llu / SDK 错误 %llu"
                " / 字节 %llu / 重连 %llu\n",
                static_cast<unsigned long long>(stats.frames_received),
                static_cast<unsigned long long>(stats.frames_delivered),
                static_cast<unsigned long long>(stats.frames_dropped),
                static_cast<unsigned long long>(stats.timeouts),
                static_cast<unsigned long long>(stats.sdk_errors),
                static_cast<unsigned long long>(stats.bytes_received),
                static_cast<unsigned long long>(stats.reconnects));
    if (stats.sdk_errors != 0U) {
        std::printf("最后一次 SDK 错误码: 0x%08X\n",
                    static_cast<unsigned>(stats.last_error_vendor_code));
    }
}

std::vector<foray::platform::driver::hikvision::detail::DeviceDescriptor>
deduplicate_by_serial(
    const std::vector<foray::platform::driver::hikvision::detail::DeviceDescriptor>& devices) {
    std::vector<foray::platform::driver::hikvision::detail::DeviceDescriptor> unique;
    unique.reserve(devices.size());
    for (const auto& device : devices) {
        if (device.serial.empty()) {
            unique.push_back(device);
            continue;
        }
        const bool seen =
            std::any_of(unique.begin(), unique.end(), [&device](const auto& candidate) {
                return !candidate.serial.empty() && candidate.serial == device.serial;
            });
        if (!seen) {
            unique.push_back(device);
        }
    }
    return unique;
}

foray::platform::Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>
pick_device(const std::vector<foray::platform::driver::hikvision::detail::DeviceDescriptor>& devices,
            const Options& options) {
    using foray::platform::ErrorCode;
    using foray::platform::make_error;
    using foray::platform::Result;

    const auto matches = [&devices](auto predicate) {
        std::vector<foray::platform::driver::hikvision::detail::DeviceDescriptor> found;
        for (const auto& device : devices) {
            if (predicate(device)) {
                found.push_back(device);
            }
        }
        return found;
    };

    if (!options.serial.empty()) {
        const auto found = matches(
            [&options](const auto& d) { return d.serial == options.serial; });
        if (found.empty()) {
            return Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>(
                make_error(ErrorCode::kDeviceNotFound,
                           "未找到 serial=" + options.serial + " 的相机"));
        }
        return found.front();
    }
    if (!options.name.empty()) {
        const auto found = matches(
            [&options](const auto& d) { return d.user_defined_name == options.name; });
        if (found.empty()) {
            return Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>(
                make_error(ErrorCode::kDeviceNotFound,
                           "未找到 name=" + options.name + " 的相机"));
        }
        if (found.size() > 1) {
            return Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>(
                make_error(ErrorCode::kDeviceAmbiguous,
                           "name=" + options.name + " 命中多台相机，请改用 --serial"));
        }
        return found.front();
    }
    if (!options.ip.empty()) {
        const auto found = matches([&options](const auto& d) {
            return d.transport == foray::platform::hal::Transport::kGigE &&
                   d.ip_address == options.ip;
        });
        if (found.empty()) {
            return Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>(
                make_error(ErrorCode::kDeviceNotFound,
                           "未找到 ip=" + options.ip + " 的相机"));
        }
        if (found.size() > 1) {
            return Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>(
                make_error(ErrorCode::kDeviceAmbiguous,
                           "ip=" + options.ip + " 命中多台相机，请改用 --serial"));
        }
        return found.front();
    }
    if (!options.model.empty()) {
        const auto found =
            matches([&options](const auto& d) { return d.model == options.model; });
        if (found.empty()) {
            return Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>(
                make_error(ErrorCode::kDeviceNotFound,
                           "未找到 model=" + options.model + " 的相机"));
        }
        if (found.size() > 1) {
            return Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>(
                make_error(ErrorCode::kDeviceAmbiguous,
                           "model=" + options.model + " 命中多台相机，请改用 --serial"));
        }
        return found.front();
    }

    if (devices.empty()) {
        return Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>(
            make_error(ErrorCode::kDeviceNotFound,
                       "未枚举到 GigE 相机：检查网线、相机供电与 --iface"));
    }
    if (devices.size() > 1) {
        return Result<foray::platform::driver::hikvision::detail::DeviceDescriptor>(
            make_error(ErrorCode::kDeviceAmbiguous,
                       "枚举到 " + std::to_string(devices.size()) +
                           " 台相机，请用 --serial/--model/--ip/--name 指定"));
    }
    return devices.front();
}

int bootstrap_network(const Options& options) {
    const auto topology = foray::platform::net::probe_topology();
    if (!topology.ok()) {
        std::fprintf(stderr, "拓扑探测失败: %s\n", topology.error().message.c_str());
        return kExitEnvironment;
    }
    const auto* iface = topology.value().find(options.iface);
    if (iface == nullptr) {
        std::fprintf(stderr, "网口不存在: %s\n", options.iface.c_str());
        return kExitEnvironment;
    }
    if (!iface->carrier) {
        std::fprintf(stderr, "网口 %s 无载波：相机未上电或网线未插好\n", options.iface.c_str());
        return kExitEnvironment;
    }

    // 先试探一次：能看到相机就用它的 IP 推导主机地址；看不到也继续配网，配完再枚举。
    std::string device_ip;
    auto api = foray::platform::driver::hikvision::detail::make_mvs_api();
    const auto probe = api->enumerate(foray::platform::hal::Transport::kGigE);
    if (probe.ok() && !probe.value().empty()) {
        device_ip = probe.value().front().ip_address;
    }

    auto plan = foray::platform::driver::hikvision::detail::plan_host_address(
        device_ip, topology.value().find(options.bridge));
    if (!options.host_ip.empty()) {
        const auto parsed = foray::platform::net::Ipv4::parse(options.host_ip);
        if (!parsed.has_value()) {
            std::fprintf(stderr, "主机地址非法: %s\n", options.host_ip.c_str());
            return kExitUsage;
        }
        plan.host_ip = *parsed;
        plan.prefix = static_cast<std::uint8_t>(options.prefix);
        plan.reason = "使用 --host-ip 指定的地址";
    }
    if (plan.host_ip.has_value()) {
        std::printf("主机地址规划: %s/%u（%s）\n", plan.host_ip->to_string().c_str(),
                    static_cast<unsigned>(plan.prefix), plan.reason.c_str());
    }

    foray::platform::net::BootstrapOptions bootstrap;
    bootstrap.iface = options.iface;
    bootstrap.bridge = options.bridge;
    bootstrap.host_ip = plan.host_ip;
    bootstrap.prefix = plan.prefix;
    bootstrap.min_prefix = 16;
    bootstrap.mtu = static_cast<std::uint32_t>(options.mtu);
    bootstrap.allow_reassign_master = options.allow_reassign_master;
    bootstrap.dry_run = options.dry_run;
    bootstrap.helper = resolve_helper(options.helper);

    foray::platform::net::ExecProcessRunner runner;
    foray::platform::net::ProcessOutput observed;
    const auto result =
        foray::platform::net::bootstrap_link(bootstrap, runner, &observed);
    if (!observed.out.empty()) {
        std::fputs(observed.out.c_str(), stdout);
    }
    if (!observed.err.empty()) {
        std::fputs(observed.err.c_str(), stderr);
    }
    if (!result.ok()) {
        std::fprintf(stderr, "网络自举失败: %s\n", result.error().message.c_str());
        return kExitEnvironment;
    }
    if (result.value().skipped) {
        std::fprintf(stderr, "链路已就绪: %s -> %s\n", options.iface.c_str(),
                     options.bridge.c_str());
    }
    return 0;
}

int run_usb3() {
    auto api = foray::platform::driver::hikvision::detail::make_mvs_api();
    const auto devices =
        api->enumerate(foray::platform::hal::Transport::kUsb3);
    if (!devices.ok()) {
        std::fprintf(stderr, "USB3 枚举失败: %s\n", devices.error().message.c_str());
    } else if (devices.value().empty()) {
        std::fprintf(stderr, "未发现 USB3 相机：检查线缆、USB3 供电与 udev 权限\n");
    } else {
        for (const auto& device : devices.value()) {
            std::printf("USB3 相机: %s %s serial=%s\n", device.model.c_str(),
                        device.user_defined_name.c_str(), device.serial.c_str());
        }
    }
    std::printf("TODO: USB3 协议自举尚未完善（当前只做可见性检测）\n");
    return 0;
}

int run_camera(const Options& options) {
    auto api = foray::platform::driver::hikvision::detail::make_mvs_api();
    const auto enumerated =
        api->enumerate(foray::platform::hal::Transport::kGigE);
    if (!enumerated.ok()) {
        std::fprintf(stderr, "枚举相机失败: %s\n", enumerated.error().message.c_str());
        return kExitEnvironment;
    }
    const auto devices = deduplicate_by_serial(enumerated.value());
    const auto picked = pick_device(devices, options);
    if (!picked.ok()) {
        std::fprintf(stderr, "%s\n", picked.error().message.c_str());
        return kExitNoDevice;
    }

    CameraConfig config;
    config.selector.transport = foray::platform::hal::Transport::kGigE;
    if (!options.serial.empty()) {
        config.selector.serial = options.serial;
    } else if (!options.name.empty()) {
        config.selector.user_defined_name = options.name;
    } else if (!options.ip.empty()) {
        config.selector.ip_address = options.ip;
    } else if (!options.model.empty()) {
        config.selector.model = options.model;
    } else if (!picked.value().serial.empty()) {
        config.selector.serial = picked.value().serial;
    } else if (!picked.value().ip_address.empty()) {
        config.selector.ip_address = picked.value().ip_address;
    } else {
        config.selector.model = picked.value().model;
    }

    auto created = foray::platform::driver::hikvision::make_camera(config);
    if (!created.ok()) {
        std::fprintf(stderr, "构造相机失败: %s (code=%s)\n", created.error().message.c_str(),
                     foray::platform::to_string(created.error().code));
        return kExitDriver;
    }
    auto camera = std::move(created.value());

    auto status = camera->open();
    if (!status.ok()) {
        std::fprintf(stderr, "打开相机失败: %s (code=%s vendor=0x%08X)\n",
                     status.error().message.c_str(),
                     foray::platform::to_string(status.error().code),
                     static_cast<unsigned>(status.error().vendor_code));
        (void)camera->close();
        return kExitDriver;
    }
    const auto info = camera->info();
    std::printf("已连接: %s %s serial=%s fw=%s ip=%s\n", info.vendor.c_str(),
                info.model.c_str(), info.serial.c_str(), info.firmware_version.c_str(),
                info.ip_address.c_str());

    status = camera->start();
    if (!status.ok()) {
        std::fprintf(stderr, "启动取流失败: %s (code=%s)\n", status.error().message.c_str(),
                     foray::platform::to_string(status.error().code));
        (void)camera->close();
        return kExitDriver;
    }

    const auto frames_wanted = static_cast<std::uint64_t>(options.frames);
    std::uint64_t received = 0;
    std::uint64_t read_timeouts = 0;
    std::uint32_t consecutive_timeouts = 0;
    const std::int64_t loop_begin_ns = steady_now_ns();
    std::int64_t first_stamp_ns = 0;
    std::int64_t last_stamp_ns = 0;
    std::uint32_t last_width = 0;
    std::uint32_t last_height = 0;
    bool read_failed = false;

    while (frames_wanted == 0U || received < frames_wanted) {
        FrameLease<ImageFrame> lease;
        status = camera->read(lease, Duration::from_milliseconds(options.timeout_ms));
        if (!status.ok()) {
            if (status.error().code == ErrorCode::kTimeout ||
                status.error().code == ErrorCode::kNoData) {
                ++read_timeouts;
                ++consecutive_timeouts;
                // 按目标帧数给同样的超时预算：连续 frames 次读超时说明相机没有出图。
                if (frames_wanted != 0U && consecutive_timeouts >= frames_wanted) {
                    std::fprintf(stderr, "连续 %u 次读超时，相机没有出图\n", consecutive_timeouts);
                    read_failed = true;
                    break;
                }
                continue;
            }
            std::fprintf(stderr, "读帧失败: %s (code=%s)\n", status.error().message.c_str(),
                         foray::platform::to_string(status.error().code));
            read_failed = true;
            break;
        }
        if (!lease) {
            continue;
        }

        const ImageFrame& frame = *lease;
        ++received;
        consecutive_timeouts = 0;
        if (received == 1U) {
            first_stamp_ns = frame.stamp.nanoseconds_since_epoch();
        }
        last_stamp_ns = frame.stamp.nanoseconds_since_epoch();
        last_width = frame.width;
        last_height = frame.height;
        std::printf("帧 %llu: %ux%u stride=%u seq=%llu dev_no=%u sync=%s\n",
                    static_cast<unsigned long long>(received), frame.width, frame.height,
                    frame.stride, static_cast<unsigned long long>(frame.sequence),
                    frame.device_frame_number,
                    foray::platform::hal::to_string(frame.sync));
    }

    if (received > 1U && first_stamp_ns != 0 && last_stamp_ns > first_stamp_ns) {
        const double span_s = static_cast<double>(last_stamp_ns - first_stamp_ns) / 1e9;
        std::printf("平均帧率: %.2f Hz（%llu 帧 / %.3f s，%ux%u）\n",
                    static_cast<double>(received - 1U) / span_s,
                    static_cast<unsigned long long>(received - 1U), span_s, last_width,
                    last_height);
    } else if (received > 1U) {
        const double span_s = static_cast<double>(steady_now_ns() - loop_begin_ns) / 1e9;
        if (span_s > 0.0) {
            std::printf("平均帧率（主机节拍）: %.2f Hz（%llu 帧 / %.3f s，%ux%u）\n",
                        static_cast<double>(received) / span_s,
                        static_cast<unsigned long long>(received), span_s, last_width,
                        last_height);
        }
    }
    if (read_timeouts != 0U) {
        std::printf("读超时 %llu 次\n", static_cast<unsigned long long>(read_timeouts));
    }

    (void)camera->stop();
    print_stats(*camera);
    (void)camera->close();

    if (read_failed || (frames_wanted != 0U && received < frames_wanted)) {
        return kExitNoFrames;
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    bool help = false;
    if (!parse_options(argc, argv, options, help)) {
        print_usage(argv[0]);
        return kExitUsage;
    }
    if (help) {
        print_usage(argv[0]);
        return 0;
    }

    if (options.transport == "usb3") {
        return run_usb3();
    }

    if (!options.no_network) {
        if (options.iface.empty()) {
            std::fprintf(stderr, "GigE 自举需要 --iface（或用 --no-network 跳过）\n");
            print_usage(argv[0]);
            return kExitUsage;
        }
        const int status = bootstrap_network(options);
        if (status != 0) {
            return status;
        }
        if (options.dry_run) {
            std::fprintf(stderr, "dry-run: 跳过相机枚举与取流\n");
            return 0;
        }
    }

    return run_camera(options);
}
