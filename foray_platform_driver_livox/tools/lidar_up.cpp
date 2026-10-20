/**
 * @file lidar_up.cpp
 * @brief 一键起雷达：网络自举（可经 pkexec）→ 打开驱动 → 读若干帧确认数据流。
 *
 * 与 `foray_livox_dump` 的分工：本工具只做「编排 + 健康确认」，
 * 点云落盘或管道输出交给 `foray_livox_dump`。
 *
 * 退出码：0 成功 / 1 用法 / 2 驱动创建失败 / 3 open 失败 / 4 start 失败 /
 * 5 网络自举失败 / 6 没读到帧。
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <unistd.h>

#include "foray/platform/driver/livox/lidar.hpp"
#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/lidar.hpp"
#include "foray/platform/net/net_types.hpp"
#include "foray/platform/net/process.hpp"
#include "foray/platform/net/setup.hpp"
#include "foray/platform/time/clock.hpp"
#include "foray/platform/time/duration.hpp"

namespace {

namespace hal = foray::platform::hal;
namespace net = foray::platform::net;
namespace time = foray::platform::time;

struct Options {
    std::string iface;
    std::string bridge;
    std::string host_ip;
    std::string lidar_ip;
    std::string config_json;
    std::string helper = "foray-net-helper";
    int frames = 3;
    int timeout_ms = 1000;
    bool dry_run = false;
};

void print_usage() {
    std::fputs(
        "用法: foray_lidar_up --iface <物理口> --bridge <网桥> --host-ip <IPv4> \\\n"
        "                      [--lidar-ip <IPv4>] [--json <配置>] [--helper <路径>] \\\n"
        "                      [--frames N] [--timeout-ms N] [--dry-run]\n"
        "  --frames 0   读到为止（Ctrl-C 停止）\n"
        "  --dry-run    只做网络自举规划与载波检查，不启动驱动\n",
        stderr);
}

bool take_value(int argc, char** argv, int& i, std::string& out) {
    if (i + 1 >= argc) {
        return false;
    }
    out = argv[++i];
    return true;
}

bool parse_options(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        } else if (arg == "--iface") {
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
        } else if (arg == "--lidar-ip") {
            if (!take_value(argc, argv, i, options.lidar_ip)) {
                return false;
            }
        } else if (arg == "--json") {
            if (!take_value(argc, argv, i, options.config_json)) {
                return false;
            }
        } else if (arg == "--helper") {
            if (!take_value(argc, argv, i, options.helper)) {
                return false;
            }
        } else if (arg == "--frames") {
            std::string text;
            if (!take_value(argc, argv, i, text)) {
                return false;
            }
            options.frames = std::atoi(text.c_str());
        } else if (arg == "--dry-run") {
            options.dry_run = true;
        } else if (arg == "--timeout-ms") {
            std::string text;
            if (!take_value(argc, argv, i, text)) {
                return false;
            }
            options.timeout_ms = std::atoi(text.c_str());
        } else {
            std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
            return false;
        }
    }
    if (options.iface.empty() || options.bridge.empty() || options.host_ip.empty()) {
        std::fputs("--iface / --bridge / --host-ip 必填\n", stderr);
        return false;
    }
    return true;
}

int bootstrap_link(const Options& options) {
    const auto topology = net::probe_topology();
    if (!topology.ok()) {
        std::fprintf(stderr, "拓扑探测失败: %s\n", topology.error().message.c_str());
        return 5;
    }
    const auto host_ip = net::Ipv4::parse(options.host_ip);
    if (host_ip.has_value() &&
        net::link_ready(topology.value(), options.iface, options.bridge, *host_ip)) {
        std::fprintf(stderr, "链路已就绪: %s -> %s\n", options.iface.c_str(),
                     options.bridge.c_str());
        return 0;
    }

    // 默认名解析成安装路径；显式 --helper 保持原样。
    std::string helper = options.helper;
    if (helper == "foray-net-helper") {
#ifdef FORAY_NET_HELPER_DEFAULT
        helper = FORAY_NET_HELPER_DEFAULT;
#endif
    }

    std::vector<std::string> argv;
    if (geteuid() != 0) {
        // 非 root 时走 polkit 图形授权，不要求操作手在终端敲密码。
        argv.emplace_back("pkexec");
    }
    argv.push_back(helper);
    argv.insert(argv.end(),
                {"--iface", options.iface, "--bridge", options.bridge, "--host-ip",
                 options.host_ip});
    if (options.dry_run) {
        argv.emplace_back("--dry-run");
    }

    net::ExecProcessRunner runner;
    const auto executed = runner.run(argv);
    if (!executed.ok()) {
        std::fprintf(stderr, "调用网络自举失败: %s\n", executed.error().message.c_str());
        return 5;
    }
    const auto& output = executed.value();
    if (!output.out.empty()) {
        std::fputs(output.out.c_str(), stdout);
    }
    if (!output.err.empty()) {
        std::fputs(output.err.c_str(), stderr);
    }
    if (output.exit_code != 0) {
        std::fprintf(stderr, "网络自举退出码 %d（可用 --helper 指定路径）\n",
                     output.exit_code);
        return 5;
    }
    return 0;
}

int run(const Options& options) {
    const int link_status = bootstrap_link(options);
    if (link_status != 0) {
        return link_status;
    }

    if (options.dry_run) {
        std::fputs("dry-run: 跳过驱动启动\n", stderr);
        return 0;
    }

    hal::LidarConfig config;
    config.selector.ip_address = options.lidar_ip;
    config.host_ip = options.host_ip;
    config.config_json_path = options.config_json;
    config.frame_id = "livox_lidar_up";
    config.frame_timeout_ms = 200;
    config.point_buffer_capacity = 200000;

    auto created = foray::platform::driver::livox::make_lidar(config);
    if (!created.ok()) {
        std::fprintf(stderr, "make_lidar 失败: %s\n", created.error().message.c_str());
        return 2;
    }
    std::unique_ptr<hal::Lidar> lidar = std::move(created.value());
    if (auto opened = lidar->open(); !opened.ok()) {
        std::fprintf(stderr, "open 失败: %s\n", opened.error().message.c_str());
        return 3;
    }
    if (auto started = lidar->start(); !started.ok()) {
        std::fprintf(stderr, "start 失败: %s\n", started.error().message.c_str());
        lidar->close();
        return 4;
    }

    const time::Duration timeout = time::Duration::from_milliseconds(options.timeout_ms);
    std::uint64_t frames = 0;
    std::uint64_t points = 0;
    const time::Timestamp begin = time::SteadyClock{}.now();
    while (options.frames == 0 || frames < static_cast<std::uint64_t>(options.frames)) {
        hal::FrameLease<hal::PointCloudFrame> lease;
        const auto read = lidar->read(lease, timeout);
        if (!read.ok()) {
            std::fprintf(stderr, "读帧失败: %s\n", read.error().message.c_str());
            break;
        }
        const hal::PointCloudFrame& frame = *lease;
        ++frames;
        points += frame.count;
        std::printf("frame=%llu seq=%llu points=%zu dropped=%u sync=%s\n",
                    static_cast<unsigned long long>(frames),
                    static_cast<unsigned long long>(frame.sequence), frame.count,
                    frame.dropped_packets, hal::to_string(frame.sync));
    }

    const auto stats = lidar->stats();
    const double elapsed_s =
        static_cast<double>((time::SteadyClock{}.now() - begin).nanoseconds()) / 1e9;
    std::printf("汇总: frames=%llu points=%llu elapsed_s=%.2f\n",
                static_cast<unsigned long long>(frames),
                static_cast<unsigned long long>(points), elapsed_s);
    std::printf("stats: packets=%llu packets_dropped=%llu frames_dropped=%llu timeouts=%llu\n",
                static_cast<unsigned long long>(stats.packets_received),
                static_cast<unsigned long long>(stats.packets_dropped),
                static_cast<unsigned long long>(stats.frames_dropped),
                static_cast<unsigned long long>(stats.timeouts));

    lidar->stop();
    lidar->close();
    if (frames == 0) {
        std::fputs("没有读到任何帧（检查雷达 IP、JSON 与 UDP 端口）\n", stderr);
        return 6;
    }
    return 0;
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
