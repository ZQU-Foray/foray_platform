#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "foray/platform/driver/hikvision/camera.hpp"
#include "foray/platform/hal/camera.hpp"
#include "foray/platform/hal/frame_lease.hpp"
#include "foray/platform/result/error.hpp"
#include "foray/platform/time/duration.hpp"
#include "viewer.hpp"

namespace {

using foray::platform::ErrorCode;
using foray::platform::hal::Camera;
using foray::platform::hal::CameraConfig;
using foray::platform::hal::FrameLease;
using foray::platform::hal::ImageFrame;
using foray::platform::time::Duration;

void print_usage(const char* program) {
    std::fprintf(stderr,
                 "用法: %s <serial> [选项]\n"
                 "  --model <型号>      按型号选择（需保证唯一）\n"
                 "  --name <名称>       按用户自定义名选择\n"
                 "  --ip <地址>         按 IP 选择（GigE）\n"
                 "  --frames <n>        取多少帧后退出（默认 5，0 = 不限，Ctrl-C 退出）\n"
                 "  --timeout-ms <n>    单帧读超时毫秒（默认 1000，必须 > 0）\n"
                 "  --view              开一个 SDL2 窗口实时显示（需构建期找到 SDL2；\n"
                 "                      窗口里按 ESC 或 q 关闭；关闭即停止取流）\n"
                 "  --width <n>         期望宽度（默认用设备当前值）\n"
                 "  --height <n>        期望高度（默认用设备当前值）\n"
                 "  --exposure-us <f>   曝光时间\n"
                 "  --gain-db <f>       增益\n"
                 "  --trigger-software  软触发模式：配置后每帧先 trigger() 再读\n",
                 program);
}

/// @brief 解析一个 double 选项；非法输入直接退出，避免带着半截配置开流。
bool parse_double(const char* text, double& out) {
    try {
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != std::strlen(text)) {
            return false;
        }
        out = value;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parse_unsigned(const char* text, std::uint32_t& out) {
    double value = 0.0;
    if (!parse_double(text, value) || value < 0.0) {
        return false;
    }
    out = static_cast<std::uint32_t>(value);
    return true;
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

const char* sync_name(foray::platform::hal::TimestampSync sync) {
    return foray::platform::hal::to_string(sync);
}

/// @brief 预览窗口标题的刷新间隔（纳秒）。0.5 s 取「人眼能读清」与「不每帧调
/// SDL_SetWindowTitle」的折中；126 Hz 下每帧刷标题会白白吃掉几个百分点的 CPU。
constexpr std::int64_t kTitleIntervalNs = 500'000'000;

/// @brief 收到 SIGINT/SIGTERM 后置位，主循环据此退出并发 stop()/close()。
///
/// 必须自己接管：`--view` 下 SDL 默认会装自己的 SIGINT/SIGTERM handler，只往事件
/// 队列推一个 SDL_QUIT 而不终止进程。取流线程在阻塞等帧时那个事件迟迟不生效，
/// 实测进程会挂着不退出、相机独占也不释放（下一次 open 报 MV_E_ACCESS_DENIED
/// 0x80000203）。viewer 侧已用 SDL_HINT_NO_SIGNAL_HANDLERS 关掉 SDL 的接管。
volatile std::sig_atomic_t g_stop_requested = 0;

extern "C" void handle_stop_signal(int /*signal*/) {
    g_stop_requested = 1;
}

} // namespace

int main(int argc, char** argv) {
    // --help 必须在位置参数解析之前处理，否则会被当成 serial 去打设备。
    if (argc < 2 || std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0) {
        print_usage(argv[0]);
        return argc < 2 ? 2 : 0;
    }

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);

    // 第一个位置参数按 serial 处理；其它选择条件走选项，逐个替换选择条件，
    // 避免「同时给了 serial 与型号」时被驱动的优先级静默取其一。
    CameraConfig config;
    config.selector.serial = argv[1];

    std::uint64_t frames_wanted = 5;
    std::int64_t timeout_ms = 1000;
    bool timeout_ms_explicit = false;
    bool software_trigger = false;
    bool view_requested = false;

    for (int index = 2; index < argc; ++index) {
        const std::string option = argv[index];
        const auto next = [&index, argc, argv](const char* name) -> const char* {
            if (index + 1 >= argc) {
                std::fprintf(stderr, "选项 %s 缺少参数\n", name);
                return nullptr;
            }
            return argv[++index];
        };
        if (option == "--help" || option == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        if (option == "--model" || option == "--name" || option == "--ip") {
            const char* text = next(option.c_str());
            if (text == nullptr) {
                return 2;
            }
            // 一次只用一种选择条件：后给的覆盖前面的，走哪条由驱动优先级决定。
            config.selector.serial.clear();
            config.selector.user_defined_name.clear();
            config.selector.model.clear();
            config.selector.ip_address.clear();
            if (option == "--model") {
                config.selector.model = text;
            } else if (option == "--name") {
                config.selector.user_defined_name = text;
            } else {
                config.selector.ip_address = text;
            }
            continue;
        }
        if (option == "--frames") {
            const char* text = next("--frames");
            std::uint32_t value = 0;
            if (text == nullptr || !parse_unsigned(text, value)) {
                std::fprintf(stderr, "--frames 需要一个非负整数\n");
                return 2;
            }
            frames_wanted = value;
            continue;
        }
        if (option == "--timeout-ms") {
            const char* text = next("--timeout-ms");
            std::uint32_t value = 0;
            if (text == nullptr || !parse_unsigned(text, value) || value == 0) {
                // 0 会让 read() 立即超时（HAL 语义是 timeout <= 0 直接返回），没有意义。
                std::fprintf(stderr, "--timeout-ms 需要一个正整数\n");
                return 2;
            }
            timeout_ms = static_cast<std::int64_t>(value);
            timeout_ms_explicit = true;
            continue;
        }
        if (option == "--view") {
            view_requested = true;
            continue;
        }
        if (option == "--width" || option == "--height") {
            const char* text = next(option.c_str());
            std::uint32_t value = 0;
            if (text == nullptr || !parse_unsigned(text, value)) {
                std::fprintf(stderr, "%s 需要一个非负整数\n", option.c_str());
                return 2;
            }
            if (option == "--width") {
                config.width = value;
            } else {
                config.height = value;
            }
            continue;
        }
        if (option == "--exposure-us") {
            const char* text = next("--exposure-us");
            double value = 0.0;
            if (text == nullptr || !parse_double(text, value)) {
                std::fprintf(stderr, "--exposure-us 需要一个数字\n");
                return 2;
            }
            config.exposure_us = value;
            continue;
        }
        if (option == "--gain-db") {
            const char* text = next("--gain-db");
            double value = 0.0;
            if (text == nullptr || !parse_double(text, value)) {
                std::fprintf(stderr, "--gain-db 需要一个数字\n");
                return 2;
            }
            config.gain_db = value;
            continue;
        }
        if (option == "--trigger-software") {
            config.trigger.mode = foray::platform::hal::TriggerMode::kSoftware;
            software_trigger = true;
            continue;
        }
        std::fprintf(stderr, "未知选项: %s\n", option.c_str());
        print_usage(argv[0]);
        return 2;
    }

    if (view_requested && !timeout_ms_explicit) {
        // 预览模式下把读超时压到 100 ms：循环同时负责刷窗口与响应关窗，
        // 1 s 的阻塞读会让「按下关窗到程序退出」明显卡顿。
        timeout_ms = 100;
    }

    auto created = foray::platform::driver::hikvision::make_camera(config);
    if (!created.ok()) {
        std::fprintf(stderr, "构造相机失败: %s\n", created.error().message.c_str());
        return 1;
    }
    auto camera = std::move(created.value());
    if (camera == nullptr) {
        std::fprintf(stderr, "构造相机失败: 返回空指针\n");
        return 1;
    }

    // open() 才会真正访问设备；失败信息里带 MVS 原始码，便于现场排查。
    auto status = camera->open();
    if (!status.ok()) {
        std::fprintf(stderr, "open 失败: %s (code=0x%08X)\n", status.error().message.c_str(),
                     static_cast<unsigned>(status.error().vendor_code));
        return 1;
    }

    const auto info = camera->info();
    std::printf("已连接: %s %s serial=%s fw=%s ip=%s\n", info.vendor.c_str(), info.model.c_str(),
                info.serial.c_str(), info.firmware_version.c_str(), info.ip_address.c_str());

    status = camera->start();
    if (!status.ok()) {
        std::fprintf(stderr, "start 失败: %s\n", status.error().message.c_str());
        (void)camera->close();
        return 1;
    }

    // 预览窗口要拿到第一帧才知道真实分辨率（config.width/height 为 0 时由设备决定），
    // 所以窗口延迟到首帧再建。
    foray::platform::driver::hikvision::tools::Viewer viewer;
    bool viewer_ready = false;
    bool viewer_failed = false;

    const auto steady_now_ns = []() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    };

    std::uint64_t received = 0;
    std::uint64_t read_timeouts = 0;
    std::int64_t first_stamp_ns = 0;
    std::int64_t last_stamp_ns = 0;
    std::int64_t title_stamp_ns = 0;
    std::uint64_t title_frames = 0;
    const std::int64_t loop_begin_ns = steady_now_ns();

    while (g_stop_requested == 0 && (frames_wanted == 0U || received < frames_wanted)) {
        // 先抽事件再读帧：这样即使相机暂时不出图，关窗也能立刻生效。
        if (viewer_ready && !viewer.pump()) {
            std::printf("预览窗口已关闭，停止取流\n");
            break;
        }

        FrameLease<ImageFrame> lease;
        if (software_trigger) {
            // 软触发模式下每次取帧前主动触发一次，否则相机不会出图。
            const auto trigger_status = camera->trigger();
            if (!trigger_status.ok()) {
                std::fprintf(stderr, "软触发失败: %s\n", trigger_status.error().message.c_str());
                (void)camera->stop();
                print_stats(*camera);
                (void)camera->close();
                return 1;
            }
        }
        status = camera->read(lease, Duration::from_milliseconds(timeout_ms));
        if (!status.ok()) {
            // 超时/无数据是正常节拍（软触发尤其），继续等下一帧；只有真故障才退出。
            if (status.error().code == ErrorCode::kTimeout ||
                status.error().code == ErrorCode::kNoData) {
                ++read_timeouts;
                continue;
            }
            std::fprintf(stderr, "读帧失败: %s (code=0x%08X)\n", status.error().message.c_str(),
                         static_cast<unsigned>(status.error().vendor_code));
            (void)camera->stop();
            print_stats(*camera);
            (void)camera->close();
            return 1;
        }
        if (!lease) {
            continue;
        }
        const ImageFrame& frame = *lease;
        ++received;
        if (received == 1U) {
            first_stamp_ns = frame.stamp.nanoseconds_since_epoch();
        }
        last_stamp_ns = frame.stamp.nanoseconds_since_epoch();

        if (view_requested && !viewer_ready && !viewer_failed) {
            auto opened = viewer.open(static_cast<int>(frame.width), static_cast<int>(frame.height));
            if (!opened.ok()) {
                // 打不开窗口不该让整次取流作废（远程 SSH、无显示设备的现场很常见）。
                std::fprintf(stderr, "警告: 无法打开预览窗口: %s\n继续以文本模式运行\n",
                             opened.error().message.c_str());
                viewer_failed = true;
            } else {
                viewer_ready = true;
            }
        }
        if (viewer_ready) {
            // 帧尺寸变了（ROI/binning）时 show() 返回 false，这里只丢显示不影响取流。
            (void)viewer.show(frame);
        }

        // 预览模式下每帧一行会把终端刷爆，也拖慢取流节奏。
        const bool verbose = !viewer_ready || received <= 3U || (received % 60U) == 0U;
        if (verbose) {
            std::printf(
                "帧 %llu: %ux%u stride=%u seq=%llu dev_no=%u 曝光=%.1fus 增益=%.2fdB "
                "sync=%s 丢包=%u\n",
                static_cast<unsigned long long>(received), frame.width, frame.height, frame.stride,
                static_cast<unsigned long long>(frame.sequence), frame.device_frame_number,
                static_cast<double>(frame.exposure_time_us), static_cast<double>(frame.gain_db),
                sync_name(frame.sync), frame.lost_packets);
        }

        if (viewer_ready) {
            const std::int64_t now_ns = steady_now_ns();
            if (title_stamp_ns == 0) {
                title_stamp_ns = now_ns;
                title_frames = received;
            } else if (now_ns - title_stamp_ns >= kTitleIntervalNs) {
                const double fps = static_cast<double>(received - title_frames) * 1e9 /
                                   static_cast<double>(now_ns - title_stamp_ns);
                const auto live = camera->stats();
                char title[192] = {};
                std::snprintf(
                    title, sizeof(title),
                    "foray_hikvision_dump | %ux%u | %.1f Hz | 帧 %llu | 丢弃 %llu | 超时 %llu",
                    frame.width, frame.height, fps, static_cast<unsigned long long>(received),
                    static_cast<unsigned long long>(live.frames_dropped),
                    static_cast<unsigned long long>(live.timeouts));
                viewer.set_title(title);
                title_stamp_ns = now_ns;
                title_frames = received;
            }
        }
    }

    if (received > 1U && first_stamp_ns != 0 && last_stamp_ns > first_stamp_ns) {
        const double span_s = static_cast<double>(last_stamp_ns - first_stamp_ns) / 1e9;
        std::printf("平均帧率: %.2f Hz（%llu 帧 / %.3f s）\n",
                    static_cast<double>(received - 1U) / span_s,
                    static_cast<unsigned long long>(received - 1U), span_s);
    } else if (received > 1U) {
        // 没有可用设备时间戳（sync = kNoSync）时退回主机侧节拍，现场估算吞吐够用。
        const double span_s = static_cast<double>(steady_now_ns() - loop_begin_ns) / 1e9;
        if (span_s > 0.0) {
            std::printf("平均帧率（主机节拍）: %.2f Hz（%llu 帧 / %.3f s）\n",
                        static_cast<double>(received) / span_s,
                        static_cast<unsigned long long>(received), span_s);
        }
    }
    if (g_stop_requested != 0) {
        std::printf("收到中断信号，停止取流并释放设备\n");
    }
    if (read_timeouts != 0U) {
        std::printf("读超时 %llu 次（已跳过，不影响统计里的交付帧数）\n",
                    static_cast<unsigned long long>(read_timeouts));
    }

    (void)camera->stop();
    print_stats(*camera);
    (void)camera->close();
    return 0;
}
