#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "foray/platform/driver/hikvision/camera.hpp"
#include "foray/platform/hal/camera.hpp"
#include "foray/platform/hal/frame_lease.hpp"
#include "foray/platform/time/duration.hpp"

namespace {

using foray::platform::hal::Camera;
using foray::platform::hal::CameraConfig;
using foray::platform::hal::FrameLease;
using foray::platform::hal::ImageFrame;
using foray::platform::time::Duration;

void print_usage(const char* program) {
    std::fprintf(stderr,
                 "用法: %s <serial | 型号 | 用户自定义名> [选项]\n"
                 "  --ip <地址>         按 IP 选择（GigE）\n"
                 "  --frames <n>        取多少帧后退出（默认 5，0 = 不限）\n"
                 "  --timeout-ms <n>    单帧读超时毫秒（默认 1000）\n"
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

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 2;
    }

    // 第一个位置参数按 serial 处理；其它选择条件走选项，避免歧义。
    CameraConfig config;
    const std::string selector = argv[1];
    config.selector.serial = selector;

    std::uint64_t frames_wanted = 5;
    std::int64_t timeout_ms = 1000;
    bool software_trigger = false;

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
        if (option == "--ip") {
            const char* text = next("--ip");
            if (text == nullptr) {
                return 2;
            }
            config.selector.serial.clear();
            config.selector.ip_address = text;
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
            if (text == nullptr || !parse_unsigned(text, value)) {
                std::fprintf(stderr, "--timeout-ms 需要一个非负整数\n");
                return 2;
            }
            timeout_ms = static_cast<std::int64_t>(value);
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

    std::uint64_t received = 0;
    std::int64_t first_stamp_ns = 0;
    std::int64_t last_stamp_ns = 0;
    while (frames_wanted == 0U || received < frames_wanted) {
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
        std::printf("帧 %llu: %ux%u stride=%u seq=%llu dev_no=%u 曝光=%.1fus 增益=%.2fdB "
                    "sync=%s 丢包=%u\n",
                    static_cast<unsigned long long>(received), frame.width, frame.height, frame.stride,
                    static_cast<unsigned long long>(frame.sequence), frame.device_frame_number,
                    static_cast<double>(frame.exposure_time_us), static_cast<double>(frame.gain_db),
                    sync_name(frame.sync), frame.lost_packets);
    }

    if (received > 1U && first_stamp_ns != 0 && last_stamp_ns > first_stamp_ns) {
        const double span_s = static_cast<double>(last_stamp_ns - first_stamp_ns) / 1e9;
        std::printf("平均帧率: %.2f Hz（%llu 帧 / %.3f s）\n",
                    static_cast<double>(received - 1U) / span_s,
                    static_cast<unsigned long long>(received - 1U), span_s);
    }

    (void)camera->stop();
    print_stats(*camera);
    (void)camera->close();
    return 0;
}
