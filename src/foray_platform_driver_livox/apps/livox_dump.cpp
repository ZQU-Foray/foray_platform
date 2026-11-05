#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "foray/platform/driver/livox/lidar.hpp"
#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/sensor_stats.hpp"
#include "foray/platform/time/clock.hpp"
#include "foray/platform/time/duration.hpp"

namespace {

namespace hal = foray::platform::hal;
namespace time = foray::platform::time;

// 俯视图输出分辨率；不需要外部图像库，直接写 PPM。
constexpr int kImageSize = 800;
// 俯视图世界坐标范围上限，避免远点把比例压得过小。
constexpr double kMaxPlotRangeMeters = 50.0;
// 每个点绘制的像素块半径（0 表示 1×1）。
constexpr int kPointPixelRadius = 1;
// 流式模式下单点文本行的临时缓冲；足够容纳全部 7 列（含 9 位小数时间）。
constexpr std::size_t kStreamLineCapacity = 160;

// 流式输出的表头与列序：下游可跳过 '#' 开头的行。
constexpr const char* kStreamHeader =
    "# foray_livox_dump stream v1 fields=frame x y z intensity time_s line\n";

// SIGINT / SIGTERM 只置标志，由主循环决定何时收尾（不在信号处理里做 I/O）。
volatile std::sig_atomic_t g_stop_requested = 0;

extern "C" void handle_stop_signal(int /*signal_number*/) {
    g_stop_requested = 1;
}

struct Options {
    std::string host_ip;
    std::string lidar_ip;
    std::string config_json;
    std::string runtime_dir;
    std::string output_prefix = "livox_cloud";
    int frames = 1;
    int timeout_ms = 15000;
    bool help = false;
    bool self_test = false;
    bool stream = false;
};

/// @brief 流式输出的累计统计（只用于收尾汇报）。
struct StreamSummary {
    std::uint64_t frames = 0;
    std::uint64_t points = 0;
    std::uint64_t bytes = 0;
    time::Timestamp start_steady;
    time::Timestamp last_frame_steady;
};

/// 流式模式下 stdout 会被 SDK 日志占用，点云数据改走这个保存的 fd。
int g_stream_fd = -1;

/// @brief 全量写；EINTR 重试，部分写继续。失败返回 false 并把 errno 写进 error。
bool write_all(const char* data, std::size_t size, int& error) {
    std::size_t offset = 0;
    while (offset < size) {
        const ssize_t written = ::write(g_stream_fd, data + offset, size - offset);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            error = errno;
            return false;
        }
        offset += static_cast<std::size_t>(written);
    }
    return true;
}

/// @brief 一帧流式写入的结果；`ok == false` 时 `error` 为 errno。
struct StreamWriteResult {
    std::size_t bytes = 0;
    bool ok = false;
    int error = 0;
};

/// @brief 把一帧点云写入 stdout；失败时保留 errno（区分 EPIPE 与真写失败）。
StreamWriteResult stream_frame(std::uint64_t frame_index, const hal::PointCloudFrame& frame,
                               std::string& buffer) {
    StreamWriteResult result;
    buffer.clear();
    buffer.reserve(frame.count * kStreamLineCapacity);

    char line[kStreamLineCapacity];
    for (std::size_t index = 0; index < frame.count; ++index) {
        const hal::PointXYZITL& point = frame.points[index];
        const int length = std::snprintf(
            line, sizeof(line), "%llu %.4f %.4f %.4f %.1f %.9f %u\n",
            static_cast<unsigned long long>(frame_index), static_cast<double>(point.x),
            static_cast<double>(point.y), static_cast<double>(point.z),
            static_cast<double>(point.intensity),
            static_cast<double>(point.time_offset_ns) / 1'000'000'000.0,
            static_cast<unsigned>(point.line));
        if (length <= 0) {
            continue;
        }
        buffer.append(line, static_cast<std::size_t>(length));
    }

    if (!write_all(buffer.data(), buffer.size(), result.error)) {
        return result;
    }
    result.bytes = buffer.size();
    result.ok = true;
    return result;
}

const char* errno_text(int error) {
    return std::strerror(error == 0 ? EIO : error);
}

void report_stream_frame(std::uint64_t frame_index, const hal::PointCloudFrame& frame,
                         std::uint64_t frame_bytes, double interval_ms, double fps) {
    std::fprintf(stderr,
                 "[frame %llu] seq=%llu points=%zu dropped_packets=%u sync=%s "
                 "frame_bytes=%llu interval_ms=%.1f fps=%.1f\n",
                 static_cast<unsigned long long>(frame_index),
                 static_cast<unsigned long long>(frame.sequence), frame.count,
                 static_cast<unsigned>(frame.dropped_packets),
                 hal::to_string(frame.sync),
                 static_cast<unsigned long long>(frame_bytes), interval_ms, fps);
    std::fflush(stderr);
}

int write_stream_header() {
    int error = 0;
    if (!write_all(kStreamHeader, std::strlen(kStreamHeader), error)) {
        return error;
    }
    return 0;
}

void print_usage() {
    std::cout << "用法: foray_livox_dump --host-ip <ip> --lidar-ip <ip> [选项]\n"
              << "\n"
              << "读取 Livox 点云并写出 .pcd（完整 3D）与 .ppm（俯视图）。\n"
              << "\n"
              << "选项:\n"
              << "  --frames N          读取帧数（默认 1；与 --stream 同用时 0 表示持续输出）\n"
              << "  --timeout-ms N      每帧读取超时（默认 15000）\n"
              << "  --out PREFIX        输出前缀（默认 livox_cloud）\n"
              << "  --stream            点云逐点流式写入 stdout（每帧刷新；不写 PCD/PPM）\n"
              << "  --config-json PATH  自定义 Livox SDK2 JSON\n"
              << "  --runtime-dir DIR   自动生成 JSON 的目录\n"
              << "  --self-test         生成合成点云，验证输出文件（不需要雷达）\n"
              << "  --help              显示本帮助\n"
              << "\n"
              << "流式格式（stdout，文本按行）:\n"
              << "  # foray_livox_dump stream v1 fields=frame x y z intensity time_s line\n"
              << "  <frame> <x> <y> <z> <intensity> <time_s> <line>\n"
              << "  每帧统计写 stderr，因此可直接管道给下游程序。\n";
}

bool parse_int(const std::string& text, int& value) {
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        value = parsed;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parse_options(int argc, char** argv, Options& options, std::string& error) {
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto next = [&](std::string& target) -> bool {
            if (index + 1 >= argc) {
                error = argument + " 缺少参数";
                return false;
            }
            target = argv[++index];
            return true;
        };

        if (argument == "--help" || argument == "-h") {
            options.help = true;
        } else if (argument == "--self-test") {
            options.self_test = true;
        } else if (argument == "--stream") {
            options.stream = true;
        } else if (argument == "--host-ip") {
            if (!next(options.host_ip)) {
                return false;
            }
        } else if (argument == "--lidar-ip") {
            if (!next(options.lidar_ip)) {
                return false;
            }
        } else if (argument == "--config-json") {
            if (!next(options.config_json)) {
                return false;
            }
        } else if (argument == "--runtime-dir") {
            if (!next(options.runtime_dir)) {
                return false;
            }
        } else if (argument == "--out") {
            if (!next(options.output_prefix)) {
                return false;
            }
        } else if (argument == "--frames") {
            std::string value;
            if (!next(value) || !parse_int(value, options.frames) || options.frames < 0) {
                error = "--frames 必须是非负整数（0 仅在 --stream 下表示持续输出）";
                return false;
            }
        } else if (argument == "--timeout-ms") {
            std::string value;
            if (!next(value) || !parse_int(value, options.timeout_ms) || options.timeout_ms <= 0) {
                error = "--timeout-ms 必须是正整数";
                return false;
            }
        } else {
            error = "未知参数: " + argument;
            return false;
        }
    }

    if (!options.help && !options.self_test) {
        if (options.frames == 0 && !options.stream) {
            error = "--frames 0 只在 --stream 模式下有效";
            return false;
        }
        if (options.lidar_ip.empty()) {
            error = "必须提供 --lidar-ip";
            return false;
        }
        if (options.host_ip.empty() && options.config_json.empty()) {
            error = "必须提供 --host-ip，或用 --config-json 指定完整配置";
            return false;
        }
    }
    return true;
}

bool write_pcd(const std::string& path,
               const std::vector<foray::platform::hal::PointXYZITL>& points) {
    std::ofstream stream(path, std::ios::out | std::ios::trunc);
    if (!stream) {
        return false;
    }

    stream << "# .PCD v0.7 - Point Cloud Data file format\n"
           << "VERSION 0.7\n"
           << "FIELDS x y z intensity time line\n"
           << "SIZE 4 4 4 4 4 1\n"
           << "TYPE F F F F F U\n"
           << "COUNT 1 1 1 1 1 1\n"
           << "WIDTH " << points.size() << "\n"
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << points.size() << "\n"
           << "DATA ascii\n";

    stream << std::fixed << std::setprecision(4);
    for (const auto& point : points) {
        stream << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << ' '
               << std::setprecision(9)
               << static_cast<double>(point.time_offset_ns) / 1'000'000'000.0 << ' '
               << static_cast<unsigned>(point.line) << '\n';
        stream << std::setprecision(4);
    }
    return static_cast<bool>(stream);
}

void height_color(double normalized, std::uint8_t& red, std::uint8_t& green, std::uint8_t& blue) {
    normalized = std::clamp(normalized, 0.0, 1.0);
    if (normalized < 0.25) {
        red = 0;
        green = static_cast<std::uint8_t>(normalized * 4.0 * 255.0);
        blue = 255;
    } else if (normalized < 0.5) {
        red = 0;
        green = 255;
        blue = static_cast<std::uint8_t>((1.0 - (normalized - 0.25) * 4.0) * 255.0);
    } else if (normalized < 0.75) {
        red = static_cast<std::uint8_t>((normalized - 0.5) * 4.0 * 255.0);
        green = 255;
        blue = 0;
    } else {
        red = 255;
        green = static_cast<std::uint8_t>((1.0 - (normalized - 0.75) * 4.0) * 255.0);
        blue = 0;
    }
}

bool write_ppm(const std::string& path,
               const std::vector<foray::platform::hal::PointXYZITL>& points) {
    if (points.empty()) {
        return false;
    }

    double max_abs_xy = 0.0;
    double min_z = points.front().z;
    double max_z = points.front().z;
    for (const auto& point : points) {
        const double point_abs_xy =
            static_cast<double>(std::max(std::abs(point.x), std::abs(point.y)));
        max_abs_xy = std::max(max_abs_xy, point_abs_xy);
        min_z = std::min(min_z, static_cast<double>(point.z));
        max_z = std::max(max_z, static_cast<double>(point.z));
    }
    max_abs_xy = std::clamp(max_abs_xy, 0.5, kMaxPlotRangeMeters);
    const double z_range = std::max(1e-3, max_z - min_z);

    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(kImageSize) * kImageSize * 3, 0);
    for (std::size_t index = 0; index < pixels.size(); index += 3) {
        pixels[index] = 10;
        pixels[index + 1] = 10;
        pixels[index + 2] = 20;
    }

    const double scale = (kImageSize / 2.0 - 2.0) / max_abs_xy;
    const int center = kImageSize / 2;
    for (const auto& point : points) {
        const int column = center + static_cast<int>(point.x * scale);
        const int row = center - static_cast<int>(point.y * scale);
        if (column < 0 || column >= kImageSize || row < 0 || row >= kImageSize) {
            continue;
        }

        std::uint8_t red = 0;
        std::uint8_t green = 0;
        std::uint8_t blue = 0;
        height_color((point.z - min_z) / z_range, red, green, blue);

        for (int dy = -kPointPixelRadius; dy <= kPointPixelRadius; ++dy) {
            for (int dx = -kPointPixelRadius; dx <= kPointPixelRadius; ++dx) {
                const int x = column + dx;
                const int y = row + dy;
                if (x < 0 || x >= kImageSize || y < 0 || y >= kImageSize) {
                    continue;
                }
                const std::size_t offset = (static_cast<std::size_t>(y) * kImageSize + x) * 3;
                pixels[offset] = red;
                pixels[offset + 1] = green;
                pixels[offset + 2] = blue;
            }
        }
    }

    std::ofstream stream(path, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!stream) {
        return false;
    }
    stream << "P6\n" << kImageSize << ' ' << kImageSize << "\n255\n";
    stream.write(reinterpret_cast<const char*>(pixels.data()),
                 static_cast<std::streamsize>(pixels.size()));
    return static_cast<bool>(stream);
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    std::string error;
    if (!parse_options(argc, argv, options, error)) {
        std::cerr << "参数错误: " << error << "\n\n";
        print_usage();
        return 1;
    }
    if (options.help) {
        print_usage();
        return 0;
    }

    using namespace foray::platform;

    // 信号只置停止标志；写剩余文件与汇总在主循环外完成（信号处理里不做 I/O）。
    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);
#ifdef SIGPIPE
    // 下游管道提前关闭时不要被 SIGPIPE 杀掉，改为检测写失败后优雅退出。
    std::signal(SIGPIPE, SIG_IGN);
#endif

    if (options.stream) {
        // SDK 的 console 日志写 stdout，和点云流会混在一起；保存真实 stdout，
        // 把 fd 1 指向 /dev/null，点云改走 g_stream_fd。
        const int saved = ::dup(STDOUT_FILENO);
        if (saved < 0) {
            std::cerr << "保存 stdout 失败: " << errno_text(errno) << '\n';
            return 7;
        }
        g_stream_fd = saved;
        if (std::freopen("/dev/null", "w", stdout) == nullptr) {
            std::cerr << "重定向 stdout 失败: " << errno_text(errno) << '\n';
            return 7;
        }
    }

    const time::SteadyClock steady_clock;
    std::vector<hal::PointXYZITL> points;
    std::string stream_buffer;
    StreamSummary summary;

    if (options.self_test) {
        constexpr int kSyntheticPoints = 20000;
        points.reserve(kSyntheticPoints);
        for (int index = 0; index < kSyntheticPoints; ++index) {
            hal::PointXYZITL point;
            const double angle = index * 0.002;
            const double radius = 1.0 + 8.0 * static_cast<double>(index) / kSyntheticPoints;
            point.x = static_cast<float>(radius * std::cos(angle));
            point.y = static_cast<float>(radius * std::sin(angle));
            point.z = static_cast<float>(1.5 * std::sin(angle * 3.0));
            point.intensity = static_cast<float>(index % 256);
            point.time_offset_ns = index * 1000;
            point.line = static_cast<std::uint8_t>(index % 4);
            points.push_back(point);
        }

        if (options.stream) {
            const int header_error = write_stream_header();
            if (header_error != 0) {
                if (header_error == EPIPE) {
                    std::cerr << "stdout 已关闭（下游提前结束），已停止流式输出\n";
                    return 0;
                }
                std::cerr << "流式写出表头失败: " << errno_text(header_error) << '\n';
                return 7;
            }
            hal::PointCloudFrame frame;
            frame.points = points.data();
            frame.count = points.size();
            frame.sync = hal::TimestampSync::kNoSync;
            const StreamWriteResult stream_result = stream_frame(0, frame, stream_buffer);
            if (!stream_result.ok) {
                // 下游只读一行就退出（如 `| head -1`）不算错误。
                if (stream_result.error == EPIPE) {
                    std::cerr << "stdout 已关闭（下游提前结束），已停止流式输出\n";
                    return 0;
                }
                std::cerr << "流式写出合成点云失败: " << errno_text(stream_result.error) << '\n';
                return 7;
            }
            std::cerr << "self-test(stream): " << points.size() << " 点, " << stream_result.bytes
                      << " 字节 -> stdout\n";
            return 0;
        }

        const std::string pcd_path = options.output_prefix + ".pcd";
        const std::string ppm_path = options.output_prefix + ".ppm";
        if (!write_pcd(pcd_path, points) || !write_ppm(ppm_path, points)) {
            std::cerr << "写出合成点云失败\n";
            return 7;
        }
        std::cout << "self-test: " << points.size() << " 点\n"
                  << "PCD: " << pcd_path << "\n"
                  << "PPM: " << ppm_path << "\n";
        return 0;
    }

    hal::LidarConfig config;
    config.selector.ip_address = options.lidar_ip;
    config.host_ip = options.host_ip;
    config.config_json_path = options.config_json;
    config.runtime_dir = options.runtime_dir;
    config.frame_id = "livox_dump";
    config.frame_timeout_ms = 200;
    config.point_buffer_capacity = 200000;

    Result<std::unique_ptr<hal::Lidar>> created = driver::livox::make_lidar(config);
    if (!created.ok()) {
        std::cerr << "make_lidar 失败: " << created.error().message << '\n';
        return 2;
    }
    std::unique_ptr<hal::Lidar> lidar = std::move(created.value());

    Result<void> opened = lidar->open();
    if (!opened.ok()) {
        std::cerr << "open 失败: " << opened.error().message << '\n';
        return 3;
    }
    Result<void> started = lidar->start();
    if (!started.ok()) {
        std::cerr << "start 失败: " << started.error().message << '\n';
        lidar->close();
        return 4;
    }

    const time::Duration timeout = time::Duration::from_milliseconds(options.timeout_ms);
    // --stream --frames 0：持续输出，直到 Ctrl-C / SIGTERM。
    const bool unbounded = options.stream && options.frames == 0;
    const std::uint64_t frame_limit = static_cast<std::uint64_t>(options.frames);
    std::uint64_t frame_index = 0;
    int stdout_error = 0;

    if (options.stream) {
        summary.start_steady = steady_clock.now();
        stdout_error = write_stream_header();
        if (stdout_error != 0) {
            // 表头都写不出去，后面每帧也写不出去；按同一套收尾逻辑处理。
            if (stdout_error == EPIPE) {
                std::cerr << "stdout 已关闭（下游提前结束），已停止流式输出\n";
                lidar->close();
                return 0;
            }
            std::cerr << "流式写出表头失败: " << errno_text(stdout_error) << '\n';
            lidar->close();
            return 7;
        }
    }

    while (!g_stop_requested && (unbounded || frame_index < frame_limit)) {
        hal::FrameLease<hal::PointCloudFrame> lease;
        Result<void> read = lidar->read(lease, timeout);
        if (!read.ok()) {
            if (g_stop_requested) {
                break;
            }
            const hal::SensorStats stats = lidar->stats();
            std::cerr << "读取第 " << frame_index << " 帧失败: " << read.error().message << '\n'
                      << "stats: packets=" << stats.packets_received
                      << " packets_dropped=" << stats.packets_dropped
                      << " frames_received=" << stats.frames_received
                      << " frames_dropped=" << stats.frames_dropped
                      << " timeouts=" << stats.timeouts << '\n';
            lidar->close();
            return 5;
        }

        const time::Timestamp frame_steady = steady_clock.now();
        const double interval_ms =
            summary.frames == 0
                ? 0.0
                : static_cast<double>((frame_steady - summary.last_frame_steady).nanoseconds()) /
                      1'000'000.0;
        summary.frames += 1;
        summary.points += lease->count;
        summary.last_frame_steady = frame_steady;

        if (options.stream) {
            const StreamWriteResult write = stream_frame(frame_index, *lease, stream_buffer);
            if (!write.ok) {
                stdout_error = write.error;
                break;
            }
            summary.bytes += write.bytes;
            const double elapsed_s =
                static_cast<double>((frame_steady - summary.start_steady).nanoseconds()) /
                1'000'000'000.0;
            const double fps =
                elapsed_s > 0.0 ? static_cast<double>(summary.frames) / elapsed_s : 0.0;
            report_stream_frame(frame_index, *lease, write.bytes, interval_ms, fps);
        } else {
            std::cout << "frame " << frame_index << ": seq=" << lease->sequence
                      << " points=" << lease->count << " sync=" << hal::to_string(lease->sync)
                      << " base_time_ns=" << lease->base_time.nanoseconds_since_epoch() << '\n';
            if (lease->count > 0) {
                points.insert(points.end(), lease->points, lease->points + lease->count);
            }
        }
        ++frame_index;
    }

    const hal::SensorStats stats = lidar->stats();
    lidar->close();

    if (options.stream) {
        const time::Timestamp now = steady_clock.now();
        const double elapsed_s =
            static_cast<double>((now - summary.start_steady).nanoseconds()) / 1'000'000'000.0;
        const double fps =
            elapsed_s > 0.0 ? static_cast<double>(summary.frames) / elapsed_s : 0.0;
        const double bytes_per_second =
            elapsed_s > 0.0 ? static_cast<double>(summary.bytes) / elapsed_s : 0.0;

        std::cerr << "stream 汇总: frames=" << summary.frames << " points=" << summary.points
                  << " bytes=" << summary.bytes << " elapsed_s=" << std::fixed
                  << std::setprecision(3) << elapsed_s << " avg_fps=" << std::setprecision(2) << fps
                  << " throughput_MBps=" << std::setprecision(2)
                  << bytes_per_second / 1'000'000.0 << std::defaultfloat << '\n'
                  << "stats: packets=" << stats.packets_received
                  << " packets_dropped=" << stats.packets_dropped
                  << " frames_received=" << stats.frames_received
                  << " frames_dropped=" << stats.frames_dropped
                  << " timeouts=" << stats.timeouts << '\n';
        if (stdout_error == EPIPE) {
            std::cerr << "stdout 已关闭（下游提前结束），已停止流式输出\n";
        } else if (stdout_error != 0) {
            // 真写失败优先于「没读到点」的判定，避免掩盖磁盘满类问题。
            std::cerr << "stdout 写入失败: " << errno_text(stdout_error) << '\n';
            return 7;
        }
        if (summary.points == 0) {
            std::cerr << "没有读到任何点\n";
            return 6;
        }
        return 0;
    }

    if (points.empty()) {
        std::cerr << "没有读到任何点\n";
        return 6;
    }

    const std::string pcd_path = options.output_prefix + ".pcd";
    const std::string ppm_path = options.output_prefix + ".ppm";
    const bool pcd_ok = write_pcd(pcd_path, points);
    const bool ppm_ok = write_ppm(ppm_path, points);

    std::cout << "共 " << points.size() << " 点\n"
              << "PCD: " << pcd_path << (pcd_ok ? " (ok)" : " (失败)") << '\n'
              << "PPM: " << ppm_path << (ppm_ok ? " (ok)" : " (失败)") << '\n'
              << "packets=" << stats.packets_received
              << " packets_dropped=" << stats.packets_dropped
              << " frames_received=" << stats.frames_received << '\n';

    if (!pcd_ok || !ppm_ok) {
        return 7;
    }
    return 0;
}
