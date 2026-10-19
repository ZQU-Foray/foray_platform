#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
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
#include "foray/platform/time/duration.hpp"

namespace {

// 俯视图输出分辨率；不需要外部图像库，直接写 PPM。
constexpr int kImageSize = 800;
// 俯视图世界坐标范围上限，避免远点把比例压得过小。
constexpr double kMaxPlotRangeMeters = 50.0;
// 每个点绘制的像素块半径（0 表示 1×1）。
constexpr int kPointPixelRadius = 1;

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
};

void print_usage() {
    std::cout << "用法: foray_livox_dump --host-ip <ip> --lidar-ip <ip> [选项]\n"
              << "\n"
              << "读取 Livox 点云并写出 .pcd（完整 3D）与 .ppm（俯视图）。\n"
              << "\n"
              << "选项:\n"
              << "  --frames N          读取帧数（默认 1）\n"
              << "  --timeout-ms N      每帧读取超时（默认 15000）\n"
              << "  --out PREFIX        输出前缀（默认 livox_cloud）\n"
              << "  --config-json PATH  自定义 Livox SDK2 JSON\n"
              << "  --runtime-dir DIR   自动生成 JSON 的目录\n"
              << "  --self-test         生成合成点云，验证输出文件（不需要雷达）\n"
              << "  --help              显示本帮助\n";
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
            if (!next(value) || !parse_int(value, options.frames) || options.frames <= 0) {
                error = "--frames 必须是正整数";
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

    if (options.self_test) {
        std::vector<hal::PointXYZITL> points;
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

    std::vector<hal::PointXYZITL> points;
    const time::Duration timeout = time::Duration::from_milliseconds(options.timeout_ms);
    for (int frame_index = 0; frame_index < options.frames; ++frame_index) {
        hal::FrameLease<hal::PointCloudFrame> lease;
        Result<void> read = lidar->read(lease, timeout);
        if (!read.ok()) {
            std::cerr << "读取第 " << frame_index << " 帧失败: " << read.error().message << '\n';
            lidar->close();
            return 5;
        }

        std::cout << "frame " << frame_index << ": seq=" << lease->sequence
                  << " points=" << lease->count << " sync=" << hal::to_string(lease->sync)
                  << " base_time_ns=" << lease->base_time.nanoseconds_since_epoch() << '\n';
        if (lease->count > 0) {
            points.insert(points.end(), lease->points, lease->points + lease->count);
        }
    }

    const hal::SensorStats stats = lidar->stats();
    lidar->close();

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
