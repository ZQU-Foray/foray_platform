#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/types.hpp"
#include "foray/platform/result/result.hpp"
#include "livox/livox_api.hpp"

namespace foray::platform::driver::livox::detail {

/// @brief Livox Ethernet 包类型。
enum class PacketKind {
    kUnsupported,
    kImu,
    kPointCloud,
};

/// @brief 固定 36 字节头部；字段偏移见 Livox 通信协议。
struct PacketHeader {
    std::uint8_t version = 0;
    std::uint16_t length = 0;
    /// @brief 包内点间隔，单位 0.1 µs。
    std::uint16_t time_interval = 0;
    std::uint16_t dot_num = 0;
    std::uint16_t udp_cnt = 0;
    std::uint8_t frame_cnt = 0;
    std::uint8_t data_type = 0;
    std::uint8_t time_type = 0;
    /// @brief 包内首点时间，单位 ns；语义由 `time_type` 决定。
    std::uint64_t timestamp_ns = 0;
};

/// @brief 解析固定头部；失败返回 `kInvalidArgument` 或 `kFrameTooLarge`。
Result<PacketHeader> parse_header(const std::uint8_t* data, std::size_t size);

/// @brief 返回点类型对应的字节数；未知类型返回 0。
std::size_t point_size(std::uint8_t data_type) noexcept;

/// @brief 包类型判定：0x00 为 IMU，0x01/0x02/0x03 为点云。
PacketKind packet_kind(std::uint8_t data_type) noexcept;

/**
 * @brief 解析点云载荷到 HAL 点结构。
 * @param header 已解析的头部。
 * @param packet 完整包（含 36 字节头）。
 * @param size 包长度，必须不小于 `header.length`。
 * @param line_num 线数，用于计算 `line = i % line_num`。
 * @param base_device_ns 帧首包时间，用于计算逐点相对时间。
 * @param out 输出缓冲，容量为 `capacity` 个点。
 * @param capacity 输出容量。
 * @param count 实际写入点数。
 */
Result<void> parse_points(const PacketHeader& header, const std::uint8_t* packet, std::size_t size,
                          std::uint8_t line_num, std::int64_t base_device_ns, hal::PointXYZITL* out,
                          std::size_t capacity, std::size_t& count);

/// @brief 解析 IMU 载荷；只填 gyro/acc，其余字段由调用方补齐。
Result<void> parse_imu(const PacketHeader& header, const std::uint8_t* packet, std::size_t size,
                       hal::ImuSample& out);

/// @brief `time_type` → HAL 同步状态。
hal::TimestampSync sync_from_time_type(std::uint8_t time_type) noexcept;

/// @brief 机型默认线数：Mid-360/360s = 4，HAP = 6，其余 = 1。
std::uint8_t default_line_num(std::uint8_t dev_type) noexcept;

/// @brief 机型名称，用于 `SensorInfo::model`。
const char* model_name(std::uint8_t dev_type) noexcept;

/**
 * @brief IPv4 点分字符串 → Livox handle。
 *
 * Livox 的 handle 按网络字节序打包：`a.b.c.d` → `(a << 24) | (b << 16) | (c << 8) | d`。
 */
std::uint32_t ipv4_to_handle(const std::string& ip) noexcept;

/// @brief handle → 点分 IPv4；非法 handle 返回空字符串。
std::string handle_to_ipv4(std::uint32_t handle);

} // namespace foray::platform::driver::livox::detail
