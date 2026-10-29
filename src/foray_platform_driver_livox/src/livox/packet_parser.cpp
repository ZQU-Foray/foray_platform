#include "livox/packet_parser.hpp"

#include <arpa/inet.h>

#include <cmath>
#include <cstdio>
#include <cstring>

namespace foray::platform::driver::livox::detail {
namespace {

// 圆周率；球坐标 theta/phi 的单位为 0.01°（Livox 通信协议）。
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;

// 单位换算：高精度笛卡尔为 mm，低精度为 cm，球坐标为 mm。
constexpr double kMillimetersToMeters = 0.001;
constexpr double kCentimetersToMeters = 0.01;
// `time_interval` 的单位是 0.1 µs。
constexpr std::int64_t kTimeIntervalToNanoseconds = 100;

// 协议固定偏移（字节）。
constexpr std::size_t kOffsetVersion = 0;
constexpr std::size_t kOffsetLength = 1;
constexpr std::size_t kOffsetTimeInterval = 3;
constexpr std::size_t kOffsetDotNum = 5;
constexpr std::size_t kOffsetUdpCnt = 7;
constexpr std::size_t kOffsetFrameCnt = 9;
constexpr std::size_t kOffsetDataType = 10;
constexpr std::size_t kOffsetTimeType = 11;
constexpr std::size_t kOffsetTimestamp = 28;

// 数据类型（Livox 通信协议 / LivoxLidarPointDataType）。
constexpr std::uint8_t kDataTypeImu = 0x00;
constexpr std::uint8_t kDataTypeCartesianHigh = 0x01;
constexpr std::uint8_t kDataTypeCartesianLow = 0x02;
constexpr std::uint8_t kDataTypeSpherical = 0x03;
constexpr std::uint8_t kDataTypeDualEcho = 0x11;

// 机型 ID（LivoxLidarDeviceType）。
constexpr std::uint8_t kDeviceMid360 = 9;
constexpr std::uint8_t kDeviceIndustrialHap = 10;
constexpr std::uint8_t kDeviceHap = 15;
constexpr std::uint8_t kDeviceMid360s = 35;

// 各点类型的线上字节数（packed）。
constexpr std::size_t kHighPointBytes = 14;
constexpr std::size_t kLowPointBytes = 8;
constexpr std::size_t kSphericalPointBytes = 10;
constexpr std::size_t kImuBytes = 24;

#pragma pack(push, 1)
struct HighPoint {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;
    std::uint8_t reflectivity;
    std::uint8_t tag;
};

struct LowPoint {
    std::int16_t x;
    std::int16_t y;
    std::int16_t z;
    std::uint8_t reflectivity;
    std::uint8_t tag;
};

struct SphericalPoint {
    std::uint32_t depth;
    std::uint16_t theta;
    std::uint16_t phi;
    std::uint8_t reflectivity;
    std::uint8_t tag;
};

struct ImuPayload {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
};
#pragma pack(pop)

static_assert(sizeof(HighPoint) == kHighPointBytes, "Livox 高精度点布局必须为 packed");
static_assert(sizeof(LowPoint) == kLowPointBytes, "Livox 低精度点布局必须为 packed");
static_assert(sizeof(SphericalPoint) == kSphericalPointBytes, "Livox 球坐标点布局必须为 packed");
static_assert(sizeof(ImuPayload) == kImuBytes, "Livox IMU 载荷必须为 24 字节");

std::uint16_t read_u16(const std::uint8_t* data) noexcept {
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

std::uint64_t read_u64(const std::uint8_t* data) noexcept {
    std::uint64_t value = 0;
    for (std::size_t index = 0; index < 8; ++index) {
        value |= static_cast<std::uint64_t>(data[index]) << (8 * index);
    }
    return value;
}

Error invalid(const char* message) {
    return make_error(ErrorCode::kInvalidArgument, message);
}

} // namespace

Result<PacketHeader> parse_header(const std::uint8_t* data, std::size_t size) {
    if (data == nullptr) {
        return invalid("Livox 包头指针为空");
    }
    if (size < kPacketHeaderBytes) {
        return make_error(ErrorCode::kFrameTooLarge, "Livox 包长度小于固定头部");
    }

    PacketHeader header;
    header.version = data[kOffsetVersion];
    header.length = read_u16(data + kOffsetLength);
    header.time_interval = read_u16(data + kOffsetTimeInterval);
    header.dot_num = read_u16(data + kOffsetDotNum);
    header.udp_cnt = read_u16(data + kOffsetUdpCnt);
    header.frame_cnt = data[kOffsetFrameCnt];
    header.data_type = data[kOffsetDataType];
    header.time_type = data[kOffsetTimeType];
    header.timestamp_ns = read_u64(data + kOffsetTimestamp);

    if (header.length < kPacketHeaderBytes) {
        return make_error(ErrorCode::kInvalidArgument, "Livox 包头 length 字段非法",
                          static_cast<std::int32_t>(header.length));
    }
    if (header.length > size) {
        return make_error(ErrorCode::kFrameTooLarge, "Livox 包长度超过回调缓冲");
    }

    return Result<PacketHeader>(header);
}

std::size_t point_size(std::uint8_t data_type) noexcept {
    switch (data_type) {
    case kDataTypeCartesianHigh:
        return kHighPointBytes;
    case kDataTypeCartesianLow:
        return kLowPointBytes;
    case kDataTypeSpherical:
        return kSphericalPointBytes;
    default:
        return 0;
    }
}

PacketKind packet_kind(std::uint8_t data_type) noexcept {
    if (data_type == kDataTypeImu) {
        return PacketKind::kImu;
    }
    if (point_size(data_type) != 0) {
        return PacketKind::kPointCloud;
    }
    return PacketKind::kUnsupported;
}

Result<void> parse_points(const PacketHeader& header, const std::uint8_t* packet, std::size_t size,
                          std::uint8_t line_num, std::int64_t base_device_ns, hal::PointXYZITL* out,
                          std::size_t capacity, std::size_t& count) {
    count = 0;

    if (packet == nullptr || out == nullptr) {
        return invalid("Livox 点云输出缓冲为空");
    }
    if (header.length > size) {
        return make_error(ErrorCode::kFrameTooLarge, "Livox 点云包被截断");
    }
    if (header.dot_num > capacity) {
        return make_error(ErrorCode::kQueueFull, "Livox 点云输出缓冲不足",
                          static_cast<std::int32_t>(header.dot_num));
    }

    const std::size_t stride = point_size(header.data_type);
    if (stride == 0) {
        return make_error(ErrorCode::kUnsupportedFormat, "不支持的 Livox 点数据类型",
                          static_cast<std::int32_t>(header.data_type));
    }

    const std::size_t payload_bytes = header.length - kPacketHeaderBytes;
    const std::size_t required = static_cast<std::size_t>(header.dot_num) * stride;
    if (payload_bytes < required) {
        return make_error(ErrorCode::kInvalidArgument, "Livox 点云载荷长度与点数不匹配");
    }
    if (header.dot_num == 0) {
        return Result<void>();
    }

    const std::uint8_t* payload = packet + kPacketHeaderBytes;
    const std::uint8_t lines = line_num == 0 ? 1 : line_num;
    const std::int64_t packet_offset_ns =
        static_cast<std::int64_t>(header.timestamp_ns) - base_device_ns;

    for (std::size_t index = 0; index < header.dot_num; ++index) {
        // 先乘后除，避免逐点累计误差；time_interval 单位为 0.1 µs。
        const std::int64_t intra_ns =
            static_cast<std::int64_t>(index) * static_cast<std::int64_t>(header.time_interval) *
            kTimeIntervalToNanoseconds / static_cast<std::int64_t>(header.dot_num);

        hal::PointXYZITL& point = out[index];
        point = hal::PointXYZITL{};
        point.time_offset_ns = static_cast<std::int32_t>(packet_offset_ns + intra_ns);
        point.line = static_cast<std::uint8_t>(index % lines);

        const std::uint8_t* raw = payload + index * stride;
        switch (header.data_type) {
        case kDataTypeCartesianHigh: {
            HighPoint value{};
            std::memcpy(&value, raw, sizeof(value));
            point.x = static_cast<float>(value.x) * static_cast<float>(kMillimetersToMeters);
            point.y = static_cast<float>(value.y) * static_cast<float>(kMillimetersToMeters);
            point.z = static_cast<float>(value.z) * static_cast<float>(kMillimetersToMeters);
            point.intensity = static_cast<float>(value.reflectivity);
            point.tag = value.tag;
            break;
        }
        case kDataTypeCartesianLow: {
            LowPoint value{};
            std::memcpy(&value, raw, sizeof(value));
            point.x = static_cast<float>(value.x) * static_cast<float>(kCentimetersToMeters);
            point.y = static_cast<float>(value.y) * static_cast<float>(kCentimetersToMeters);
            point.z = static_cast<float>(value.z) * static_cast<float>(kCentimetersToMeters);
            point.intensity = static_cast<float>(value.reflectivity);
            point.tag = value.tag;
            break;
        }
        case kDataTypeSpherical: {
            SphericalPoint value{};
            std::memcpy(&value, raw, sizeof(value));
            const double depth = static_cast<double>(value.depth) * kMillimetersToMeters;
            const double theta = static_cast<double>(value.theta) * kDegreesToRadians / 100.0;
            const double phi = static_cast<double>(value.phi) * kDegreesToRadians / 100.0;
            point.x = static_cast<float>(depth * std::cos(theta) * std::cos(phi));
            point.y = static_cast<float>(depth * std::sin(theta) * std::cos(phi));
            point.z = static_cast<float>(depth * std::sin(phi));
            point.intensity = static_cast<float>(value.reflectivity);
            point.tag = value.tag;
            break;
        }
        default:
            return make_error(ErrorCode::kUnsupportedFormat, "不支持的 Livox 点数据类型",
                              static_cast<std::int32_t>(header.data_type));
        }
    }

    count = header.dot_num;
    return Result<void>();
}

Result<void> parse_imu(const PacketHeader& header, const std::uint8_t* packet, std::size_t size,
                       hal::ImuSample& out) {
    if (packet == nullptr) {
        return invalid("Livox IMU 包指针为空");
    }
    if (header.length > size || header.length < kPacketHeaderBytes + kImuBytes) {
        return make_error(ErrorCode::kInvalidArgument, "Livox IMU 包长度非法");
    }

    ImuPayload value{};
    std::memcpy(&value, packet + kPacketHeaderBytes, sizeof(value));
    out.gyro_x = value.gyro_x;
    out.gyro_y = value.gyro_y;
    out.gyro_z = value.gyro_z;
    out.acc_x = value.acc_x;
    out.acc_y = value.acc_y;
    out.acc_z = value.acc_z;
    return Result<void>();
}

hal::TimestampSync sync_from_time_type(std::uint8_t time_type) noexcept {
    switch (time_type) {
    case 0:
        // 设备开机时间，不保证 Unix 纪元。
        return hal::TimestampSync::kNoSync;
    case 1:
        return hal::TimestampSync::kPtp;
    case 2:
        return hal::TimestampSync::kGpsPps;
    case 3:
    case 4:
        // PPS：只有秒脉冲基准，不保证绝对纪元。
        return hal::TimestampSync::kPps;
    default:
        return hal::TimestampSync::kUnknown;
    }
}

std::uint8_t default_line_num(std::uint8_t dev_type) noexcept {
    switch (dev_type) {
    case kDeviceMid360:
    case kDeviceMid360s:
        return 4;
    case kDeviceIndustrialHap:
    case kDeviceHap:
        return 6;
    default:
        return 1;
    }
}

const char* model_name(std::uint8_t dev_type) noexcept {
    switch (dev_type) {
    case kDeviceMid360:
        return "Livox Mid-360";
    case kDeviceMid360s:
        return "Livox Mid-360s";
    case kDeviceIndustrialHap:
    case kDeviceHap:
        return "Livox HAP";
    default:
        return "Livox";
    }
}

std::uint32_t ipv4_to_handle(const std::string& ip) noexcept {
    // SDK 的 handle 就是 sockaddr_in::sin_addr.s_addr 的原始数值；
    // 在小端机上它看起来是字节反序的，不能用 (a<<24)|... 手工拼接。
    struct in_addr address{};
    if (inet_pton(AF_INET, ip.c_str(), &address) != 1) {
        return 0;
    }
    return address.s_addr;
}

std::string handle_to_ipv4(std::uint32_t handle) {
    if (handle == 0) {
        return {};
    }
    struct in_addr address{};
    address.s_addr = handle;
    char buffer[INET_ADDRSTRLEN] = {};
    if (inet_ntop(AF_INET, &address, buffer, sizeof(buffer)) == nullptr) {
        return {};
    }
    return std::string(buffer);
}

} // namespace foray::platform::driver::livox::detail
