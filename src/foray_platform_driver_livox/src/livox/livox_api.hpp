#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "foray/platform/result/result.hpp"

namespace foray::platform::driver::livox::detail {

/// @brief Livox 单个 UDP 包的最大长度。
///
/// 取标准以太网 MTU 1500 字节；超过该长度的包由回调直接丢弃并计数，
/// 避免环形缓冲无限膨胀。
constexpr std::size_t kMaxPacketBytes = 1500;

/// @brief Livox 点云/IMU 包的固定头部长度。
constexpr std::size_t kPacketHeaderBytes = 36;

/// @brief SDK 支持的雷达数量上限（与 `kMaxLidarCount` 一致）。
constexpr std::size_t kMaxLidars = 32;

/// @brief SDK 回调给出的原始包视图；指针只在回调期间有效。
struct PacketView {
    std::uint32_t handle = 0;
    std::uint8_t dev_type = 0;
    const std::uint8_t* data = nullptr;
    std::uint32_t size = 0;
};

/// @brief 设备上线/信息变化视图；字符串指针只在回调期间有效。
struct InfoView {
    std::uint32_t handle = 0;
    std::uint8_t dev_type = 0;
    const char* serial = nullptr;
    const char* ip_address = nullptr;
};

/// @brief 从雷达读回的关键配置；用于写后校验。
struct LidarInternalInfo {
    /// @brief `kKeyLidarIpCfg(0x0004)` 解析出的 IP；未下发过配置时为空。
    std::string ip_address;
    /// @brief 回包里的参数条目数；0 表示没有解析到有效内容。
    std::uint16_t param_count = 0;
};

using PacketCallback = void (*)(void* user, const PacketView& packet);
using InfoCallback = void (*)(void* user, const InfoView& info);

/**
 * @brief Livox SDK2 的最小函数表。
 *
 * 驱动逻辑只依赖该接口；真实实现把 SDK 类型限制在 `livox_api_real.cpp`，
 * 无 SDK 环境使用 stub 或测试 mock。
 */
class LivoxApi {
  public:
    virtual ~LivoxApi() = default;

    /// @brief 初始化进程级 SDK；`json_path` 为设备/网络配置。
    virtual Result<void> init(const std::string& json_path, const std::string& host_ip) = 0;

    /// @brief 启动设备扫描线程。
    virtual void start() = 0;

    /// @brief 反初始化进程级 SDK；返回后不得再有回调在飞行。
    virtual void uninit() = 0;

    virtual void set_point_cloud_callback(PacketCallback callback, void* user) = 0;
    virtual void set_imu_callback(PacketCallback callback, void* user) = 0;
    virtual void set_info_change_callback(InfoCallback callback, void* user) = 0;

    virtual Result<void> set_work_mode(std::uint32_t handle, std::uint8_t mode) = 0;
    virtual Result<void> set_point_data_type(std::uint32_t handle, std::uint8_t type) = 0;
    virtual Result<void> set_scan_pattern(std::uint32_t handle, std::uint8_t pattern) = 0;
    virtual Result<void> set_dual_emit(std::uint32_t handle, bool enable) = 0;
    virtual Result<void> request_reset(std::uint32_t handle) = 0;

    /// @brief 下发雷达 IP（`kKeyLidarIpCfg`）；地址为点分十进制字符串。
    virtual Result<void> set_lidar_ip(std::uint32_t handle, const std::string& ip,
                                      const std::string& net_mask,
                                      const std::string& gateway) = 0;

    /// @brief 下发安装姿态（角度 + 毫米级安装偏移）。
    virtual Result<void> set_install_attitude(std::uint32_t handle, float roll_deg,
                                              float pitch_deg, float yaw_deg, std::int32_t x_mm,
                                              std::int32_t y_mm, std::int32_t z_mm) = 0;

    /// @brief 下发 PPS 同步模式；Mid-360s 不支持该指令（SDK 返回 kNotSupported）。
    virtual Result<void> set_pps_sync_mode(std::uint32_t handle, std::uint8_t mode) = 0;

    /// @brief 下发点云/IMU 的目标主机地址与端口。
    virtual Result<void> set_host_port_cfg(std::uint32_t handle, const std::string& host_ip,
                                           std::uint16_t host_port,
                                           std::uint16_t lidar_port) = 0;

    /// @brief 下发 RMC/GPRMC 授时语句。
    virtual Result<void> set_rmc_sync_time(std::uint32_t handle, const std::string& rmc) = 0;

    /// @brief 读回关键配置（IP、参数条目数），用于写后校验。
    virtual Result<LidarInternalInfo> query_internal_info(std::uint32_t handle) = 0;
};

/**
 * @brief 创建 SDK 实现。
 *
 * - 编译期启用 SDK：返回真实实现；
 * - 未启用：返回 stub，所有操作返回 `ErrorCode::kSdkUnavailable`。
 */
std::shared_ptr<LivoxApi> make_livox_api();

/// @brief 环形缓冲中的原始包槽位；带主机接收时刻。
struct PacketSlot {
    std::uint32_t handle = 0;
    std::uint8_t dev_type = 0;
    std::uint16_t length = 0;
    std::uint64_t receive_steady_ns = 0;
    std::uint64_t receive_wall_ns = 0;
    std::array<std::uint8_t, kMaxPacketBytes> data{};
};

} // namespace foray::platform::driver::livox::detail
