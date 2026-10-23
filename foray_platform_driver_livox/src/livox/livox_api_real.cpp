#include "livox/livox_api.hpp"

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <string>

namespace foray::platform::driver::livox::detail {
namespace {

// SDK2 的回调是按类型全局注册的（每个进程一份），与 `LivoxApi` 实例无关。
PacketCallback g_point_callback = nullptr;
void* g_point_user = nullptr;
PacketCallback g_imu_callback = nullptr;
void* g_imu_user = nullptr;
InfoCallback g_info_callback = nullptr;
void* g_info_user = nullptr;

void point_cloud_trampoline(const std::uint32_t handle, const std::uint8_t dev_type,
                            LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (g_point_callback == nullptr || data == nullptr) {
        return;
    }
    PacketView view;
    view.handle = handle;
    view.dev_type = dev_type;
    view.data = reinterpret_cast<const std::uint8_t*>(data);
    // SDK 保证回调期间 `data` 指向完整包；`length` 是头 + 载荷的总长度。
    view.size = data->length;
    g_point_callback(g_point_user, view);
}

void imu_trampoline(const std::uint32_t handle, const std::uint8_t dev_type,
                    LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (g_imu_callback == nullptr || data == nullptr) {
        return;
    }
    PacketView view;
    view.handle = handle;
    view.dev_type = dev_type;
    view.data = reinterpret_cast<const std::uint8_t*>(data);
    view.size = data->length;
    g_imu_callback(g_imu_user, view);
}

void info_trampoline(const std::uint32_t handle, const LivoxLidarInfo* info,
                     void* /*client_data*/) {
    if (g_info_callback == nullptr || info == nullptr) {
        return;
    }

    // SDK 的 sn/lidar_ip 是定长数组，不保证以 NUL 结尾；复制到本地缓冲。
    std::array<char, 17> serial{};
    std::array<char, 17> ip_address{};
    std::memcpy(serial.data(), info->sn, sizeof(info->sn));
    std::memcpy(ip_address.data(), info->lidar_ip, sizeof(info->lidar_ip));

    InfoView view;
    view.handle = handle;
    view.dev_type = info->dev_type;
    view.serial = serial.data();
    view.ip_address = ip_address.data();
    g_info_callback(g_info_user, view);
}

Error sdk_error(const char* operation, livox_status status) {
    return make_error(ErrorCode::kSdkError, std::string("Livox ") + operation + " 失败",
                      static_cast<std::int32_t>(status));
}

/// @brief 控制面等待雷达应答的上限；超过即判定超时并让调用方重试。
constexpr std::chrono::milliseconds kControlTimeout{800};

/// @brief 把 `std::string` 写入 SDK 的定长字符数组（保证 NUL 结尾、超长截断）。
void copy_fixed(char* dest, std::size_t capacity, const std::string& value) {
    std::memset(dest, 0, capacity);
    const std::size_t length = value.size() < capacity - 1 ? value.size() : capacity - 1;
    std::memcpy(dest, value.data(), length);
}

struct ControlAwaiter {
    std::mutex mutex;
    std::condition_variable cv;
    bool done = false;
    livox_status status = kLivoxLidarStatusFailure;
    std::uint8_t ret_code = 0;
};

void control_callback(livox_status status, std::uint32_t /*handle*/,
                      LivoxLidarAsyncControlResponse* response, void* client_data) {
    if (client_data == nullptr) {
        return;
    }
    auto& awaiter = *static_cast<ControlAwaiter*>(client_data);
    std::lock_guard<std::mutex> lock(awaiter.mutex);
    awaiter.done = true;
    awaiter.status = status;
    awaiter.ret_code = response != nullptr ? response->ret_code : 0;
    awaiter.cv.notify_one();
}

Result<void> wait_control(ControlAwaiter& awaiter, const char* operation) {
    std::unique_lock<std::mutex> lock(awaiter.mutex);
    if (!awaiter.cv.wait_for(lock, kControlTimeout, [&awaiter] { return awaiter.done; })) {
        return make_error(ErrorCode::kTimeout,
                          std::string("Livox ") + operation + " 等待雷达应答超时");
    }
    if (awaiter.status != kLivoxLidarStatusSuccess) {
        return sdk_error(operation, awaiter.status);
    }
    if (awaiter.ret_code != 0) {
        return make_error(ErrorCode::kSdkError,
                          std::string("Livox ") + operation + " 被雷达拒绝",
                          static_cast<std::int32_t>(awaiter.ret_code));
    }
    return Result<void>();
}

std::string read_c_string(const char* value, std::size_t capacity) {
    std::size_t length = 0;
    while (length < capacity && value[length] != '\0') {
        ++length;
    }
    return std::string(value, length);
}

/// @brief 解析 `QueryLivoxLidarInternalInfo` 的 TLV 负载，提取 IP 配置。
///
/// SDK 头文件只给出 `param_num`，没有总长度；先在 4 字节对齐的分条下扫描，
/// 找不到再按紧凑分条扫描，兼容两种序列化方式。
LidarInternalInfo parse_internal_info(const LivoxLidarDiagInternalInfoResponse& response) {
    LidarInternalInfo info;
    info.param_count = response.param_num;
    for (const std::size_t stride_align : {std::size_t{4}, std::size_t{0}}) {
        const std::uint8_t* cursor = response.data;
        for (std::uint16_t i = 0; i < response.param_num; ++i) {
            std::uint16_t key = 0;
            std::uint16_t length = 0;
            std::memcpy(&key, cursor, sizeof(key));
            std::memcpy(&length, cursor + sizeof(key), sizeof(length));
            const char* value = reinterpret_cast<const char*>(cursor + 2 * sizeof(std::uint16_t));
            if (key == kKeyLidarIpCfg && length > 0) {
                info.ip_address = read_c_string(value, std::min<std::size_t>(length, 16));
                return info;
            }
            const std::size_t base = 4U + length;
            cursor += stride_align == 0 ? base : ((base + stride_align - 1) & ~(stride_align - 1));
        }
    }
    return info;
}

struct QueryAwaiter {
    std::mutex mutex;
    std::condition_variable cv;
    bool done = false;
    bool ok = false;
    LidarInternalInfo info;
};

void query_callback(livox_status status, std::uint32_t /*handle*/,
                    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
    if (client_data == nullptr) {
        return;
    }
    auto& awaiter = *static_cast<QueryAwaiter*>(client_data);
    std::lock_guard<std::mutex> lock(awaiter.mutex);
    awaiter.done = true;
    awaiter.ok = status == kLivoxLidarStatusSuccess && response != nullptr &&
                 response->ret_code == 0;
    if (awaiter.ok) {
        awaiter.info = parse_internal_info(*response);
    }
    awaiter.cv.notify_one();
}

class RealLivoxApi final : public LivoxApi {
  public:
    Result<void> init(const std::string& json_path, const std::string& host_ip) override {
        const bool ok = LivoxLidarSdkInit(json_path.c_str(), host_ip.c_str(), nullptr);
        if (!ok) {
            return make_error(ErrorCode::kSdkError, "LivoxLidarSdkInit 失败: " + json_path);
        }
        return Result<void>();
    }

    void start() override {
        LivoxLidarSdkStart();
    }

    void uninit() override {
        LivoxLidarSdkUninit();
    }

    void set_point_cloud_callback(PacketCallback callback, void* user) override {
        g_point_callback = callback;
        g_point_user = user;
        SetLivoxLidarPointCloudCallBack(&point_cloud_trampoline, nullptr);
    }

    void set_imu_callback(PacketCallback callback, void* user) override {
        g_imu_callback = callback;
        g_imu_user = user;
        SetLivoxLidarImuDataCallback(&imu_trampoline, nullptr);
    }

    void set_info_change_callback(InfoCallback callback, void* user) override {
        g_info_callback = callback;
        g_info_user = user;
        SetLivoxLidarInfoChangeCallback(&info_trampoline, nullptr);
    }

    Result<void> set_work_mode(std::uint32_t handle, std::uint8_t mode) override {
        const livox_status status =
            SetLivoxLidarWorkMode(handle, static_cast<LivoxLidarWorkMode>(mode), nullptr, nullptr);
        return status == kLivoxLidarStatusSuccess ? Result<void>()
                                                  : sdk_error("SetLivoxLidarWorkMode", status);
    }

    Result<void> set_point_data_type(std::uint32_t handle, std::uint8_t type) override {
        const livox_status status = SetLivoxLidarPclDataType(
            handle, static_cast<LivoxLidarPointDataType>(type), nullptr, nullptr);
        return status == kLivoxLidarStatusSuccess ? Result<void>()
                                                  : sdk_error("SetLivoxLidarPclDataType", status);
    }

    Result<void> set_scan_pattern(std::uint32_t handle, std::uint8_t pattern) override {
        const livox_status status = SetLivoxLidarScanPattern(
            handle, static_cast<LivoxLidarScanPattern>(pattern), nullptr, nullptr);
        return status == kLivoxLidarStatusSuccess ? Result<void>()
                                                  : sdk_error("SetLivoxLidarScanPattern", status);
    }

    Result<void> set_dual_emit(std::uint32_t handle, bool enable) override {
        const livox_status status = SetLivoxLidarDualEmit(handle, enable, nullptr, nullptr);
        return status == kLivoxLidarStatusSuccess ? Result<void>()
                                                  : sdk_error("SetLivoxLidarDualEmit", status);
    }

    Result<void> request_reset(std::uint32_t handle) override {
        const livox_status status = LivoxLidarRequestReset(handle, nullptr, nullptr);
        return status == kLivoxLidarStatusSuccess ? Result<void>()
                                                  : sdk_error("LivoxLidarRequestReset", status);
    }

    Result<void> set_lidar_ip(std::uint32_t handle, const std::string& ip,
                              const std::string& net_mask, const std::string& gateway) override {
        LivoxLidarIpInfo config{};
        copy_fixed(config.ip_addr, sizeof(config.ip_addr), ip);
        copy_fixed(config.net_mask, sizeof(config.net_mask), net_mask);
        copy_fixed(config.gw_addr, sizeof(config.gw_addr), gateway);
        ControlAwaiter awaiter;
        const livox_status status = SetLivoxLidarIp(handle, &config, &control_callback, &awaiter);
        if (status != kLivoxLidarStatusSuccess) {
            return sdk_error("SetLivoxLidarIp", status);
        }
        return wait_control(awaiter, "SetLivoxLidarIp");
    }

    Result<void> set_install_attitude(std::uint32_t handle, float roll_deg, float pitch_deg,
                                      float yaw_deg, std::int32_t x_mm, std::int32_t y_mm,
                                      std::int32_t z_mm) override {
        LivoxLidarInstallAttitude attitude{};
        attitude.roll_deg = roll_deg;
        attitude.pitch_deg = pitch_deg;
        attitude.yaw_deg = yaw_deg;
        attitude.x = x_mm;
        attitude.y = y_mm;
        attitude.z = z_mm;
        ControlAwaiter awaiter;
        const livox_status status =
            SetLivoxLidarInstallAttitude(handle, &attitude, &control_callback, &awaiter);
        if (status != kLivoxLidarStatusSuccess) {
            return sdk_error("SetLivoxLidarInstallAttitude", status);
        }
        return wait_control(awaiter, "SetLivoxLidarInstallAttitude");
    }

    Result<void> set_pps_sync_mode(std::uint32_t handle, std::uint8_t mode) override {
        ControlAwaiter awaiter;
        const livox_status status = SetLivoxLidarPpsSyncMode(
            handle, static_cast<LivoxLidarPpsSyncMode>(mode), &control_callback, &awaiter);
        if (status != kLivoxLidarStatusSuccess) {
            return sdk_error("SetLivoxLidarPpsSyncMode", status);
        }
        return wait_control(awaiter, "SetLivoxLidarPpsSyncMode");
    }

    Result<void> set_host_port_cfg(std::uint32_t handle, const std::string& host_ip,
                                   std::uint16_t host_port, std::uint16_t lidar_port) override {
        HostPointIPInfo config{};
        copy_fixed(config.host_ip_addr, sizeof(config.host_ip_addr), host_ip);
        config.host_point_data_port = host_port;
        config.lidar_point_data_port = lidar_port;
        ControlAwaiter awaiter;
        const livox_status status =
            SetLivoxLidarPointDataHostIPCfg(handle, &config, &control_callback, &awaiter);
        if (status != kLivoxLidarStatusSuccess) {
            return sdk_error("SetLivoxLidarPointDataHostIPCfg", status);
        }
        return wait_control(awaiter, "SetLivoxLidarPointDataHostIPCfg");
    }

    Result<void> set_rmc_sync_time(std::uint32_t handle, const std::string& rmc) override {
        const livox_status status = SetLivoxLidarRmcSyncTime(
            handle, rmc.c_str(), static_cast<std::uint16_t>(rmc.size()), nullptr, nullptr);
        return status == kLivoxLidarStatusSuccess ? Result<void>()
                                                  : sdk_error("SetLivoxLidarRmcSyncTime", status);
    }

    Result<LidarInternalInfo> query_internal_info(std::uint32_t handle) override {
        QueryAwaiter awaiter;
        const livox_status status = QueryLivoxLidarInternalInfo(handle, &query_callback, &awaiter);
        if (status != kLivoxLidarStatusSuccess) {
            return sdk_error("QueryLivoxLidarInternalInfo", status);
        }
        std::unique_lock<std::mutex> lock(awaiter.mutex);
        if (!awaiter.cv.wait_for(lock, kControlTimeout, [&awaiter] { return awaiter.done; })) {
            return make_error(ErrorCode::kTimeout, "Livox QueryLivoxLidarInternalInfo 等待应答超时");
        }
        if (!awaiter.ok) {
            return make_error(ErrorCode::kSdkError, "Livox QueryLivoxLidarInternalInfo 返回错误");
        }
        return awaiter.info;
    }
};

} // namespace

std::shared_ptr<LivoxApi> make_livox_api() {
    static const std::shared_ptr<LivoxApi> api = std::make_shared<RealLivoxApi>();
    return api;
}

} // namespace foray::platform::driver::livox::detail
