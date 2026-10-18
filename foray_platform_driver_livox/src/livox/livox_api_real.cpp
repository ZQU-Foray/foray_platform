#include "livox/livox_api.hpp"

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <array>
#include <cstring>
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
};

} // namespace

std::shared_ptr<LivoxApi> make_livox_api() {
    static const std::shared_ptr<LivoxApi> api = std::make_shared<RealLivoxApi>();
    return api;
}

} // namespace foray::platform::driver::livox::detail
