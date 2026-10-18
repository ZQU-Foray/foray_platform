#include "livox/livox_api.hpp"

namespace foray::platform::driver::livox::detail {
namespace {

Error sdk_unavailable() {
    return make_error(ErrorCode::kSdkUnavailable,
                      "Livox SDK2 未在编译期启用（FORAY_BUILD_DRIVER_LIVOX=OFF）");
}

/// @brief 未启用 SDK 时的占位实现；保证下游可链接并显式失败。
class StubLivoxApi final : public LivoxApi {
  public:
    Result<void> init(const std::string&, const std::string&) override {
        return sdk_unavailable();
    }

    void start() override {
    }

    void uninit() override {
    }

    void set_point_cloud_callback(PacketCallback, void*) override {
    }

    void set_imu_callback(PacketCallback, void*) override {
    }

    void set_info_change_callback(InfoCallback, void*) override {
    }

    Result<void> set_work_mode(std::uint32_t, std::uint8_t) override {
        return sdk_unavailable();
    }

    Result<void> set_point_data_type(std::uint32_t, std::uint8_t) override {
        return sdk_unavailable();
    }

    Result<void> set_scan_pattern(std::uint32_t, std::uint8_t) override {
        return sdk_unavailable();
    }

    Result<void> set_dual_emit(std::uint32_t, bool) override {
        return sdk_unavailable();
    }

    Result<void> request_reset(std::uint32_t) override {
        return sdk_unavailable();
    }
};

} // namespace

std::shared_ptr<LivoxApi> make_livox_api() {
    static const std::shared_ptr<LivoxApi> api = std::make_shared<StubLivoxApi>();
    return api;
}

} // namespace foray::platform::driver::livox::detail
