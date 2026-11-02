#include "mvs_api.hpp"

#include <utility>

#include "foray/platform/result/error.hpp"

namespace foray::platform::driver::hikvision::detail {

namespace {

Error sdk_unavailable_error() {
    return make_error(ErrorCode::kSdkUnavailable,
                      "Hikvision MVS 未在编译期启用（FORAY_BUILD_DRIVER_HIKVISION=OFF）");
}

template <typename T> Result<T> unavailable() {
    return Result<T>(sdk_unavailable_error());
}

/**
 * @brief 关闭 SDK 时的后端。
 *
 * 所有操作返回 `kSdkUnavailable`，让上层（配置加载、状态机）走与真实后端相同的
 * 分支，只是失败在第一步。
 */
class StubMvsApi final : public MvsApi {
  public:
    Result<std::vector<DeviceDescriptor>> enumerate(hal::Transport transport) override {
        static_cast<void>(transport);
        return unavailable<std::vector<DeviceDescriptor>>();
    }

    Result<void*> create_handle(const DeviceDescriptor& device) override {
        static_cast<void>(device);
        return unavailable<void*>();
    }

    void destroy_handle(void* handle) noexcept override {
        static_cast<void>(handle);
    }

    Result<void> open_device(void* handle) override {
        static_cast<void>(handle);
        return Result<void>(sdk_unavailable_error());
    }

    Result<void> close_device(void* handle) override {
        static_cast<void>(handle);
        return Result<void>(sdk_unavailable_error());
    }

    void register_exception(void* handle, ExceptionCallback callback, void* user) override {
        static_cast<void>(handle);
        static_cast<void>(callback);
        static_cast<void>(user);
    }

    Result<void> set_int(void* handle, std::string_view node_name,
                        std::int64_t value) override {
        static_cast<void>(handle);
        static_cast<void>(node_name);
        static_cast<void>(value);
        return Result<void>(sdk_unavailable_error());
    }

    Result<std::int64_t> get_int(void* handle, std::string_view node_name) override {
        static_cast<void>(handle);
        static_cast<void>(node_name);
        return unavailable<std::int64_t>();
    }

    Result<void> set_float(void* handle, std::string_view node_name, double value) override {
        static_cast<void>(handle);
        static_cast<void>(node_name);
        static_cast<void>(value);
        return Result<void>(sdk_unavailable_error());
    }

    Result<double> get_float(void* handle, std::string_view node_name) override {
        static_cast<void>(handle);
        static_cast<void>(node_name);
        return unavailable<double>();
    }

    Result<void> set_bool(void* handle, std::string_view node_name, bool value) override {
        static_cast<void>(handle);
        static_cast<void>(node_name);
        static_cast<void>(value);
        return Result<void>(sdk_unavailable_error());
    }

    Result<bool> get_bool(void* handle, std::string_view node_name) override {
        static_cast<void>(handle);
        static_cast<void>(node_name);
        return unavailable<bool>();
    }

    Result<void> set_enum_by_string(void* handle, std::string_view node_name,
                                    std::string_view value) override {
        static_cast<void>(handle);
        static_cast<void>(node_name);
        static_cast<void>(value);
        return Result<void>(sdk_unavailable_error());
    }

    Result<std::string> get_enum(void* handle, std::string_view node_name) override {
        static_cast<void>(handle);
        static_cast<void>(node_name);
        return unavailable<std::string>();
    }

    Result<void> set_image_node_count(void* handle, std::uint32_t count) override {
        static_cast<void>(handle);
        static_cast<void>(count);
        return Result<void>(sdk_unavailable_error());
    }

    Result<void> start_grabbing(void* handle) override {
        static_cast<void>(handle);
        return Result<void>(sdk_unavailable_error());
    }

    Result<void> stop_grabbing(void* handle) override {
        static_cast<void>(handle);
        return Result<void>(sdk_unavailable_error());
    }

    Result<std::unique_ptr<FrameView>> get_frame(void* handle,
                                                 std::uint32_t timeout_ms) override {
        static_cast<void>(handle);
        static_cast<void>(timeout_ms);
        return unavailable<std::unique_ptr<FrameView>>();
    }

    void free_frame(void* handle, std::unique_ptr<FrameView> frame) noexcept override {
        static_cast<void>(handle);
        static_cast<void>(frame);
    }

    Result<std::uint32_t> optimal_packet_size(void* handle) override {
        static_cast<void>(handle);
        return unavailable<std::uint32_t>();
    }

    Result<void> trigger_software(void* handle) override {
        static_cast<void>(handle);
        return Result<void>(sdk_unavailable_error());
    }

    std::uint32_t sdk_version() const noexcept override {
        return 0;
    }
};

} // namespace

std::shared_ptr<MvsApi> make_mvs_api() {
    static const std::shared_ptr<MvsApi> api = std::make_shared<StubMvsApi>();
    return api;
}

} // namespace foray::platform::driver::hikvision::detail
