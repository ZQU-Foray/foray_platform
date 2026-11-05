#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "foray/platform/hal/types.hpp"
#include "foray/platform/result/result.hpp"

namespace foray::platform::driver::hikvision::detail {

/**
 * @brief 枚举得到的相机描述。
 *
 * 只含 HAL 已定义的字段，不含任何 MVS 类型；`mvs_api_real.cpp` 负责从
 * `MV_CC_DEVICE_INFO` 填充。
 */
struct DeviceDescriptor {
    std::string serial;
    std::string user_defined_name;
    std::string model;
    std::string firmware;
    hal::Transport transport = hal::Transport::kAny;
    std::uint16_t vendor_id = 0;
    std::uint16_t product_id = 0;
    std::string ip_address;
    /// 在该传输层枚举结果中的下标，仅用于诊断；匹配一律按 serial。
    std::uint32_t index = 0;
};

/**
 * @brief 一帧的元数据视图。
 *
 * 字段与 `MV_FRAME_OUT_INFO_EX` 对应，但不暴露 SDK 结构体。时间字段单位存疑的
 * （`host_timestamp_raw`）保留原值，由 `timestamp.cpp` 决定是否使用。
 */
struct FrameInfoView {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t frame_length = 0;
    hal::PixelFormat pixel_format = hal::PixelFormat::kMono8;
    std::uint32_t frame_number = 0;
    std::uint32_t trigger_index = 0;
    std::uint32_t lost_packets = 0;
    float exposure_us = 0.0F;
    float gain_db = 0.0F;
    /// `nDevTimeStampHigh << 32 | nDevTimeStampLow`，单位是设备 tick。
    std::uint64_t device_ticks = 0;
    /// `nHostTimeStamp`：MVS 未定义单位，不使用。
    std::uint32_t host_timestamp_raw = 0;
    /// `nSecondCount` / `nCycleCount` / `nCycleOffset`：PTP 时间戳。
    std::uint32_t second_count = 0;
    std::uint32_t cycle_count = 0;
    std::uint32_t cycle_offset = 0;
};

/**
 * @brief 一帧图像的所有权视图。
 *
 * `get_frame()` 返回，`free_frame()` 归还；两者必须在同一线程内配对，保证
 * `MV_CC_FreeImageBuffer` 与 `MV_CC_GetImageBuffer` 同线程。
 */
class FrameView {
  public:
    virtual ~FrameView() = default;

    virtual const std::uint8_t* data() const noexcept = 0;

    virtual const FrameInfoView& info() const noexcept = 0;
};

/**
 * @brief SDK 异常回调。
 *
 * 运行在 SDK 内部线程上：只允许置状态与计数，不得调用任何控制接口或阻塞。
 */
using ExceptionCallback = void (*)(void* user, std::uint32_t code);

/**
 * @brief MVS 的最小可用子集。
 *
 * 所有方法的错误码都已经是 HAL `Error`；`vendor_code` 保留 MVS 原始错误码的
 * 位模式，便于现场按 uint32 打印对照。
 */
class MvsApi {
  public:
    virtual ~MvsApi() = default;

    virtual Result<std::vector<DeviceDescriptor>> enumerate(hal::Transport transport) = 0;

    virtual Result<void*> create_handle(const DeviceDescriptor& device) = 0;

    virtual void destroy_handle(void* handle) noexcept = 0;

    virtual Result<void> open_device(void* handle) = 0;

    virtual Result<void> close_device(void* handle) = 0;

    virtual void register_exception(void* handle, ExceptionCallback callback,
                                    void* user) = 0;

    virtual Result<void> set_int(void* handle, std::string_view node,
                                std::int64_t value) = 0;

    virtual Result<std::int64_t> get_int(void* handle, std::string_view node) = 0;

    virtual Result<void> set_float(void* handle, std::string_view node,
                                  double value) = 0;

    virtual Result<double> get_float(void* handle, std::string_view node) = 0;

    virtual Result<void> set_bool(void* handle, std::string_view node, bool value) = 0;

    virtual Result<bool> get_bool(void* handle, std::string_view node) = 0;

    virtual Result<void> set_enum_by_string(void* handle, std::string_view node,
                                            std::string_view value) = 0;

    virtual Result<std::string> get_enum(void* handle, std::string_view node) = 0;

    /// @brief `MV_CC_SetImageNodeNum`，必须在 `start_grabbing()` 之前调用。
    virtual Result<void> set_image_node_count(void* handle, std::uint32_t count) = 0;

    virtual Result<void> start_grabbing(void* handle) = 0;

    virtual Result<void> stop_grabbing(void* handle) = 0;

    virtual Result<std::unique_ptr<FrameView>> get_frame(void* handle,
                                                         std::uint32_t timeout_ms) = 0;

    virtual void free_frame(void* handle, std::unique_ptr<FrameView> frame) noexcept = 0;

    /// @brief `MV_CC_GetOptimalPacketSize`，仅 GigE 有效。
    virtual Result<std::uint32_t> optimal_packet_size(void* handle) = 0;

    /// @brief `MV_CC_TriggerSoftwareExecute`，对应 `hal::Camera::trigger()`。
    virtual Result<void> trigger_software(void* handle) = 0;

    /// @brief `MV_CC_GetSDKVersion()`，用于诊断日志。
    virtual std::uint32_t sdk_version() const noexcept = 0;
};

/**
 * @brief 取后端实现。
 *
 * 编译期启用 `FORAY_BUILD_DRIVER_HIKVISION=ON` 时返回真实实现；否则返回 stub，
 * 其所有操作都返回 `kSdkUnavailable`，使上层加载路径保持一致。
 */
std::shared_ptr<MvsApi> make_mvs_api();

} // namespace foray::platform::driver::hikvision::detail
