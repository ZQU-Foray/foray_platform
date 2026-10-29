#include <MvCameraControl.h>

#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "hikvision/mvs_api.hpp"
#include "hikvision/mvs_error.hpp"

namespace foray::platform::driver::hikvision::detail {
namespace {

/// @brief 定长字符字段可能不带 `'\0'`，按容量截断后再构造字符串。
std::string copy_fixed(const unsigned char* data, std::size_t capacity) {
    std::size_t length = 0;
    while (length < capacity && data[length] != '\0') {
        ++length;
    }
    return std::string(reinterpret_cast<const char*>(data), length);
}

/// @brief `char` 字段（如枚举符号名）的重载，避免调用点到处写 reinterpret_cast。
std::string copy_fixed(const char* data, std::size_t capacity) {
    return copy_fixed(reinterpret_cast<const unsigned char*>(data), capacity);
}

/// @brief `nCurrentIp` 是按字节顺序打包的 32 位地址（高位是第一个字节）。
std::string to_ipv4(unsigned int address) {
    return std::to_string((address >> 24) & 0xFFU) + "." + std::to_string((address >> 16) & 0xFFU) +
           "." + std::to_string((address >> 8) & 0xFFU) + "." + std::to_string(address & 0xFFU);
}

/**
 * @brief MVS 像素格式 → HAL 像素格式。
 *
 * 只映射 8 位 Mono/Bayer；其余一律返回 `kMono12`，由 `publish_frame()` 的
 * `is_supported_pixel_format()` 拒绝并计 `sdk_errors`——驱动不替调用方猜布局。
 */
hal::PixelFormat to_pixel_format(MvGvspPixelType type) {
    switch (type) {
    case PixelType_Gvsp_Mono8:
        return hal::PixelFormat::kMono8;
    case PixelType_Gvsp_BayerRG8:
        return hal::PixelFormat::kBayerRG8;
    case PixelType_Gvsp_BayerGR8:
        return hal::PixelFormat::kBayerGR8;
    case PixelType_Gvsp_BayerGB8:
        return hal::PixelFormat::kBayerGB8;
    case PixelType_Gvsp_BayerBG8:
        return hal::PixelFormat::kBayerBG8;
    default:
        return hal::PixelFormat::kMono12;
    }
}

/// @brief HAL 传输层 → `MV_CC_EnumDevicesEx2` 的 `nTLayerType`（CameraParams.h:146/148）。
unsigned int to_tlayer(hal::Transport transport) {
    switch (transport) {
    case hal::Transport::kGigE:
        return MV_GIGE_DEVICE;
    case hal::Transport::kUsb3:
        return MV_USB_DEVICE;
    case hal::Transport::kAny:
    default:
        return MV_GIGE_DEVICE | MV_USB_DEVICE;
    }
}

DeviceDescriptor describe(const MV_CC_DEVICE_INFO& info, std::uint32_t index) {
    DeviceDescriptor device;
    device.index = index;
    if (info.nTLayerType == MV_GIGE_DEVICE) {
        const MV_GIGE_DEVICE_INFO& gige = info.SpecialInfo.stGigEInfo;
        device.transport = hal::Transport::kGigE;
        device.model = copy_fixed(gige.chModelName, sizeof(gige.chModelName));
        device.serial = copy_fixed(gige.chSerialNumber, sizeof(gige.chSerialNumber));
        device.user_defined_name =
            copy_fixed(gige.chUserDefinedName, sizeof(gige.chUserDefinedName));
        device.firmware = copy_fixed(gige.chDeviceVersion, sizeof(gige.chDeviceVersion));
        device.ip_address = to_ipv4(gige.nCurrentIp);
        return device;
    }
    if (info.nTLayerType == MV_USB_DEVICE) {
        const MV_USB3_DEVICE_INFO& usb = info.SpecialInfo.stUsb3VInfo;
        device.transport = hal::Transport::kUsb3;
        device.vendor_id = usb.idVendor;
        device.product_id = usb.idProduct;
        device.model = copy_fixed(usb.chModelName, sizeof(usb.chModelName));
        device.serial = copy_fixed(usb.chSerialNumber, sizeof(usb.chSerialNumber));
        device.user_defined_name =
            copy_fixed(usb.chUserDefinedName, sizeof(usb.chUserDefinedName));
        device.firmware = copy_fixed(usb.chDeviceVersion, sizeof(usb.chDeviceVersion));
    }
    // 其余传输层（CameraLink / 采集卡）不在本驱动范围内，保留空字段。
    return device;
}

/**
 * @brief 一帧的实现，同时持有 `MV_FRAME_OUT` 以便原样归还。
 *
 * `MV_CC_FreeImageBuffer()` 要求传回与 `MV_CC_GetImageBuffer()` 相同的
 * `MV_FRAME_OUT`（含 `stFrameInfo` 里的校验信息），所以这里保存整个结构体，
 * 而不是只存 `pBufAddr`。
 */
class RealFrameView final : public FrameView {
  public:
    const std::uint8_t* data() const noexcept override {
        return out_.pBufAddr;
    }

    const FrameInfoView& info() const noexcept override {
        return info_;
    }

    MV_FRAME_OUT& out() noexcept {
        return out_;
    }

    /// @brief 在 `MV_CC_GetImageBuffer()` 成功后填充元数据视图。
    void refresh() {
        const MV_FRAME_OUT_INFO_EX& raw = out_.stFrameInfo;
        info_ = FrameInfoView{};
        // 超过 65535 的分辨率由扩展字段承载；常规相机只填 `nWidth`/`nHeight`。
        info_.width = raw.nWidth != 0 ? raw.nWidth : raw.nExtendWidth;
        info_.height = raw.nHeight != 0 ? raw.nHeight : raw.nExtendHeight;
        info_.frame_length =
            raw.nFrameLenEx != 0 ? static_cast<std::uint32_t>(raw.nFrameLenEx) : raw.nFrameLen;
        info_.pixel_format = to_pixel_format(raw.enPixelType);
        info_.frame_number = raw.nFrameNum;
        info_.trigger_index = raw.nTriggerIndex;
        info_.lost_packets = raw.nLostPacket;
        info_.exposure_us = raw.fExposureTime;
        info_.gain_db = raw.fGain;
        info_.device_ticks =
            (static_cast<std::uint64_t>(raw.nDevTimeStampHigh) << 32) | raw.nDevTimeStampLow;
        // `nHostTimeStamp` 单位未定义，只保留低 32 位供诊断，不参与时间换算。
        info_.host_timestamp_raw = static_cast<std::uint32_t>(raw.nHostTimeStamp);
        info_.second_count = raw.nSecondCount;
        info_.cycle_count = raw.nCycleCount;
        info_.cycle_offset = raw.nCycleOffset;
    }

  private:
    MV_FRAME_OUT out_{};
    FrameInfoView info_{};
};

/// @brief 把 HAL 回调桥接到 MVS 的 `__stdcall` 异常回调。
struct ExceptionBridge {
    ExceptionCallback callback = nullptr;
    void* user = nullptr;
};

void __stdcall exception_trampoline(unsigned int message_type, void* user) {
    const auto* bridge = static_cast<const ExceptionBridge*>(user);
    if (bridge == nullptr || bridge->callback == nullptr) {
        return;
    }
    bridge->callback(bridge->user, message_type);
}

/**
 * @brief MVS 真实后端。
 *
 * 设备描述里没有可直接序列化的句柄载体，因此 `enumerate()` 会把原始
 * `MV_CC_DEVICE_INFO` 按 serial 缓存；`create_handle()` 命中缓存即可，未命中时
 * 重新枚举一次（现场 DHCP 会让 IP 在两次调用之间变化）。
 */
class RealMvsApi final : public MvsApi {
  public:
    Result<std::vector<DeviceDescriptor>> enumerate(hal::Transport transport) override {
        MV_CC_DEVICE_INFO_LIST list{};
        const int ret =
            MV_CC_EnumDevicesEx2(to_tlayer(transport), &list, nullptr, SortMethod_SerialNumber);
        if (ret != MV_OK) {
            return Result<std::vector<DeviceDescriptor>>(
                mvs_failure("MV_CC_EnumDevicesEx2", static_cast<std::uint32_t>(ret)));
        }

        std::vector<DeviceDescriptor> devices;
        devices.reserve(list.nDeviceNum);
        std::lock_guard<std::mutex> lock(cache_mutex_);
        device_cache_.clear();
        for (unsigned int index = 0; index < list.nDeviceNum; ++index) {
            const MV_CC_DEVICE_INFO* raw = list.pDeviceInfo[index];
            if (raw == nullptr) {
                continue;
            }
            CachedDevice entry;
            entry.descriptor = describe(*raw, index);
            entry.raw = *raw;
            devices.push_back(entry.descriptor);
            device_cache_.push_back(std::move(entry));
        }
        return Result<std::vector<DeviceDescriptor>>(std::move(devices));
    }

    Result<void*> create_handle(const DeviceDescriptor& device) override {
        std::optional<MV_CC_DEVICE_INFO> raw = find_cached(device);
        if (!raw.has_value()) {
            // 缓存未命中：重新枚举一次，避免因 IP 变化而永远绑不上设备。
            const auto refreshed = enumerate(hal::Transport::kAny);
            if (!refreshed.ok()) {
                return Result<void*>(refreshed.error());
            }
            raw = find_cached(device);
        }
        if (!raw.has_value()) {
            return Result<void*>(make_error(ErrorCode::kDeviceNotFound,
                                            "枚举结果中找不到匹配的 MVS 设备描述"));
        }

        void* handle = nullptr;
        const int ret = MV_CC_CreateHandle(&handle, &raw.value());
        if (ret != MV_OK || handle == nullptr) {
            return Result<void*>(mvs_failure("MV_CC_CreateHandle", static_cast<std::uint32_t>(ret)));
        }
        return Result<void*>(handle);
    }

    void destroy_handle(void* handle) noexcept override {
        if (handle == nullptr) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(bridge_mutex_);
            bridges_.erase(handle);
        }
        (void)MV_CC_DestroyHandle(handle);
    }

    Result<void> open_device(void* handle) override {
        // `MV_ACCESS_Exclusive`：独占访问，避免与 MVS 客户端抢参数。
        const int ret = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
        if (ret != MV_OK) {
            return Result<void>(mvs_failure("MV_CC_OpenDevice", static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    Result<void> close_device(void* handle) override {
        const int ret = MV_CC_CloseDevice(handle);
        if (ret != MV_OK) {
            return Result<void>(mvs_failure("MV_CC_CloseDevice", static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    void register_exception(void* handle, ExceptionCallback callback, void* user) override {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        auto bridge = std::make_unique<ExceptionBridge>();
        bridge->callback = callback;
        bridge->user = user;
        ExceptionBridge* raw = bridge.get();
        bridges_[handle] = std::move(bridge);
        (void)MV_CC_RegisterExceptionCallBack(handle, &exception_trampoline, raw);
    }

    Result<void> set_int(void* handle, std::string_view node, std::int64_t value) override {
        const std::string key(node);
        const int ret = MV_CC_SetIntValueEx(handle, key.c_str(), value);
        if (ret != MV_OK) {
            return Result<void>(mvs_failure("MV_CC_SetIntValueEx " + key,
                                            static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    Result<std::int64_t> get_int(void* handle, std::string_view node) override {
        const std::string key(node);
        MVCC_INTVALUE_EX value{};
        const int ret = MV_CC_GetIntValueEx(handle, key.c_str(), &value);
        if (ret != MV_OK) {
            return Result<std::int64_t>(
                mvs_failure("MV_CC_GetIntValueEx " + key, static_cast<std::uint32_t>(ret)));
        }
        return Result<std::int64_t>(value.nCurValue);
    }

    Result<void> set_float(void* handle, std::string_view node, double value) override {
        const std::string key(node);
        const int ret = MV_CC_SetFloatValue(handle, key.c_str(), static_cast<float>(value));
        if (ret != MV_OK) {
            return Result<void>(mvs_failure("MV_CC_SetFloatValue " + key,
                                            static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    Result<double> get_float(void* handle, std::string_view node) override {
        const std::string key(node);
        MVCC_FLOATVALUE value{};
        const int ret = MV_CC_GetFloatValue(handle, key.c_str(), &value);
        if (ret != MV_OK) {
            return Result<double>(
                mvs_failure("MV_CC_GetFloatValue " + key, static_cast<std::uint32_t>(ret)));
        }
        return Result<double>(static_cast<double>(value.fCurValue));
    }

    Result<void> set_bool(void* handle, std::string_view node, bool value) override {
        const std::string key(node);
        const int ret = MV_CC_SetBoolValue(handle, key.c_str(), value);
        if (ret != MV_OK) {
            return Result<void>(mvs_failure("MV_CC_SetBoolValue " + key,
                                            static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    Result<bool> get_bool(void* handle, std::string_view node) override {
        const std::string key(node);
        bool value = false;
        const int ret = MV_CC_GetBoolValue(handle, key.c_str(), &value);
        if (ret != MV_OK) {
            return Result<bool>(
                mvs_failure("MV_CC_GetBoolValue " + key, static_cast<std::uint32_t>(ret)));
        }
        return Result<bool>(value);
    }

    Result<void> set_enum_by_string(void* handle, std::string_view node,
                                    std::string_view value) override {
        const std::string key(node);
        const std::string symbolic(value);
        const int ret = MV_CC_SetEnumValueByString(handle, key.c_str(), symbolic.c_str());
        if (ret != MV_OK) {
            return Result<void>(mvs_failure("MV_CC_SetEnumValueByString " + key + "=" + symbolic,
                                            static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    Result<std::string> get_enum(void* handle, std::string_view node) override {
        const std::string key(node);
        MVCC_ENUMVALUE value{};
        int ret = MV_CC_GetEnumValue(handle, key.c_str(), &value);
        if (ret != MV_OK) {
            return Result<std::string>(
                mvs_failure("MV_CC_GetEnumValue " + key, static_cast<std::uint32_t>(ret)));
        }
        MVCC_ENUMENTRY entry{};
        entry.nValue = value.nCurValue;
        ret = MV_CC_GetEnumEntrySymbolic(handle, key.c_str(), &entry);
        if (ret != MV_OK) {
            return Result<std::string>(
                mvs_failure("MV_CC_GetEnumEntrySymbolic " + key, static_cast<std::uint32_t>(ret)));
        }
        return Result<std::string>(copy_fixed(entry.chSymbolic, MV_MAX_SYMBOLIC_LEN));
    }

    Result<void> set_image_node_count(void* handle, std::uint32_t count) override {
        const int ret = MV_CC_SetImageNodeNum(handle, count);
        if (ret != MV_OK) {
            return Result<void>(
                mvs_failure("MV_CC_SetImageNodeNum", static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    Result<void> start_grabbing(void* handle) override {
        const int ret = MV_CC_StartGrabbing(handle);
        if (ret != MV_OK) {
            return Result<void>(mvs_failure("MV_CC_StartGrabbing", static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    Result<void> stop_grabbing(void* handle) override {
        const int ret = MV_CC_StopGrabbing(handle);
        if (ret != MV_OK) {
            return Result<void>(mvs_failure("MV_CC_StopGrabbing", static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    Result<std::unique_ptr<FrameView>> get_frame(void* handle, std::uint32_t timeout_ms) override {
        auto view = std::make_unique<RealFrameView>();
        const int ret = MV_CC_GetImageBuffer(handle, &view->out(), timeout_ms);
        if (ret != MV_OK) {
            return Result<std::unique_ptr<FrameView>>(
                mvs_failure("MV_CC_GetImageBuffer", static_cast<std::uint32_t>(ret)));
        }
        view->refresh();
        return Result<std::unique_ptr<FrameView>>(std::unique_ptr<FrameView>(std::move(view)));
    }

    void free_frame(void* handle, std::unique_ptr<FrameView> frame) noexcept override {
        if (frame == nullptr) {
            return;
        }
        auto* real = static_cast<RealFrameView*>(frame.get());
        // 拷贝一份：SDK 要求归还时传回的 `MV_FRAME_OUT` 与取流时一致。
        MV_FRAME_OUT out = real->out();
        (void)MV_CC_FreeImageBuffer(handle, &out);
    }

    Result<std::uint32_t> optimal_packet_size(void* handle) override {
        // 该接口直接返回包大小（不是 MV_OK 语义），负值才是错误。
        const int value = MV_CC_GetOptimalPacketSize(handle);
        if (value <= 0) {
            return Result<std::uint32_t>(mvs_failure("MV_CC_GetOptimalPacketSize",
                                                     static_cast<std::uint32_t>(value)));
        }
        return Result<std::uint32_t>(static_cast<std::uint32_t>(value));
    }

    Result<void> trigger_software(void* handle) override {
        const int ret = MV_CC_TriggerSoftwareExecute(handle);
        if (ret != MV_OK) {
            return Result<void>(
                mvs_failure("MV_CC_TriggerSoftwareExecute", static_cast<std::uint32_t>(ret)));
        }
        return Result<void>();
    }

    std::uint32_t sdk_version() const noexcept override {
        return MV_CC_GetSDKVersion();
    }

  private:
    struct CachedDevice {
        DeviceDescriptor descriptor;
        MV_CC_DEVICE_INFO raw{};
    };

    std::optional<MV_CC_DEVICE_INFO> find_cached(const DeviceDescriptor& device) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (!device.serial.empty()) {
            for (const auto& entry : device_cache_) {
                if (entry.descriptor.serial == device.serial) {
                    return entry.raw;
                }
            }
            return std::nullopt;
        }
        if (!device.model.empty()) {
            for (const auto& entry : device_cache_) {
                if (entry.descriptor.model == device.model &&
                    entry.descriptor.ip_address == device.ip_address) {
                    return entry.raw;
                }
            }
            return std::nullopt;
        }
        // 无任何识别字段时才退回下标，避免 serial 不匹配却绑错设备。
        if (device.index < device_cache_.size()) {
            return device_cache_[device.index].raw;
        }
        return std::nullopt;
    }

    std::mutex cache_mutex_;
    std::vector<CachedDevice> device_cache_;

    std::mutex bridge_mutex_;
    std::map<void*, std::unique_ptr<ExceptionBridge>> bridges_;
};

} // namespace

std::shared_ptr<MvsApi> make_mvs_api() {
    static const std::shared_ptr<MvsApi> api = std::make_shared<RealMvsApi>();
    return api;
}

} // namespace foray::platform::driver::hikvision::detail
