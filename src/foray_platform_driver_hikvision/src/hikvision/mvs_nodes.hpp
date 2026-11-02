#pragma once

#include <cstdint>
#include <string_view>

#include "foray/platform/hal/config.hpp"
#include "foray/platform/result/result.hpp"
#include "mvs_api.hpp"
#include "timestamp.hpp"

namespace foray::platform::driver::hikvision::detail {

/**
 * @brief 用到的 MVS 节点名。
 *
 * 全部是 GenICam 名字字符串，来源 `MvCameraControl.h` 的 `MV_CC_Set*Value` 系列
 * 接口说明与 `CameraParams.h` 的枚举定义；集中在此便于和文档 §6 的映射表对照。
 */
namespace node {

inline constexpr std::string_view kWidth = "Width";
inline constexpr std::string_view kHeight = "Height";
inline constexpr std::string_view kOffsetX = "OffsetX";
inline constexpr std::string_view kOffsetY = "OffsetY";
inline constexpr std::string_view kBinningHorizontal = "BinningHorizontal";
inline constexpr std::string_view kBinningVertical = "BinningVertical";
inline constexpr std::string_view kDecimationHorizontal = "DecimationHorizontal";
inline constexpr std::string_view kDecimationVertical = "DecimationVertical";
inline constexpr std::string_view kPixelFormat = "PixelFormat";
inline constexpr std::string_view kFrameRateEnable = "AcquisitionFrameRateEnable";
inline constexpr std::string_view kFrameRate = "AcquisitionFrameRate";
inline constexpr std::string_view kExposureAuto = "ExposureAuto";
inline constexpr std::string_view kExposureTime = "ExposureTime";
inline constexpr std::string_view kExposureLowerLimit = "AutoExposureTimeLowerLimit";
inline constexpr std::string_view kExposureUpperLimit = "AutoExposureTimeUpperLimit";
inline constexpr std::string_view kGainAuto = "GainAuto";
inline constexpr std::string_view kGain = "Gain";
inline constexpr std::string_view kGainLowerLimit = "AutoGainLowerLimit";
inline constexpr std::string_view kGainUpperLimit = "AutoGainUpperLimit";
inline constexpr std::string_view kTriggerMode = "TriggerMode";
inline constexpr std::string_view kTriggerSource = "TriggerSource";
inline constexpr std::string_view kTriggerActivation = "TriggerActivation";
inline constexpr std::string_view kLineDebouncerTime = "LineDebouncerTime";
inline constexpr std::string_view kPtpEnable = "GevIEEE1588";
inline constexpr std::string_view kPtpStatus = "GevIEEE1588Status";
inline constexpr std::string_view kPacketSize = "GevSCPSPacketSize";
inline constexpr std::string_view kPacketDelay = "GevSCPD";
inline constexpr std::string_view kHeartbeatTimeout = "GevHeartbeatTimeout";
inline constexpr std::string_view kDeviceTicksFrequency = "DeviceTicksFrequency";

} // namespace node

/**
 * @brief 应用一整套静态配置（`open()` 之后、`start_grabbing()` 之前）。
 *
 * 失败语义分两级：
 * - 几何、像素格式、曝光、增益、帧率、触发这些影响语义的写失败 → 直接返回错误，
 *   `open()` 整体失败（否则上层会拿到与配置不符的帧）；
 * - GigE 调优（包长、包间隔、心跳）与 PTP 使能属于尽力而为：失败只累加到
 *   `optional_errors`，不阻断 open。现场网卡差异大，这类写失败不该让相机不可用。
 *
 * @param optional_errors 输出参数，累加可选写失败的次数。
 */
Result<void> apply_config(MvsApi& api, void* handle, const hal::CameraConfig& config,
                          std::uint32_t& optional_errors);

/**
 * @brief 应用运行时调参。
 *
 * `CameraTuning` 的字段是 `optional<optional<T>>`：外层空 = 不动，内层空 = 清除
 * （例如把 `exposure_us` 清空表示交回自动曝光）。任一字段失败立即返回，调用方
 * 负责记录「部分生效」。
 */
Result<void> apply_tuning(MvsApi& api, void* handle, const hal::CameraTuning& tuning);

/// @brief `GevIEEE1588Status == "Slave"`；节点不存在或读取失败都返回 false。
bool query_ptp_slave(MvsApi& api, void* handle) noexcept;

/// @brief 设备 tick 频率；读不到返回 0（时间换算据此降级）。
std::uint32_t query_tick_frequency(MvsApi& api, void* handle) noexcept;

/**
 * @brief 由相机配置与设备实况合成时间戳配置。
 *
 * 纯函数，便于单测：`ptp_slave` 与 `tick_frequency_hz` 由调用方探测后传入。
 */
TimestampConfig make_timestamp_config(const hal::CameraConfig& config, bool ptp_slave,
                                      std::uint32_t tick_frequency_hz) noexcept;

} // namespace foray::platform::driver::hikvision::detail
