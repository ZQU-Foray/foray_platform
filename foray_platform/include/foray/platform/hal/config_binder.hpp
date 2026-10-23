#pragma once

#include "foray/platform/hal/config.hpp"
#include "foray/platform/param/schema.hpp"
#include "foray/platform/param/snapshot.hpp"
#include "foray/platform/result/result.hpp"

namespace foray::platform::hal {

/**
 * @brief HAL 相机参数的 schema。
 *
 * 供装配根用 `ParamManager` 加载 YAML；字段名与 `CameraConfig` 一一对应。
 * 启动期字段（serial/transport/pixel_format 等）不热加载；曝光/增益/帧率可热加载。
 */
param::ParamSchema make_camera_param_schema();

/// @brief HAL 激光雷达参数的 schema。
param::ParamSchema make_lidar_param_schema();

/**
 * @brief 把已校验的参数快照映射为相机配置。
 *
 * 负责跨字段约束（transport 与 serial/ip、trigger 与 frame_rate、像素格式支持）；
 * 不读取文件、不探测设备能力，失败返回 `ErrorCode::kInvalidArgument`。
 */
Result<CameraConfig> bind_camera_config(const param::ParamSnapshot& snapshot);

/// @brief 把已校验的参数快照映射为激光雷达配置。
Result<LidarConfig> bind_lidar_config(const param::ParamSnapshot& snapshot);

} // namespace foray::platform::hal
