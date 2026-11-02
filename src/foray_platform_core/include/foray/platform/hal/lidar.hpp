#pragma once

#include "foray/platform/hal/device.hpp"
#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/frame_lease.hpp"
#include "foray/platform/time/duration.hpp"

namespace foray::platform::hal {

/**
 * @brief 激光雷达抽象。
 *
 * 点云与 IMU 分开读取；`read_imu()` 可被同一个消费者线程交错调用。
 * 每个雷达只允许**一个消费者线程**调用读取接口。
 */
class Lidar : public Device {
  public:
    /// @brief 读取一帧点云；超时返回 `ErrorCode::kTimeout`。
    virtual Result<void> read(FrameLease<PointCloudFrame>& out, time::Duration timeout) = 0;

    /// @brief 读取一个 IMU 采样；没有数据时返回 `ErrorCode::kNoData` 或 `kTimeout`。
    virtual Result<void> read_imu(ImuSample& out, time::Duration timeout) = 0;
};

} // namespace foray::platform::hal
