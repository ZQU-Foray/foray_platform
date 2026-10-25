#pragma once

#include "foray/platform/hal/sensor_info.hpp"
#include "foray/platform/hal/sensor_stats.hpp"
#include "foray/platform/hal/types.hpp"
#include "foray/platform/result/result.hpp"

namespace foray::platform::hal {

/**
 * @brief 设备生命周期接口。
 *
 * 调用约定：
 * - `open/start/stop/close/recover` 由**单一控制线程**调用；
 * - `close()` 幂等；即使中间步骤失败也必须尝试释放句柄；
 * - 默认不自动重连；由上层按退避策略调用 `recover()`；
 * - 回调线程不得调用这些方法。
 */
class Device {
  public:
    virtual ~Device() = default;

    virtual Result<void> open() = 0;
    virtual Result<void> start() = 0;
    virtual Result<void> stop() = 0;
    virtual Result<void> close() = 0;
    virtual Result<void> recover() = 0;

    virtual DeviceState state() const noexcept = 0;
    virtual SensorInfo info() const = 0;
    virtual SensorStats stats() const noexcept = 0;
};

} // namespace foray::platform::hal
