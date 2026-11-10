#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "foray/platform/container/spsc_queue.hpp"
#include "foray/platform/hal/config.hpp"
#include "foray/platform/result/result.hpp"
#include "livox/livox_api.hpp"

namespace foray::platform::driver::livox::detail {

/**
 * @brief 每个雷达一个回调接收器。
 *
 * 生命周期：`Lidar` 关闭时从运行时注销并移入运行时墓地，
 * 直到 SDK 反初始化后才销毁。这样 SDK 回调即使正在飞行也不会访问已释放内存。
 */
struct CallbackSink {
    std::atomic<std::uint32_t> handle{0};
    std::atomic<bool> active{true};
    std::atomic<bool> info_received{false};
    std::atomic<std::uint8_t> dev_type{0};

    std::atomic<std::uint64_t> packets_received{0};
    std::atomic<std::uint64_t> packets_dropped{0};
    std::atomic<std::uint64_t> bytes_received{0};

    /// @brief 原始包环形队列；由 sink 与 `Lidar` 共同持有。
    std::shared_ptr<container::SpscQueue<PacketSlot>> queue;

    /// @brief 选择条件；只在下线前由控制线程写，回调读取。
    std::string serial;
    std::string ip_address;

    /// @brief 单个包最大字节数；由创建者在注册前写入。
    std::uint16_t max_packet_bytes = static_cast<std::uint16_t>(kMaxPacketBytes);

    /// @brief SDK 回调入口：校验长度、打接收时间戳并拷贝到队列。
    void push(const PacketView& packet) noexcept;
};

/**
 * @brief Livox SDK2 的进程级运行时。
 *
 * SDK2 是进程级单例：`LivoxLidarSdkInit/Uninit` 不可并发重复调用。
 * 本类用引用计数管理初始化，并按 `handle` 把全局回调分发给各个 `Lidar`。
 */
class LivoxRuntime {
  public:
    static LivoxRuntime& instance();

    LivoxRuntime(const LivoxRuntime&) = delete;
    LivoxRuntime& operator=(const LivoxRuntime&) = delete;

    /// @brief 获取一个 SDK 引用；首个引用负责初始化并启动扫描。
    Result<void> acquire(const hal::LidarConfig& config, const std::shared_ptr<LivoxApi>& api);

    /// @brief 释放一个 SDK 引用；最后一个引用反初始化 SDK。
    void release();

    /// @brief 把回调接收器注册到 handle 路由表。
    Result<void> attach(const std::shared_ptr<CallbackSink>& sink);

    /// @brief 注销接收器；对象移入墓地，等 SDK 反初始化时统一销毁。
    void detach(const std::shared_ptr<CallbackSink>& sink);

    /// @brief 测试用：强制清空引用计数、路由表与墓地。
    void reset_for_testing();

  private:
    LivoxRuntime() noexcept;

    static void packet_dispatch(void* user, const PacketView& packet);
    static void info_dispatch(void* user, const InfoView& info);

    void on_packet(const PacketView& packet) noexcept;
    void on_info(const InfoView& info);

    CallbackSink* find(std::uint32_t handle) const noexcept;
    void clear_slots() noexcept;

    std::mutex mutex_;
    std::shared_ptr<LivoxApi> api_;
    std::string json_path_;
    int ref_count_ = 0;
    std::array<std::atomic<CallbackSink*>, kMaxLidars> slots_{};
    std::vector<std::shared_ptr<CallbackSink>> graveyard_;
};

} // namespace foray::platform::driver::livox::detail
