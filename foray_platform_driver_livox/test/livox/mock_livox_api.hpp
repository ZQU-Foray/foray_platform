#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "livox/livox_api.hpp"

namespace foray::platform::driver::livox::test {

/// @brief 可注入回调的 Livox SDK 函数表，不需要真实 SDK。
class MockLivoxApi final : public detail::LivoxApi {
  public:
    Result<void> init(const std::string& json_path, const std::string& host_ip) override {
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            accepting.store(true, std::memory_order_release);
        }
        ++init_calls;
        last_json_path = json_path;
        last_host_ip = host_ip;
        initialized = true;
        return Result<void>();
    }

    void start() override {
        ++start_calls;
        started = true;
    }

    void uninit() override {
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            accepting.store(false, std::memory_order_release);
        }
        state_cv.notify_all();

        // 模拟真实 SDK：Uninit 返回后不再有回调在飞行。
        std::unique_lock<std::mutex> lock(state_mutex);
        state_cv.wait(lock, [this] { return in_flight.load(std::memory_order_acquire) == 0; });

        ++uninit_calls;
        initialized = false;
        started = false;
    }

    void set_point_cloud_callback(detail::PacketCallback callback, void* user) override {
        point_callback.store(callback, std::memory_order_release);
        point_user.store(user, std::memory_order_release);
    }

    void set_imu_callback(detail::PacketCallback callback, void* user) override {
        imu_callback.store(callback, std::memory_order_release);
        imu_user.store(user, std::memory_order_release);
    }

    void set_info_change_callback(detail::InfoCallback callback, void* user) override {
        info_callback.store(callback, std::memory_order_release);
        info_user.store(user, std::memory_order_release);
    }

    Result<void> set_work_mode(std::uint32_t handle, std::uint8_t mode) override {
        last_handle = handle;
        work_mode = mode;
        ++control_calls;
        return Result<void>();
    }

    Result<void> set_point_data_type(std::uint32_t handle, std::uint8_t type) override {
        last_handle = handle;
        point_data_type = type;
        ++control_calls;
        return Result<void>();
    }

    Result<void> set_scan_pattern(std::uint32_t handle, std::uint8_t pattern) override {
        last_handle = handle;
        scan_pattern = pattern;
        ++control_calls;
        return Result<void>();
    }

    Result<void> set_dual_emit(std::uint32_t handle, bool enable) override {
        last_handle = handle;
        dual_emit = enable;
        ++control_calls;
        return Result<void>();
    }

    Result<void> request_reset(std::uint32_t handle) override {
        last_handle = handle;
        ++reset_calls;
        return Result<void>();
    }

    void emit_info(std::uint32_t handle, std::uint8_t dev_type, const std::string& serial,
                   const std::string& ip_address) {
        if (!begin_callback()) {
            return;
        }
        const detail::InfoCallback callback = info_callback.load(std::memory_order_acquire);
        if (callback != nullptr) {
            const detail::InfoView view{handle, dev_type, serial.c_str(), ip_address.c_str()};
            callback(info_user.load(std::memory_order_acquire), view);
        }
        end_callback();
    }

    void emit_point_packet(std::uint32_t handle, std::uint8_t dev_type,
                           const std::vector<std::uint8_t>& bytes) {
        if (!begin_callback()) {
            return;
        }
        const detail::PacketCallback callback = point_callback.load(std::memory_order_acquire);
        if (callback != nullptr) {
            const detail::PacketView view{handle, dev_type, bytes.data(),
                                          static_cast<std::uint32_t>(bytes.size())};
            callback(point_user.load(std::memory_order_acquire), view);
        }
        end_callback();
    }

    void emit_imu_packet(std::uint32_t handle, std::uint8_t dev_type,
                         const std::vector<std::uint8_t>& bytes) {
        if (!begin_callback()) {
            return;
        }
        const detail::PacketCallback callback = imu_callback.load(std::memory_order_acquire);
        if (callback != nullptr) {
            const detail::PacketView view{handle, dev_type, bytes.data(),
                                          static_cast<std::uint32_t>(bytes.size())};
            callback(imu_user.load(std::memory_order_acquire), view);
        }
        end_callback();
    }

    int init_calls = 0;
    int start_calls = 0;
    int uninit_calls = 0;
    int control_calls = 0;
    int reset_calls = 0;
    bool initialized = false;
    bool started = false;

    std::string last_json_path;
    std::string last_host_ip;
    std::uint32_t last_handle = 0;
    std::uint8_t work_mode = 0;
    std::uint8_t point_data_type = 0;
    std::uint8_t scan_pattern = 0;
    bool dual_emit = false;

    std::atomic<detail::PacketCallback> point_callback{nullptr};
    std::atomic<void*> point_user{nullptr};
    std::atomic<detail::PacketCallback> imu_callback{nullptr};
    std::atomic<void*> imu_user{nullptr};
    std::atomic<detail::InfoCallback> info_callback{nullptr};
    std::atomic<void*> info_user{nullptr};

  private:
    bool begin_callback() {
        if (!accepting.load(std::memory_order_acquire)) {
            return false;
        }
        in_flight.fetch_add(1, std::memory_order_acq_rel);
        if (!accepting.load(std::memory_order_acquire)) {
            end_callback();
            return false;
        }
        return true;
    }

    void end_callback() {
        if (in_flight.fetch_sub(1, std::memory_order_acq_rel) == 1) {
            std::lock_guard<std::mutex> lock(state_mutex);
            state_cv.notify_all();
        }
    }

    std::atomic<bool> accepting{false};
    std::atomic<int> in_flight{0};
    std::mutex state_mutex;
    std::condition_variable state_cv;
};

} // namespace foray::platform::driver::livox::test
