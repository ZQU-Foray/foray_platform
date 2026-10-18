#include "livox/livox_runtime.hpp"

#include <cstring>
#include <filesystem>
#include <utility>

#include "foray/platform/time/clock.hpp"
#include "livox/config_json.hpp"

namespace foray::platform::driver::livox::detail {

void CallbackSink::push(const PacketView& packet) noexcept {
    if (!active.load(std::memory_order_acquire)) {
        return;
    }
    if (queue == nullptr) {
        return;
    }
    if (packet.data == nullptr || packet.size < kPacketHeaderBytes ||
        packet.size > max_packet_bytes) {
        packets_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    PacketSlot slot;
    slot.handle = packet.handle;
    slot.dev_type = packet.dev_type;
    slot.length = static_cast<std::uint16_t>(packet.size);
    slot.receive_steady_ns =
        static_cast<std::uint64_t>(time::SteadyClock().now().nanoseconds_since_epoch());
    slot.receive_wall_ns =
        static_cast<std::uint64_t>(time::SystemClock().now().nanoseconds_since_epoch());
    std::memcpy(slot.data.data(), packet.data, packet.size);

    if (!queue->try_push(std::move(slot))) {
        packets_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    packets_received.fetch_add(1, std::memory_order_relaxed);
    bytes_received.fetch_add(packet.size, std::memory_order_relaxed);
}

LivoxRuntime& LivoxRuntime::instance() {
    static LivoxRuntime runtime;
    return runtime;
}

LivoxRuntime::LivoxRuntime() noexcept {
    clear_slots();
}

Result<void> LivoxRuntime::acquire(const hal::LidarConfig& config,
                                   const std::shared_ptr<LivoxApi>& api) {
    if (api == nullptr) {
        return make_error(ErrorCode::kInvalidArgument, "Livox SDK 函数表为空");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (ref_count_ == 0) {
        std::string path = config.config_json_path;
        if (path.empty()) {
            if (!is_valid_ipv4(config.host_ip)) {
                return make_error(ErrorCode::kInvalidArgument, "生成 Livox 配置需要合法的 host_ip");
            }
            if (!is_local_ipv4(config.host_ip)) {
                return make_error(ErrorCode::kInvalidArgument,
                                  "host_ip 不在本机网卡上: " + config.host_ip);
            }
            path = default_config_path(config.runtime_dir, config.host_ip);
            const Result<void> written = write_config_json(path, config.host_ip);
            if (!written.ok()) {
                return written;
            }
        } else if (!std::filesystem::exists(path)) {
            return make_error(ErrorCode::kInvalidArgument, "Livox 配置文件不存在: " + path);
        }

        const Result<void> initialized = api->init(path, config.host_ip);
        if (!initialized.ok()) {
            return initialized;
        }

        api->set_point_cloud_callback(&LivoxRuntime::packet_dispatch, this);
        api->set_imu_callback(&LivoxRuntime::packet_dispatch, this);
        api->set_info_change_callback(&LivoxRuntime::info_dispatch, this);
        api->start();

        api_ = api;
        json_path_ = std::move(path);
    } else if (api_.get() != api.get()) {
        return make_error(ErrorCode::kNotSupported,
                          "Livox SDK 已由另一个函数表初始化，进程内只允许一个实例");
    }

    ++ref_count_;
    return Result<void>();
}

void LivoxRuntime::release() {
    std::shared_ptr<LivoxApi> api;
    std::vector<std::shared_ptr<CallbackSink>> graveyard;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (ref_count_ == 0) {
            return;
        }
        --ref_count_;
        if (ref_count_ > 0) {
            return;
        }

        api = std::move(api_);
        api_.reset();
        json_path_.clear();
        clear_slots();
        graveyard = std::move(graveyard_);
        graveyard_.clear();
    }

    // SDK 反初始化会停止回调线程；墓地中的 sink 必须活到 Uninit 返回之后。
    if (api != nullptr) {
        api->uninit();
    }
}

Result<void> LivoxRuntime::attach(const std::shared_ptr<CallbackSink>& sink) {
    if (sink == nullptr || sink->queue == nullptr) {
        return make_error(ErrorCode::kInvalidArgument, "Livox 回调接收器或队列为空");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& slot : slots_) {
        CallbackSink* expected = nullptr;
        if (slot.compare_exchange_strong(expected, sink.get(), std::memory_order_acq_rel)) {
            return Result<void>();
        }
    }
    return make_error(ErrorCode::kBusy, "Livox handle 路由表已满");
}

void LivoxRuntime::detach(const std::shared_ptr<CallbackSink>& sink) {
    if (sink == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    sink->active.store(false, std::memory_order_release);

    for (auto& slot : slots_) {
        CallbackSink* current = sink.get();
        slot.compare_exchange_strong(current, nullptr, std::memory_order_acq_rel);
    }

    // 不能立即销毁：SDK 回调可能仍持有该指针。
    graveyard_.push_back(sink);
}

void LivoxRuntime::reset_for_testing() {
    std::shared_ptr<LivoxApi> api;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        api = std::move(api_);
        api_.reset();
        ref_count_ = 0;
        clear_slots();
        graveyard_.clear();
    }
    if (api != nullptr) {
        api->uninit();
    }
}

void LivoxRuntime::packet_dispatch(void* user, const PacketView& packet) {
    static_cast<LivoxRuntime*>(user)->on_packet(packet);
}

void LivoxRuntime::info_dispatch(void* user, const InfoView& info) {
    static_cast<LivoxRuntime*>(user)->on_info(info);
}

void LivoxRuntime::on_packet(const PacketView& packet) noexcept {
    CallbackSink* sink = find(packet.handle);
    if (sink == nullptr) {
        return;
    }
    sink->push(packet);
}

void LivoxRuntime::on_info(const InfoView& info) {
    CallbackSink* sink = find(info.handle);
    if (sink == nullptr) {
        // 只配置了 serial 的实例在这里按序列号绑定 handle。
        for (auto& slot : slots_) {
            CallbackSink* candidate = slot.load(std::memory_order_acquire);
            if (candidate == nullptr || candidate->handle.load(std::memory_order_relaxed) != 0 ||
                !candidate->active.load(std::memory_order_relaxed)) {
                continue;
            }

            const bool serial_matches = !candidate->serial.empty() && info.serial != nullptr &&
                                        candidate->serial == info.serial;
            const bool ip_matches = !candidate->ip_address.empty() && info.ip_address != nullptr &&
                                    candidate->ip_address == info.ip_address;
            if (!serial_matches && !ip_matches) {
                continue;
            }

            candidate->handle.store(info.handle, std::memory_order_release);
            sink = candidate;
            break;
        }
    }

    if (sink == nullptr) {
        return;
    }

    sink->dev_type.store(info.dev_type, std::memory_order_relaxed);
    if (sink->serial.empty() && info.serial != nullptr) {
        sink->serial = info.serial;
    }
    if (sink->ip_address.empty() && info.ip_address != nullptr) {
        sink->ip_address = info.ip_address;
    }
    sink->info_received.store(true, std::memory_order_release);
}

CallbackSink* LivoxRuntime::find(std::uint32_t handle) const noexcept {
    for (const auto& slot : slots_) {
        CallbackSink* sink = slot.load(std::memory_order_acquire);
        if (sink != nullptr && sink->handle.load(std::memory_order_relaxed) == handle) {
            return sink;
        }
    }
    return nullptr;
}

void LivoxRuntime::clear_slots() noexcept {
    for (auto& slot : slots_) {
        slot.store(nullptr, std::memory_order_release);
    }
}

} // namespace foray::platform::driver::livox::detail
