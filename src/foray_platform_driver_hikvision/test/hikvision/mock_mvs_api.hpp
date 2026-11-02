#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "hikvision/mvs_api.hpp"
#include "hikvision/mvs_error.hpp"

namespace foray::platform::driver::hikvision::test {

/// @brief 一次节点写入的记录。
struct NodeWrite {
    std::string node;
    std::string kind;
    std::string value;
};

/// @brief 测试用帧视图：自持载荷，交给 `get_frame()` 返回。
class TestFrame final : public detail::FrameView {
  public:
    TestFrame(detail::FrameInfoView info, std::vector<std::uint8_t> payload)
        : info_(info), payload_(std::move(payload)) {
    }

    const std::uint8_t* data() const noexcept override {
        return payload_.data();
    }

    const detail::FrameInfoView& info() const noexcept override {
        return info_;
    }

  private:
    detail::FrameInfoView info_;
    std::vector<std::uint8_t> payload_;
};

/**
 * @brief 可编程的 `MvsApi` 假实现。
 *
 * 设计目标：
 * - 记录**调用顺序**（`events`），用于断言 `SetImageNodeNum` 早于 `StartGrabbing`；
 * - 记录**每次节点写入**（`writes`），用于断言配置映射与运行期调参；
 * - 可按节点名注入失败，验证必需项失败 vs GigE 调优失败的不同处理；
 * - 支持排队帧，验证取流线程的发布/丢帧/超时路径。
 *
 * 线程安全：取流线程会并发调 `get_frame()` / `free_frame()`，其余由测试线程调用，
 * 所有可变状态都由 `mutex_` 保护。
 */
class MockMvsApi final : public detail::MvsApi {
  public:
    // —— 可编程输入：枚举与生命周期 ——
    std::vector<detail::DeviceDescriptor> devices;
    /// 非 0 时 `enumerate()` 返回该 MVS 原始错误码。
    std::uint32_t enumerate_error = 0;
    /// 非 0 时 `create_handle()` 返回该错误码。
    std::uint32_t create_error = 0;
    /// 非 0 时 `open_device()` 返回该错误码。
    std::uint32_t open_error = 0;
    /// 非 0 时 `start_grabbing()` 返回该错误码。
    std::uint32_t start_grabbing_error = 0;
    /// 非 0 时 `stop_grabbing()` 返回该错误码。
    std::uint32_t stop_grabbing_error = 0;
    /// 非 0 时 `trigger_software()` 返回该错误码。
    std::uint32_t trigger_error = 0;
    /// 非 0 时 `close_device()` 返回该错误码（用于验证 destroy 仍被调用）。
    std::uint32_t close_error = 0;

    // —— 可编程输入：节点 ——
    std::map<std::string, std::int64_t, std::less<>> int_values;
    std::map<std::string, double, std::less<>> float_values;
    std::map<std::string, bool, std::less<>> bool_values;
    std::map<std::string, std::string, std::less<>> enum_values;
    /// 节点名 → MVS 原始错误码；命中则该节点写/读失败。
    std::map<std::string, std::uint32_t, std::less<>> failing_nodes;

    // —— 可编程输入：杂项 ——
    std::uint32_t sdk_version_value = 0x04080003U;
    std::uint32_t optimal_packet_size_value = 1500;
    /// 非 0 时 `optimal_packet_size()` 返回该错误码。
    std::uint32_t optimal_packet_error = 0;
    /// 非 0 时 `get_frame()` 返回该错误码（默认返回 `kTimeout`）。
    std::uint32_t frame_error = 0;

    // —— 记录 ——
    std::vector<std::string> events;
    std::vector<NodeWrite> writes;

    std::atomic<int> enumerate_calls{0};
    std::atomic<int> create_calls{0};
    std::atomic<int> destroy_calls{0};
    std::atomic<int> open_calls{0};
    std::atomic<int> close_calls{0};
    std::atomic<int> start_grabbing_calls{0};
    std::atomic<int> stop_grabbing_calls{0};
    std::atomic<int> trigger_calls{0};
    std::atomic<int> get_frame_calls{0};
    std::atomic<int> free_frame_calls{0};
    std::atomic<int> register_exception_calls{0};

    detail::ExceptionCallback exception_callback = nullptr;
    void* exception_user = nullptr;

    // —— 帧注入 ——
    void push_frame(std::unique_ptr<detail::FrameView> frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.push_back(std::move(frame));
    }

    void clear_frames() {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.clear();
    }

    std::size_t pending_frames() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return frames_.size();
    }

    // —— 查询辅助 ——
    std::vector<std::string> snapshot_events() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return events;
    }

    std::vector<NodeWrite> snapshot_writes() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return writes;
    }

    /// @brief 某节点最后一次写入的值；没写过返回空。
    std::optional<std::string> last_write(std::string_view node_name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::optional<std::string> result;
        for (const auto& write : writes) {
            if (write.node == node_name) {
                result = write.value;
            }
        }
        return result;
    }

    bool has_event(std::string_view name) const {
        const auto snapshot = snapshot_events();
        for (const auto& event : snapshot) {
            if (event == name) {
                return true;
            }
        }
        return false;
    }

    /// @brief 某个事件首次出现的下标；不存在返回 `npos`。
    std::size_t index_of_event(std::string_view name) const {
        const auto snapshot = snapshot_events();
        for (std::size_t index = 0; index < snapshot.size(); ++index) {
            if (snapshot[index] == name) {
                return index;
            }
        }
        return static_cast<std::size_t>(-1);
    }

    void fire_exception(std::uint32_t code) {
        detail::ExceptionCallback callback = nullptr;
        void* user = nullptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            callback = exception_callback;
            user = exception_user;
        }
        if (callback != nullptr) {
            callback(user, code);
        }
    }

    // —— MvsApi 实现 ——
    Result<std::vector<detail::DeviceDescriptor>> enumerate(hal::Transport transport) override {
        static_cast<void>(transport);
        enumerate_calls.fetch_add(1);
        record("enumerate");
        if (enumerate_error != 0) {
            return detail::mvs_failure("MV_CC_EnumDevicesEx2", enumerate_error);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        return Result<std::vector<detail::DeviceDescriptor>>(devices);
    }

    Result<void*> create_handle(const detail::DeviceDescriptor& device) override {
        static_cast<void>(device);
        create_calls.fetch_add(1);
        record("create_handle");
        if (create_error != 0) {
            return Result<void*>(detail::mvs_failure("MV_CC_CreateHandle", create_error));
        }
        return Result<void*>(static_cast<void*>(&handle_storage_));
    }

    void destroy_handle(void* handle) noexcept override {
        static_cast<void>(handle);
        destroy_calls.fetch_add(1);
        record("destroy_handle");
    }

    Result<void> open_device(void* handle) override {
        static_cast<void>(handle);
        open_calls.fetch_add(1);
        record("open_device");
        if (open_error != 0) {
            return Result<void>(detail::mvs_failure("MV_CC_OpenDevice", open_error));
        }
        return Result<void>();
    }

    Result<void> close_device(void* handle) override {
        static_cast<void>(handle);
        close_calls.fetch_add(1);
        record("close_device");
        if (close_error != 0) {
            return Result<void>(detail::mvs_failure("MV_CC_CloseDevice", close_error));
        }
        return Result<void>();
    }

    void register_exception(void* handle, detail::ExceptionCallback callback,
                            void* user) override {
        static_cast<void>(handle);
        register_exception_calls.fetch_add(1);
        record("register_exception");
        std::lock_guard<std::mutex> lock(mutex_);
        exception_callback = callback;
        exception_user = user;
    }

    Result<void> set_int(void* handle, std::string_view node_name,
                        std::int64_t value) override {
        static_cast<void>(handle);
        return record_write(node_name, "int", std::to_string(value));
    }

    Result<std::int64_t> get_int(void* handle, std::string_view node_name) override {
        static_cast<void>(handle);
        if (const auto failure = check_failing(node_name); failure.has_value()) {
            return Result<std::int64_t>(*failure);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = int_values.find(node_name);
        if (it == int_values.end()) {
            return Result<std::int64_t>(make_error(
                ErrorCode::kSdkError, "mock: 未配置 int 节点 " + std::string(node_name)));
        }
        return Result<std::int64_t>(it->second);
    }

    Result<void> set_float(void* handle, std::string_view node_name, double value) override {
        static_cast<void>(handle);
        return record_write(node_name, "float", std::to_string(value));
    }

    Result<double> get_float(void* handle, std::string_view node_name) override {
        static_cast<void>(handle);
        if (const auto failure = check_failing(node_name); failure.has_value()) {
            return Result<double>(*failure);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = float_values.find(node_name);
        if (it == float_values.end()) {
            return Result<double>(make_error(
                ErrorCode::kSdkError, "mock: 未配置 float 节点 " + std::string(node_name)));
        }
        return Result<double>(it->second);
    }

    Result<void> set_bool(void* handle, std::string_view node_name, bool value) override {
        static_cast<void>(handle);
        return record_write(node_name, "bool", value ? "true" : "false");
    }

    Result<bool> get_bool(void* handle, std::string_view node_name) override {
        static_cast<void>(handle);
        if (const auto failure = check_failing(node_name); failure.has_value()) {
            return Result<bool>(*failure);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = bool_values.find(node_name);
        if (it == bool_values.end()) {
            return Result<bool>(make_error(
                ErrorCode::kSdkError, "mock: 未配置 bool 节点 " + std::string(node_name)));
        }
        return Result<bool>(it->second);
    }

    Result<void> set_enum_by_string(void* handle, std::string_view node_name,
                                    std::string_view value) override {
        static_cast<void>(handle);
        return record_write(node_name, "enum", std::string(value));
    }

    Result<std::string> get_enum(void* handle, std::string_view node_name) override {
        static_cast<void>(handle);
        if (const auto failure = check_failing(node_name); failure.has_value()) {
            return Result<std::string>(*failure);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = enum_values.find(node_name);
        if (it == enum_values.end()) {
            return Result<std::string>(make_error(
                ErrorCode::kSdkError, "mock: 未配置 enum 节点 " + std::string(node_name)));
        }
        return Result<std::string>(it->second);
    }

    Result<void> set_image_node_count(void* handle, std::uint32_t count) override {
        static_cast<void>(handle);
        return record_write("MV_CC_SetImageNodeNum", "int", std::to_string(count));
    }

    Result<void> start_grabbing(void* handle) override {
        static_cast<void>(handle);
        start_grabbing_calls.fetch_add(1);
        record("start_grabbing");
        if (start_grabbing_error != 0) {
            return Result<void>(detail::mvs_failure("MV_CC_StartGrabbing", start_grabbing_error));
        }
        return Result<void>();
    }

    Result<void> stop_grabbing(void* handle) override {
        static_cast<void>(handle);
        stop_grabbing_calls.fetch_add(1);
        record("stop_grabbing");
        if (stop_grabbing_error != 0) {
            return Result<void>(detail::mvs_failure("MV_CC_StopGrabbing", stop_grabbing_error));
        }
        return Result<void>();
    }

    Result<std::unique_ptr<detail::FrameView>> get_frame(void* handle,
                                                         std::uint32_t timeout_ms) override {
        static_cast<void>(handle);
        get_frame_calls.fetch_add(1);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!frames_.empty()) {
                auto frame = std::move(frames_.front());
                frames_.pop_front();
                return Result<std::unique_ptr<detail::FrameView>>(std::move(frame));
            }
        }
        if (frame_error != 0) {
            return Result<std::unique_ptr<detail::FrameView>>(
                detail::mvs_failure("MV_CC_GetImageBuffer", frame_error));
        }
        // 模拟真实 SDK 的阻塞等待，避免测试里的取流线程空转。
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        static_cast<void>(timeout_ms);
        return Result<std::unique_ptr<detail::FrameView>>(
            make_error(ErrorCode::kTimeout, "mock: 无可用帧"));
    }

    void free_frame(void* handle, std::unique_ptr<detail::FrameView> frame) noexcept override {
        static_cast<void>(handle);
        static_cast<void>(frame);
        free_frame_calls.fetch_add(1);
    }

    Result<std::uint32_t> optimal_packet_size(void* handle) override {
        static_cast<void>(handle);
        if (optimal_packet_error != 0) {
            return Result<std::uint32_t>(
                detail::mvs_failure("MV_CC_GetOptimalPacketSize", optimal_packet_error));
        }
        return Result<std::uint32_t>(optimal_packet_size_value);
    }

    Result<void> trigger_software(void* handle) override {
        static_cast<void>(handle);
        trigger_calls.fetch_add(1);
        record("trigger_software");
        if (trigger_error != 0) {
            return Result<void>(detail::mvs_failure("MV_CC_TriggerSoftwareExecute", trigger_error));
        }
        return Result<void>();
    }

    std::uint32_t sdk_version() const noexcept override {
        return sdk_version_value;
    }

  private:
    void record(std::string_view name) {
        std::lock_guard<std::mutex> lock(mutex_);
        events.emplace_back(name);
    }

    Result<void> record_write(std::string_view node_name, std::string_view kind,
                              std::string value) {
        if (const auto failure = check_failing(node_name); failure.has_value()) {
            return Result<void>(*failure);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        events.emplace_back("set:" + std::string(node_name));
        writes.push_back(
            NodeWrite{std::string(node_name), std::string(kind), std::move(value)});
        return Result<void>();
    }

    std::optional<Error> check_failing(std::string_view node_name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = failing_nodes.find(node_name);
        if (it == failing_nodes.end()) {
            return std::nullopt;
        }
        return detail::mvs_failure("节点 " + std::string(node_name), it->second);
    }

    mutable std::mutex mutex_;
    std::deque<std::unique_ptr<detail::FrameView>> frames_;
    /// @brief 给 `create_handle()` 一个稳定但不可解引用的句柄值。
    int handle_storage_ = 0;
};

} // namespace foray::platform::driver::hikvision::test
