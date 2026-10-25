#include <algorithm>
#include <chrono>
#include <new>
#include <system_error>
#include <utility>

#include "foray/platform/time/clock.hpp"
#include "livox/lidar_factory.hpp"
#include "livox/lidar_impl.hpp"

namespace foray::platform::driver::livox::detail {
namespace {

// SDK 扫描到设备并触发 InfoChange 的最长等待时间。
constexpr std::int64_t kInfoWaitMs = 5000;
// 设备命令重试次数与间隔；设备刚上线时命令通道可能尚未就绪。
constexpr int kConfigAttempts = 5;
constexpr std::int64_t kConfigRetryMs = 100;
// 解析线程空闲轮询间隔；10 Hz 点云下额外延迟可忽略。
constexpr std::int64_t kPollMicroseconds = 200;
// IMU 队列容量：200 Hz 下约 5 秒缓冲。
constexpr std::size_t kImuQueueSlots = 1024;
// 标准重力加速度（ISO 80000-3）；Livox IMU 加速度单位为 g，HAL 统一为 m/s^2。
constexpr float kStandardGravity = 9.80665F;

// SDK 工作模式（LivoxLidarWorkMode）。
constexpr std::uint8_t kWorkModeNormal = 0x01;
// 点数据类型（LivoxLidarPointDataType）。
constexpr std::uint8_t kDataTypeImu = 0x00;
constexpr std::uint8_t kDataTypeHigh = 0x01;
constexpr std::uint8_t kDataTypeLow = 0x02;
constexpr std::uint8_t kDataTypeSpherical = 0x03;
// 双回波在 SDK v1.3.1 公共头中未定义结构，按协议值下发，解析侧显式拒绝。
constexpr std::uint8_t kDataTypeDualEcho = 0x11;
// 扫描模式（LivoxLidarScanPattern）。
constexpr std::uint8_t kScanPatternNonRepetitive = 0x00;
constexpr std::uint8_t kScanPatternRepetitive = 0x01;

// 恢复退避：100/200/400/800/1600 ms。
constexpr std::int64_t kRecoverBackoffMs[] = {100, 200, 400, 800, 1600};

void sleep_milliseconds(std::int64_t milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void sleep_microseconds(std::int64_t microseconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

} // namespace

LidarImpl::LidarImpl(const hal::LidarConfig& config, std::shared_ptr<LivoxApi> api)
    : config_(config), api_(std::move(api)) {
}

LidarImpl::~LidarImpl() {
    close();
}

Result<void> LidarImpl::open() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return open_locked();
}

Result<void> LidarImpl::start() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return start_locked();
}

Result<void> LidarImpl::stop() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return stop_locked();
}

Result<void> LidarImpl::close() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return close_locked();
}

Result<void> LidarImpl::recover() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return recover_locked();
}

hal::DeviceState LidarImpl::state() const noexcept {
    return state_.load(std::memory_order_acquire);
}

hal::SensorInfo LidarImpl::info() const {
    std::lock_guard<std::mutex> lock(info_mutex_);
    return info_;
}

hal::SensorStats LidarImpl::stats() const noexcept {
    hal::SensorStats result;
    result.frames_received = frames_received_.load(std::memory_order_relaxed);
    result.frames_delivered = frames_delivered_.load(std::memory_order_relaxed);
    result.frames_dropped = frames_dropped_.load(std::memory_order_relaxed);
    result.packets_dropped = imu_dropped_.load(std::memory_order_relaxed);
    result.timeouts = timeouts_.load(std::memory_order_relaxed);
    result.sdk_errors = sdk_errors_.load(std::memory_order_relaxed);
    result.reconnects = reconnects_.load(std::memory_order_relaxed);
    result.last_sequence = last_sequence_.load(std::memory_order_relaxed);
    result.last_error_vendor_code = last_vendor_code_.load(std::memory_order_relaxed);
    result.last_frame_stamp =
        time::Timestamp::from_nanoseconds(last_frame_stamp_ns_.load(std::memory_order_relaxed));
    result.last_frame_receive_stamp =
        time::Timestamp::from_nanoseconds(last_receive_stamp_ns_.load(std::memory_order_relaxed));

    const std::shared_ptr<CallbackSink> sink =
        std::atomic_load_explicit(&sink_, std::memory_order_acquire);
    if (sink != nullptr) {
        result.packets_received = sink->packets_received.load(std::memory_order_relaxed);
        result.packets_dropped += sink->packets_dropped.load(std::memory_order_relaxed);
        result.bytes_received = sink->bytes_received.load(std::memory_order_relaxed);
    }
    return result;
}

Result<void> LidarImpl::read(hal::FrameLease<hal::PointCloudFrame>& out, time::Duration timeout) {
    if (state_.load(std::memory_order_acquire) != hal::DeviceState::kStreaming ||
        pool_ == nullptr) {
        return make_error(ErrorCode::kNotOpen, "Livox 雷达未开始取流");
    }

    const time::Timestamp start = time::SteadyClock().now();
    while (true) {
        hal::FrameLease<hal::PointCloudFrame> lease = pool_->try_acquire_latest();
        if (lease) {
            frames_delivered_.fetch_add(1, std::memory_order_relaxed);
            last_sequence_.store(lease->sequence, std::memory_order_relaxed);
            const std::int64_t stamp = lease->base_time.is_zero()
                                           ? lease->receive_stamp.nanoseconds_since_epoch()
                                           : lease->base_time.nanoseconds_since_epoch();
            last_frame_stamp_ns_.store(stamp, std::memory_order_relaxed);
            last_receive_stamp_ns_.store(lease->receive_stamp.nanoseconds_since_epoch(),
                                         std::memory_order_relaxed);
            out = std::move(lease);
            return Result<void>();
        }

        if (state_.load(std::memory_order_acquire) != hal::DeviceState::kStreaming) {
            return make_error(ErrorCode::kNotOpen, "Livox 雷达在读取过程中停止");
        }
        if (timeout.nanoseconds() <= 0) {
            timeouts_.fetch_add(1, std::memory_order_relaxed);
            return make_error(ErrorCode::kTimeout, "Livox 点云读取超时");
        }
        if (time::SteadyClock().now() - start >= timeout) {
            timeouts_.fetch_add(1, std::memory_order_relaxed);
            return make_error(ErrorCode::kTimeout, "Livox 点云读取超时");
        }
        sleep_microseconds(kPollMicroseconds);
    }
}

Result<void> LidarImpl::read_imu(hal::ImuSample& out, time::Duration timeout) {
    if (state_.load(std::memory_order_acquire) != hal::DeviceState::kStreaming ||
        imu_queue_ == nullptr) {
        return make_error(ErrorCode::kNotOpen, "Livox 雷达未开始取流");
    }

    hal::ImuSample sample;
    if (imu_queue_->try_pop(sample)) {
        out = sample;
        return Result<void>();
    }
    if (timeout.nanoseconds() <= 0) {
        return make_error(ErrorCode::kNoData, "Livox IMU 暂无数据");
    }

    const time::Timestamp start = time::SteadyClock().now();
    while (time::SteadyClock().now() - start < timeout) {
        if (imu_queue_->try_pop(sample)) {
            out = sample;
            return Result<void>();
        }
        if (state_.load(std::memory_order_acquire) != hal::DeviceState::kStreaming) {
            return make_error(ErrorCode::kNotOpen, "Livox 雷达在读取过程中停止");
        }
        sleep_microseconds(kPollMicroseconds);
    }

    timeouts_.fetch_add(1, std::memory_order_relaxed);
    return make_error(ErrorCode::kTimeout, "Livox IMU 读取超时");
}

Result<void> LidarImpl::open_locked() {
    if (state_.load(std::memory_order_acquire) != hal::DeviceState::kClosed) {
        return make_error(ErrorCode::kAlreadyOpen, "Livox 雷达已经打开");
    }
    state_.store(hal::DeviceState::kOpening, std::memory_order_release);

    const auto fail = [this](Error error) -> Result<void> {
        release_resources_locked();
        state_.store(hal::DeviceState::kClosed, std::memory_order_release);
        return error;
    };

    if (config_.selector.serial.empty() && config_.selector.ip_address.empty()) {
        return fail(make_error(ErrorCode::kInvalidArgument,
                               "Livox 必须配置 selector.serial 或 selector.ip_address"));
    }
    if (config_.host_ip.empty() && config_.config_json_path.empty()) {
        return fail(
            make_error(ErrorCode::kInvalidArgument, "Livox 必须配置 host_ip 或 config_json_path"));
    }
    if (config_.packet_ring_slots < 2) {
        return fail(make_error(ErrorCode::kInvalidArgument, "packet_ring_slots 至少为 2"));
    }
    if (config_.max_packet_bytes < kPacketHeaderBytes ||
        config_.max_packet_bytes > kMaxPacketBytes) {
        return fail(
            make_error(ErrorCode::kInvalidArgument, "max_packet_bytes 超出 [36, 1500] 范围"));
    }
    if (config_.point_buffer_capacity == 0) {
        return fail(make_error(ErrorCode::kInvalidArgument, "point_buffer_capacity 必须大于 0"));
    }
    if (config_.frame_buffer_count == 0) {
        return fail(make_error(ErrorCode::kInvalidArgument, "frame_buffer_count 必须大于 0"));
    }
    if (config_.frame_duration_ms == 0) {
        return fail(make_error(ErrorCode::kInvalidArgument, "frame_duration_ms 必须大于 0"));
    }
    if (config_.frame_timeout_ms == 0) {
        return fail(make_error(ErrorCode::kInvalidArgument, "frame_timeout_ms 必须大于 0"));
    }

    std::uint32_t expected_handle = 0;
    if (!config_.selector.ip_address.empty()) {
        expected_handle = ipv4_to_handle(config_.selector.ip_address);
        if (expected_handle == 0) {
            return fail(
                make_error(ErrorCode::kInvalidArgument, "selector.ip_address 不是合法 IPv4 地址"));
        }
    }

    const Result<void> acquired = runtime_->acquire(config_, api_);
    if (!acquired.ok()) {
        return fail(acquired.error());
    }
    runtime_acquired_ = true;

    try {
        queue_ = std::make_shared<container::SpscQueue<PacketSlot>>(config_.packet_ring_slots);
        imu_queue_ = std::make_shared<container::SpscQueue<hal::ImuSample>>(kImuQueueSlots);
        pool_ = std::make_shared<hal::FramePool<hal::PointCloudFrame>>(
            std::max<std::uint32_t>(2, config_.frame_buffer_count),
            static_cast<std::size_t>(config_.point_buffer_capacity) * sizeof(hal::PointXYZITL));

        std::shared_ptr<CallbackSink> sink = std::make_shared<CallbackSink>();
        sink->queue = queue_;
        sink->serial = config_.selector.serial;
        sink->ip_address = config_.selector.ip_address;
        sink->max_packet_bytes = static_cast<std::uint16_t>(config_.max_packet_bytes);
        sink->handle.store(expected_handle, std::memory_order_release);
        std::atomic_store_explicit(&sink_, std::move(sink), std::memory_order_release);
    } catch (const std::bad_alloc&) {
        return fail(make_error(ErrorCode::kInternal, "Livox 驱动内存分配失败"));
    }

    const Result<void> attached = runtime_->attach(sink_);
    if (!attached.ok()) {
        return fail(attached.error());
    }

    state_.store(hal::DeviceState::kOpen, std::memory_order_release);
    return Result<void>();
}

Result<void> LidarImpl::start_locked() {
    const hal::DeviceState current = state_.load(std::memory_order_acquire);
    if (current == hal::DeviceState::kStreaming) {
        return Result<void>();
    }
    if (current != hal::DeviceState::kOpen) {
        return make_error(ErrorCode::kNotOpen, "Livox 雷达未打开");
    }
    if (sink_ == nullptr || queue_ == nullptr || pool_ == nullptr) {
        return make_error(ErrorCode::kInternal, "Livox 雷达资源不完整");
    }

    if (!wait_for_info_locked()) {
        timeouts_.fetch_add(1, std::memory_order_relaxed);
        state_.store(hal::DeviceState::kError, std::memory_order_release);
        return make_error(ErrorCode::kTimeout, "等待 Livox 设备上线超时");
    }

    const std::uint32_t handle = sink_->handle.load(std::memory_order_acquire);
    if (handle == 0) {
        state_.store(hal::DeviceState::kError, std::memory_order_release);
        return make_error(ErrorCode::kInternal, "Livox 设备 handle 未绑定");
    }
    handle_.store(handle, std::memory_order_release);

    const std::uint8_t dev_type = sink_->dev_type.load(std::memory_order_relaxed);
    line_num_ = config_.line_num.value_or(default_line_num(dev_type));

    const Result<void> configured = configure_locked();
    if (!configured.ok()) {
        state_.store(hal::DeviceState::kError, std::memory_order_release);
        return configured;
    }

    drain_queues_locked();

    stop_.store(false, std::memory_order_release);
    sink_->active.store(true, std::memory_order_release);
    try {
        parse_thread_ = std::thread([this] { parse_loop(); });
    } catch (const std::system_error& error) {
        state_.store(hal::DeviceState::kError, std::memory_order_release);
        return make_error(ErrorCode::kInternal,
                          std::string("创建 Livox 解析线程失败: ") + error.what());
    }

    state_.store(hal::DeviceState::kStreaming, std::memory_order_release);
    update_sensor_info_locked();
    return Result<void>();
}

Result<void> LidarImpl::stop_locked() {
    const hal::DeviceState current = state_.load(std::memory_order_acquire);
    if (current == hal::DeviceState::kClosed || current == hal::DeviceState::kOpen ||
        current == hal::DeviceState::kStopping) {
        return Result<void>();
    }

    state_.store(hal::DeviceState::kStopping, std::memory_order_release);
    if (sink_ != nullptr) {
        sink_->active.store(false, std::memory_order_release);
    }
    stop_.store(true, std::memory_order_release);
    if (parse_thread_.joinable()) {
        parse_thread_.join();
    }

    state_.store(hal::DeviceState::kOpen, std::memory_order_release);
    return Result<void>();
}

Result<void> LidarImpl::close_locked() {
    if (state_.load(std::memory_order_acquire) == hal::DeviceState::kClosed) {
        return Result<void>();
    }

    stop_locked();
    release_resources_locked();
    state_.store(hal::DeviceState::kClosed, std::memory_order_release);
    return Result<void>();
}

Result<void> LidarImpl::recover_locked() {
    if (state_.load(std::memory_order_acquire) == hal::DeviceState::kClosed) {
        return make_error(ErrorCode::kNotOpen, "Livox 雷达未打开，无法恢复");
    }

    state_.store(hal::DeviceState::kRecovering, std::memory_order_release);
    // 先停止解析线程，再释放队列/帧池；设备已离线时线程可能仍在轮询。
    stop_locked();

    // 请求设备复位；离线时失败是预期路径，不阻断重新枚举。
    const std::uint32_t handle = handle_.load(std::memory_order_relaxed);
    if (handle != 0) {
        const Result<void> reset = api_->request_reset(handle);
        if (!reset.ok()) {
            sdk_errors_.fetch_add(1, std::memory_order_relaxed);
            last_vendor_code_.store(reset.error().vendor_code, std::memory_order_relaxed);
        }
    }

    release_resources_locked();
    state_.store(hal::DeviceState::kClosed, std::memory_order_release);

    Result<void> last_error = make_error(ErrorCode::kInternal, "Livox 恢复尚未执行");
    for (const std::int64_t backoff_ms : kRecoverBackoffMs) {
        sleep_milliseconds(backoff_ms);

        const Result<void> opened = open_locked();
        if (!opened.ok()) {
            last_error = opened.error();
            continue;
        }
        const Result<void> started = start_locked();
        if (started.ok()) {
            reconnects_.fetch_add(1, std::memory_order_relaxed);
            return Result<void>();
        }
        last_error = started.error();
        close_locked();
    }

    state_.store(hal::DeviceState::kError, std::memory_order_release);
    return last_error;
}

Result<void> LidarImpl::configure_locked() {
    const std::uint32_t handle = handle_.load(std::memory_order_relaxed);
    Result<void> last_error = make_error(ErrorCode::kSdkError, "Livox 设备配置未执行");

    for (int attempt = 0; attempt < kConfigAttempts; ++attempt) {
        const Result<void> mode = api_->set_work_mode(handle, kWorkModeNormal);
        const Result<void> type = api_->set_point_data_type(handle, data_type_value());
        const Result<void> pattern = api_->set_scan_pattern(handle, scan_pattern_value());

        if (mode.ok() && type.ok() && pattern.ok()) {
            last_error = Result<void>();
            break;
        }

        last_error = !mode.ok() ? mode : (!type.ok() ? type : pattern);
        last_vendor_code_.store(last_error.error().vendor_code, std::memory_order_relaxed);
        sleep_milliseconds(kConfigRetryMs);
    }

    if (!last_error.ok()) {
        sdk_errors_.fetch_add(1, std::memory_order_relaxed);
        return make_error(ErrorCode::kSdkError, "Livox 设备配置失败: " + last_error.error().message,
                          last_error.error().vendor_code);
    }

    if (config_.dual_emit) {
        const Result<void> dual = api_->set_dual_emit(handle, true);
        if (!dual.ok()) {
            // 双回波是可选能力；设备不支持时保留主流程。
            sdk_errors_.fetch_add(1, std::memory_order_relaxed);
            last_vendor_code_.store(dual.error().vendor_code, std::memory_order_relaxed);
        }
    }
    return Result<void>();
}

bool LidarImpl::wait_for_info_locked() {
    const time::Timestamp deadline =
        time::SteadyClock().now() + time::Duration::from_milliseconds(kInfoWaitMs);
    while (!sink_->info_received.load(std::memory_order_acquire)) {
        if (time::SteadyClock().now() >= deadline) {
            return false;
        }
        sleep_milliseconds(5);
    }
    return true;
}

void LidarImpl::update_sensor_info_locked() {
    const std::uint8_t dev_type = sink_->dev_type.load(std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(info_mutex_);
    info_.vendor = "Livox";
    info_.model = model_name(dev_type);
    info_.serial = sink_->serial;
    info_.user_defined_name = config_.selector.user_defined_name;
    info_.firmware_version.clear();
    info_.transport = hal::Transport::kGigE;
    info_.ip_address = sink_->ip_address.empty() ? config_.selector.ip_address : sink_->ip_address;
    info_.frame_id = config_.frame_id;
}

void LidarImpl::parse_loop() {
    FrameAssembler assembler(pool_, config_.point_buffer_capacity, line_num_, config_.frame_id,
                             config_.frame_duration_ms);
    const std::int64_t timeout_ns = static_cast<std::int64_t>(config_.frame_timeout_ms) * 1'000'000;

    PacketSlot slot;
    while (!stop_.load(std::memory_order_acquire)) {
        bool processed = false;
        while (queue_->try_pop(slot)) {
            processed = true;
            if (stop_.load(std::memory_order_relaxed)) {
                break;
            }
            handle_packet(slot, assembler);
        }

        const std::int64_t now_ns = time::SteadyClock().now().nanoseconds_since_epoch();
        if (assembler.publish_if_idle(static_cast<std::uint64_t>(now_ns), timeout_ns)) {
            frames_received_.fetch_add(1, std::memory_order_relaxed);
        }

        if (!processed) {
            sleep_microseconds(kPollMicroseconds);
        }
    }

    if (assembler.flush()) {
        frames_received_.fetch_add(1, std::memory_order_relaxed);
    }
}

void LidarImpl::handle_packet(const PacketSlot& slot, FrameAssembler& assembler) {
    const Result<PacketHeader> header = parse_header(slot.data.data(), slot.length);
    if (!header.ok()) {
        if (sink_ != nullptr) {
            sink_->packets_dropped.fetch_add(1, std::memory_order_relaxed);
        }
        return;
    }

    if (header.value().data_type == kDataTypeImu) {
        if (!config_.imu_enabled) {
            return;
        }
        hal::ImuSample sample;
        const Result<void> parsed =
            parse_imu(header.value(), slot.data.data(), slot.length, sample);
        if (!parsed.ok()) {
            if (sink_ != nullptr) {
                sink_->packets_dropped.fetch_add(1, std::memory_order_relaxed);
            }
            return;
        }

        // Livox 协议中加速度单位是 g；HAL 的 ImuSample 约定为 m/s^2。
        sample.acc_x *= kStandardGravity;
        sample.acc_y *= kStandardGravity;
        sample.acc_z *= kStandardGravity;

        sample.receive_stamp =
            time::Timestamp::from_nanoseconds(static_cast<std::int64_t>(slot.receive_steady_ns));
        sample.device_time_ns = static_cast<std::int64_t>(header.value().timestamp_ns);
        sample.sync = sync_from_time_type(header.value().time_type);
        if (sample.sync == hal::TimestampSync::kPtp || sample.sync == hal::TimestampSync::kGpsPps) {
            sample.stamp = time::Timestamp::from_nanoseconds(sample.device_time_ns);
        }
        sample.sequence = ++imu_sequence_;
        sample.frame_id = config_.frame_id;

        if (!imu_queue_->try_push(std::move(sample))) {
            imu_dropped_.fetch_add(1, std::memory_order_relaxed);
            if (sink_ != nullptr) {
                sink_->packets_dropped.fetch_add(1, std::memory_order_relaxed);
            }
        }
        return;
    }

    const std::uint64_t dropped_before = assembler.dropped_frames();
    const bool published =
        assembler.on_packet(header.value(), slot.data.data(), slot.length, slot.receive_steady_ns);
    if (published) {
        frames_received_.fetch_add(1, std::memory_order_relaxed);
    }

    const std::uint64_t dropped_after = assembler.dropped_frames();
    if (dropped_after > dropped_before) {
        frames_dropped_.fetch_add(dropped_after - dropped_before, std::memory_order_relaxed);
    }
}

void LidarImpl::drain_queues_locked() noexcept {
    if (queue_ != nullptr) {
        PacketSlot slot;
        while (queue_->try_pop(slot)) {
        }
    }
    if (imu_queue_ != nullptr) {
        hal::ImuSample sample;
        while (imu_queue_->try_pop(sample)) {
        }
    }
}

void LidarImpl::release_resources_locked() noexcept {
    if (sink_ != nullptr) {
        runtime_->detach(sink_);
        std::atomic_store_explicit(&sink_, std::shared_ptr<CallbackSink>(),
                                   std::memory_order_release);
    }
    queue_.reset();
    imu_queue_.reset();
    pool_.reset();

    if (runtime_acquired_) {
        runtime_->release();
        runtime_acquired_ = false;
    }
}

std::uint8_t LidarImpl::data_type_value() const noexcept {
    switch (config_.data_type) {
    case hal::LidarConfig::PointDataType::kHigh:
        return kDataTypeHigh;
    case hal::LidarConfig::PointDataType::kLow:
        return kDataTypeLow;
    case hal::LidarConfig::PointDataType::kSpherical:
        return kDataTypeSpherical;
    case hal::LidarConfig::PointDataType::kDualEcho:
        return kDataTypeDualEcho;
    }
    return kDataTypeHigh;
}

std::uint8_t LidarImpl::scan_pattern_value() const noexcept {
    return config_.scan_pattern == hal::LidarConfig::ScanPattern::kRepetitive
               ? kScanPatternRepetitive
               : kScanPatternNonRepetitive;
}

Result<std::unique_ptr<hal::Lidar>> make_lidar(const hal::LidarConfig& config,
                                               std::shared_ptr<LivoxApi> api) {
    if (api == nullptr) {
        return make_error(ErrorCode::kInvalidArgument, "Livox SDK 函数表为空");
    }
    return std::unique_ptr<hal::Lidar>(new LidarImpl(config, std::move(api)));
}

} // namespace foray::platform::driver::livox::detail
