#include "camera_impl.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>
#include <system_error>
#include <utility>

#include "foray/platform/hal/types.hpp"
#include "foray/platform/result/error.hpp"
#include "foray/platform/time/clock.hpp"
#include "mvs_error.hpp"
#include "mvs_nodes.hpp"

namespace foray::platform::driver::hikvision::detail {

namespace {

/// @brief `MV_CC_GetImageBuffer` 的等待窗口。
///
/// 区间取 50–100 ms 的上界：100 ms 与 10 Hz 的下限帧周期同量级，既能及时响应
/// `stop()`，又不会在低帧率下把正常等待算成超时（文档 §5）。
constexpr std::uint32_t kGrabTimeoutMs = 100;

/// @brief 连续失败多少次判定链路已坏。
///
/// 单次 `MV_E_ABNORMAL_IMAGE` 之类的偶发错误不该让相机下线；连续 5 次（约 0.5 s）
/// 才认为需要 `recover()`。
constexpr std::uint32_t kMaxConsecutiveErrors = 5;

/// @brief `read()` 的轮询间隔，与 Livox 驱动保持一致。
constexpr std::uint32_t kPollMicroseconds = 200;

/// @brief 恢复退避序列（毫秒）：100/200/400/800/1600，总计约 3.1 s。
constexpr std::uint32_t kRecoverBackoffMs[] = {100, 200, 400, 800, 1600};

constexpr std::uint32_t kMinimumPoolSlots = 2;

void sleep_milliseconds(std::uint32_t milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void sleep_microseconds(std::uint32_t microseconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

/// @brief 按 serial 去重。
///
/// 主机同时持有多个网段地址时（例如 `192.168.1.51/24` 与 `169.254.114.1/16`），
/// MVS 会把同一台 GigE 相机枚举两次，两条记录的 serial/IP 完全相同。不去重会让
/// `model` 选择路径误判 `kDeviceAmbiguous`（文档 §4.2）。
std::vector<DeviceDescriptor> deduplicate_by_serial(
    const std::vector<DeviceDescriptor>& devices) {
    std::vector<DeviceDescriptor> unique;
    unique.reserve(devices.size());
    for (const auto& device : devices) {
        if (device.serial.empty()) {
            // 没有 serial 的条目无法去重，也无法被 serial 精确匹配，保留给 model/ip 路径。
            unique.push_back(device);
            continue;
        }
        const bool seen = std::any_of(unique.begin(), unique.end(),
                                      [&device](const DeviceDescriptor& candidate) {
                                          return !candidate.serial.empty() &&
                                                 candidate.serial == device.serial;
                                      });
        if (!seen) {
            unique.push_back(device);
        }
    }
    return unique;
}

std::vector<DeviceDescriptor> filter_by(
    const std::vector<DeviceDescriptor>& devices,
    bool (*predicate)(const DeviceDescriptor&, const std::string&), const std::string& wanted) {
    std::vector<DeviceDescriptor> matches;
    for (const auto& device : devices) {
        if (predicate(device, wanted)) {
            matches.push_back(device);
        }
    }
    return matches;
}

bool same_serial(const DeviceDescriptor& device, const std::string& wanted) {
    return device.serial == wanted;
}

bool same_user_defined_name(const DeviceDescriptor& device, const std::string& wanted) {
    return device.user_defined_name == wanted;
}

bool same_model(const DeviceDescriptor& device, const std::string& wanted) {
    return device.model == wanted;
}

bool same_ip(const DeviceDescriptor& device, const std::string& wanted) {
    return device.transport == hal::Transport::kGigE && device.ip_address == wanted;
}

/**
 * @brief HAL 设备选择规则的实现。
 *
 * 优先级：`serial` > `user_defined_name` > `(model + index)` > `ip_address`。
 * 任何一步出现多候选都返回 `kDeviceAmbiguous`，绝不默认取第 0 个。
 */
Result<DeviceDescriptor> select_device(const std::vector<DeviceDescriptor>& devices,
                                       const hal::DeviceSelector& selector) {
    if (selector.serial.empty() && selector.user_defined_name.empty() &&
        selector.model.empty() && selector.ip_address.empty()) {
        return Result<DeviceDescriptor>(make_error(
            ErrorCode::kInvalidArgument,
            "camera.selector 为空：至少给出 serial / user_defined_name / model / ip_address 之一"));
    }

    if (!selector.serial.empty()) {
        const auto matches = filter_by(devices, &same_serial, selector.serial);
        if (matches.empty()) {
            return Result<DeviceDescriptor>(make_error(
                ErrorCode::kDeviceNotFound, "未找到 serial=" + selector.serial + " 的相机"));
        }
        return Result<DeviceDescriptor>(matches.front());
    }

    if (!selector.user_defined_name.empty()) {
        const auto matches =
            filter_by(devices, &same_user_defined_name, selector.user_defined_name);
        if (matches.empty()) {
            return Result<DeviceDescriptor>(
                make_error(ErrorCode::kDeviceNotFound,
                           "未找到 user_defined_name=" + selector.user_defined_name + " 的相机"));
        }
        if (matches.size() > 1) {
            return Result<DeviceDescriptor>(make_error(
                ErrorCode::kDeviceAmbiguous,
                "user_defined_name=" + selector.user_defined_name + " 命中 " +
                    std::to_string(matches.size()) + " 台相机，请改用 serial"));
        }
        return Result<DeviceDescriptor>(matches.front());
    }

    if (!selector.ip_address.empty()) {
        if (selector.transport == hal::Transport::kUsb3) {
            return Result<DeviceDescriptor>(make_error(
                ErrorCode::kInvalidArgument, "camera.ip_address 只适用于 GigE 传输，不能用于 USB3"));
        }
        const auto matches = filter_by(devices, &same_ip, selector.ip_address);
        if (matches.empty()) {
            return Result<DeviceDescriptor>(make_error(
                ErrorCode::kDeviceNotFound, "未找到 ip=" + selector.ip_address + " 的相机"));
        }
        if (matches.size() > 1) {
            return Result<DeviceDescriptor>(
                make_error(ErrorCode::kDeviceAmbiguous,
                           "ip=" + selector.ip_address + " 命中 " + std::to_string(matches.size()) +
                               " 台相机，请改用 serial"));
        }
        return Result<DeviceDescriptor>(matches.front());
    }

    const auto matches = filter_by(devices, &same_model, selector.model);
    if (matches.empty()) {
        return Result<DeviceDescriptor>(
            make_error(ErrorCode::kDeviceNotFound, "未找到 model=" + selector.model + " 的相机"));
    }
    if (matches.size() == 1) {
        return Result<DeviceDescriptor>(matches.front());
    }
    // 多台同型号：`index` 默认值是 0，无法区分「用户没写 index」与「用户要第 0 台」。
    // 因此只有显式给了非 0 index 才认，否则一律要求调用方补 serial 或显式 index——
    // 默认取第 0 台正是 HAL 明令禁止的行为。
    if (selector.index == 0) {
        return Result<DeviceDescriptor>(make_error(
            ErrorCode::kDeviceAmbiguous,
            "model=" + selector.model + " 命中 " + std::to_string(matches.size()) +
                " 台相机，请改用 serial 或显式指定 camera.index"));
    }
    if (selector.index >= matches.size()) {
        return Result<DeviceDescriptor>(make_error(
            ErrorCode::kDeviceNotFound,
            "model=" + selector.model + " 命中 " + std::to_string(matches.size()) +
                " 台相机，camera.index=" + std::to_string(selector.index) + " 越界"));
    }
    return Result<DeviceDescriptor>(matches[selector.index]);
}

hal::SensorInfo build_info(const DeviceDescriptor& device) {
    hal::SensorInfo info;
    info.vendor = "Hikrobot";
    info.model = device.model;
    info.serial = device.serial;
    info.user_defined_name = device.user_defined_name;
    info.firmware_version = device.firmware;
    info.transport = device.transport;
    info.ip_address = device.ip_address;
    return info;
}

} // namespace

CameraImpl::CameraImpl(hal::CameraConfig config, std::shared_ptr<MvsApi> api)
    : api_(std::move(api)), config_(std::move(config)) {
    frame_id_ = config_.frame_id;
}

CameraImpl::~CameraImpl() {
    close();
}

hal::DeviceState CameraImpl::state() const noexcept {
    return state_.load(std::memory_order_acquire);
}

hal::SensorInfo CameraImpl::info() const {
    return info_;
}

hal::SensorStats CameraImpl::stats() const noexcept {
    hal::SensorStats result;
    // 载荷统计要求 SDK 层没有丢掉整帧，这里把 SDK 返回的帧都算作 received。
    result.frames_received = frames_received_.load(std::memory_order_relaxed);
    result.frames_delivered = frames_delivered_.load(std::memory_order_relaxed);
    result.frames_dropped = frames_dropped_.load(std::memory_order_relaxed);
    // MVS 不暴露包级统计：GigE 丢包只能从帧元数据 `lost_packets` 观察。
    result.packets_received = 0;
    result.packets_dropped = 0;
    result.bytes_received = bytes_received_.load(std::memory_order_relaxed);
    result.timeouts = timeouts_.load(std::memory_order_relaxed);
    result.sdk_errors = sdk_errors_.load(std::memory_order_relaxed);
    result.reconnects = reconnects_.load(std::memory_order_relaxed);
    result.last_sequence = last_sequence_.load(std::memory_order_relaxed);
    result.last_error_vendor_code = last_vendor_code_.load(std::memory_order_relaxed);
    result.last_frame_stamp = time::Timestamp::from_nanoseconds(
        last_frame_stamp_ns_.load(std::memory_order_relaxed));
    result.last_frame_receive_stamp = time::Timestamp::from_nanoseconds(
        last_receive_stamp_ns_.load(std::memory_order_relaxed));
    return result;
}

Result<void> CameraImpl::open() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return open_locked();
}

Result<void> CameraImpl::start() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return start_locked();
}

Result<void> CameraImpl::stop() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return stop_locked();
}

Result<void> CameraImpl::close() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return close_locked();
}

Result<void> CameraImpl::recover() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return recover_locked();
}

Result<void> CameraImpl::open_locked() {
    if (state_.load(std::memory_order_acquire) != hal::DeviceState::kClosed) {
        return Result<void>(
            make_error(ErrorCode::kAlreadyOpen, "相机已经打开，close() 之后才能重新 open()"));
    }
    state_.store(hal::DeviceState::kOpening, std::memory_order_release);

    // 任一阶段失败都要把已拿到的句柄还回去，并把状态退回 kClosed。
    const auto fail = [this](Error error) -> Result<void> {
        release_resources_locked();
        state_.store(hal::DeviceState::kClosed, std::memory_order_release);
        return Result<void>(std::move(error));
    };

    auto devices = api_->enumerate(config_.selector.transport);
    if (!devices.ok()) {
        return fail(devices.error());
    }
    auto selected = select_device(deduplicate_by_serial(devices.value()), config_.selector);
    if (!selected.ok()) {
        return fail(selected.error());
    }

    auto handle = api_->create_handle(selected.value());
    if (!handle.ok()) {
        return fail(handle.error());
    }
    handle_ = handle.value();
    api_->register_exception(handle_, &CameraImpl::on_exception, this);

    if (auto status = api_->open_device(handle_); !status.ok()) {
        return fail(status.error());
    }

    if (auto status = configure_locked(selected.value()); !status.ok()) {
        return fail(status.error());
    }

    {
        std::lock_guard<std::mutex> lock(tuning_mutex_);
        info_ = build_info(selected.value());
        info_.frame_id = frame_id_;
    }

    frames_received_.store(0, std::memory_order_relaxed);
    frames_delivered_.store(0, std::memory_order_relaxed);
    frames_dropped_.store(0, std::memory_order_relaxed);
    bytes_received_.store(0, std::memory_order_relaxed);
    timeouts_.store(0, std::memory_order_relaxed);
    sdk_errors_.store(0, std::memory_order_relaxed);
    consecutive_errors_.store(0, std::memory_order_relaxed);
    last_sequence_.store(0, std::memory_order_relaxed);
    last_vendor_code_.store(0, std::memory_order_relaxed);
    last_frame_stamp_ns_.store(0, std::memory_order_relaxed);
    last_receive_stamp_ns_.store(0, std::memory_order_relaxed);
    sequence_ = 0;
    last_device_frame_number_ = 0;

    state_.store(hal::DeviceState::kOpen, std::memory_order_release);
    return Result<void>();
}

Result<void> CameraImpl::configure_locked(const DeviceDescriptor& device) {
    std::uint32_t optional_errors = 0;
    if (auto status = apply_config(*api_, handle_, config_, optional_errors); !status.ok()) {
        return status;
    }
    sdk_errors_.fetch_add(optional_errors, std::memory_order_relaxed);

    const bool ptp_slave = query_ptp_slave(*api_, handle_);
    const std::uint32_t tick_frequency = query_tick_frequency(*api_, handle_);

    // 自动包长：只对 GigE 有意义，配置显式给了 packet_size 时不覆盖。
    if (config_.packet_size == 0 && device.transport == hal::Transport::kGigE) {
        auto optimal = api_->optimal_packet_size(handle_);
        if (optimal.ok() && optimal.value() != 0) {
            if (auto status = api_->set_int(handle_, node::kPacketSize,
                                            static_cast<std::int64_t>(optimal.value()));
                !status.ok()) {
                sdk_errors_.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }

    // 设备可能不接受配置里的分辨率（例如被 ROI 对齐修正），一律以读回值为准。
    auto width = api_->get_int(handle_, node::kWidth);
    auto height = api_->get_int(handle_, node::kHeight);
    if (!width.ok() || !height.ok() || width.value() <= 0 || height.value() <= 0) {
        return Result<void>(make_error(
            ErrorCode::kInternal, "无法读取相机实际分辨率（Width/Height 节点读取失败）"));
    }

    // 帧元数据里的曝光/增益默认是 0（未开 chunk），这里读一次节点作为兜底值。
    {
        std::lock_guard<std::mutex> tuning_lock(tuning_mutex_);
        const auto exposure = api_->get_float(handle_, node::kExposureTime);
        if (exposure.ok() && exposure.value() > 0.0) {
            exposure_hint_us_ = static_cast<float>(exposure.value());
        }
        const auto gain = api_->get_float(handle_, node::kGain);
        if (gain.ok()) {
            gain_hint_db_ = static_cast<float>(gain.value());
        }
    }

    // 只支持 8 位 Mono/Bayer，字节数就是像素数；这一点由 apply_config 的像素格式校验保证。
    const auto frame_bytes =
        static_cast<std::size_t>(width.value()) * static_cast<std::size_t>(height.value());
    auto slots = static_cast<std::size_t>(
        std::max<std::uint32_t>(kMinimumPoolSlots, config_.buffer_count));
    if (config_.pool_memory_limit_bytes != 0 &&
        frame_bytes * slots > config_.pool_memory_limit_bytes) {
        if (frame_bytes * kMinimumPoolSlots > config_.pool_memory_limit_bytes) {
            return Result<void>(make_error(
                ErrorCode::kFrameTooLarge,
                "camera.pool_memory_limit_bytes 太小：单帧 " + std::to_string(frame_bytes) +
                    " B，至少要能放下 " + std::to_string(kMinimumPoolSlots) + " 帧"));
        }
        slots = kMinimumPoolSlots;
    }

    try {
        pool_ = std::make_shared<hal::FramePool<hal::ImageFrame>>(slots, frame_bytes);
        // 先创建空池再填入像素格式等“每帧固定”字段，取流时只改序号与时间。
        pool_->payload_size();
    } catch (const std::bad_alloc&) {
        return Result<void>(make_error(
            ErrorCode::kInternal, "帧池内存分配失败：需要 " + std::to_string(slots) + " × " +
                                      std::to_string(frame_bytes) + " B"));
    }

    {
        std::lock_guard<std::mutex> lock(tuning_mutex_);
        timestamp_config_ = make_timestamp_config(config_, ptp_slave, tick_frequency);
        frame_id_ = config_.frame_id;
    }
    return Result<void>();
}

Result<void> CameraImpl::start_locked() {
    const auto current = state_.load(std::memory_order_acquire);
    if (current == hal::DeviceState::kStreaming) {
        return Result<void>();
    }
    if (current != hal::DeviceState::kOpen) {
        return Result<void>(
            make_error(ErrorCode::kNotOpen, "start() 之前必须先 open()，当前状态: " +
                                                std::string(hal::to_string(current))));
    }
    if (handle_ == nullptr || pool_ == nullptr) {
        return Result<void>(make_error(ErrorCode::kInternal, "相机资源已释放，无法开始取流"));
    }

    // MV_CC_SetImageNodeNum 必须在 StartGrabbing 之前；节点不存在时只计数不阻断。
    if (auto status = api_->set_image_node_count(handle_, config_.sdk_node_count);
        !status.ok()) {
        sdk_errors_.fetch_add(1, std::memory_order_relaxed);
        last_vendor_code_.store(status.error().vendor_code, std::memory_order_relaxed);
    }

    if (auto status = api_->start_grabbing(handle_); !status.ok()) {
        return status;
    }

    stop_requested_.store(false, std::memory_order_release);
    consecutive_errors_.store(0, std::memory_order_relaxed);
    try {
        grab_thread_ = std::thread([this] { grab_loop(); });
    } catch (const std::system_error& error) {
        static_cast<void>(api_->stop_grabbing(handle_));
        state_.store(hal::DeviceState::kError, std::memory_order_release);
        return Result<void>(make_error(
            ErrorCode::kInternal, std::string("创建取流线程失败: ") + error.what()));
    }

    state_.store(hal::DeviceState::kStreaming, std::memory_order_release);
    return Result<void>();
}

Result<void> CameraImpl::stop_locked() {
    const auto current = state_.load(std::memory_order_acquire);
    // kOpen 已经停了；kClosed 无事可做；kStopping 说明重入，等外层收敛。
    if (current == hal::DeviceState::kClosed || current == hal::DeviceState::kOpen ||
        current == hal::DeviceState::kStopping) {
        return Result<void>();
    }

    state_.store(hal::DeviceState::kStopping, std::memory_order_release);
    stop_requested_.store(true, std::memory_order_release);
    if (grab_thread_.joinable()) {
        grab_thread_.join();
    }
    if (handle_ != nullptr) {
        // 停止取流失败不影响状态机：句柄随后会被 close()/recover() 处理。
        if (auto status = api_->stop_grabbing(handle_); !status.ok()) {
            sdk_errors_.fetch_add(1, std::memory_order_relaxed);
            last_vendor_code_.store(status.error().vendor_code, std::memory_order_relaxed);
        }
    }
    state_.store(hal::DeviceState::kOpen, std::memory_order_release);
    return Result<void>();
}

Result<void> CameraImpl::close_locked() {
    if (state_.load(std::memory_order_acquire) == hal::DeviceState::kClosed) {
        return Result<void>();
    }
    static_cast<void>(stop_locked());
    release_resources_locked();
    state_.store(hal::DeviceState::kClosed, std::memory_order_release);
    return Result<void>();
}

void CameraImpl::release_resources_locked() {
    if (handle_ != nullptr) {
        // 关设备失败也要销毁句柄，否则下一次 open() 会泄漏。
        static_cast<void>(api_->close_device(handle_));
        api_->destroy_handle(handle_);
        handle_ = nullptr;
    }
    pool_.reset();
}

Result<void> CameraImpl::recover_locked() {
    const auto current = state_.load(std::memory_order_acquire);
    if (current == hal::DeviceState::kClosed) {
        return Result<void>(
            make_error(ErrorCode::kNotOpen, "recover() 之前必须先 open()"));
    }
    if (current == hal::DeviceState::kRecovering) {
        return Result<void>(make_error(ErrorCode::kBusy, "相机正在恢复中"));
    }

    state_.store(hal::DeviceState::kRecovering, std::memory_order_release);
    static_cast<void>(stop_locked());
    release_resources_locked();
    state_.store(hal::DeviceState::kClosed, std::memory_order_release);

    Result<void> last_error =
        Result<void>(make_error(ErrorCode::kDisconnected, "相机恢复失败：所有退避重试都未成功"));
    for (const std::uint32_t backoff : kRecoverBackoffMs) {
        sleep_milliseconds(backoff);
        auto opened = open_locked();
        if (!opened.ok()) {
            last_error = opened;
            continue;
        }
        auto started = start_locked();
        if (started.ok()) {
            reconnects_.fetch_add(1, std::memory_order_relaxed);
            return Result<void>();
        }
        last_error = started;
        static_cast<void>(close_locked());
    }

    state_.store(hal::DeviceState::kError, std::memory_order_release);
    return last_error;
}

void CameraImpl::on_exception(void* user, std::uint32_t code) {
    auto* self = static_cast<CameraImpl*>(user);
    if (self == nullptr) {
        return;
    }
    // SDK 线程：只置状态与计数，绝不调用任何控制接口（否则和 close() 抢句柄）。
    self->sdk_errors_.fetch_add(1, std::memory_order_relaxed);
    self->last_vendor_code_.store(static_cast<std::int32_t>(code), std::memory_order_relaxed);
    const auto current = self->state_.load(std::memory_order_acquire);
    if (current == hal::DeviceState::kStreaming || current == hal::DeviceState::kOpen) {
        self->state_.store(hal::DeviceState::kError, std::memory_order_release);
    }
}

void CameraImpl::grab_loop() {
    while (!stop_requested_.load(std::memory_order_acquire)) {
        auto frame = api_->get_frame(handle_, kGrabTimeoutMs);
        if (!frame.ok()) {
            const Error& error = frame.error();
            last_vendor_code_.store(error.vendor_code, std::memory_order_relaxed);
            if (error.code == ErrorCode::kTimeout || error.code == ErrorCode::kNoData) {
                // 超时是正常控制流：继续等下一帧，不 sleep（SDK 内部已经阻塞过）。
                timeouts_.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            sdk_errors_.fetch_add(1, std::memory_order_relaxed);
            const auto streak =
                consecutive_errors_.fetch_add(1, std::memory_order_relaxed) + 1;
            if (streak >= kMaxConsecutiveErrors) {
                // 连续失败意味着链路已坏；交给上层 recover()，线程自行退出。
                state_.store(hal::DeviceState::kError, std::memory_order_release);
                return;
            }
            continue;
        }
        consecutive_errors_.store(0, std::memory_order_relaxed);

        publish_frame(*frame.value());

        // 同一线程归还：MV_CC_FreeImageBuffer 必须与 MV_CC_GetImageBuffer 配对。
        api_->free_frame(handle_, std::move(frame.value()));
    }
}

void CameraImpl::publish_frame(const FrameView& frame) {
    const FrameInfoView& info = frame.info();

    if (!hal::is_supported_pixel_format(info.pixel_format)) {
        sdk_errors_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    // 8 位格式下 stride 等于宽度；帧长度不足说明元数据与分辨率不自洽。
    const std::uint32_t stride = info.width;
    if (stride == 0 || info.height == 0 ||
        info.frame_length < static_cast<std::uint64_t>(stride) * info.height) {
        sdk_errors_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    auto writer_optional = pool_->try_acquire_writer();
    if (!writer_optional.has_value()) {
        frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    auto& writer = *writer_optional;

    const std::size_t copy_bytes =
        std::min<std::size_t>(static_cast<std::size_t>(info.frame_length), writer.payload_size());
    std::memcpy(writer.data(), frame.data(), copy_bytes);

    TimingSample sample;
    sample.device_ticks = info.device_ticks;
    const time::SystemClock wall_clock;
    const time::SteadyClock steady_clock;
    sample.receive_wall_ns = wall_clock.now().nanoseconds_since_epoch();
    sample.receive_steady_ns = steady_clock.now().nanoseconds_since_epoch();

    hal::ImageFrame& out = writer.frame();
    TimingResult timing;
    float exposure_hint = 0.0F;
    float gain_hint = 0.0F;
    {
        std::lock_guard<std::mutex> lock(tuning_mutex_);
        timing = compute_timing(sample, timestamp_config_);
        // frame_id_ 由同一把锁保护：string_view 必须在锁内取，否则会和 apply() 竞争。
        out.frame_id = frame_id_;
        exposure_hint = exposure_hint_us_;
        gain_hint = gain_hint_db_;
    }

    out.data = writer.data();
    out.size = copy_bytes;
    out.width = info.width;
    out.height = info.height;
    out.stride = stride;
    out.pixel_format = info.pixel_format;
    out.sequence = ++sequence_;
    out.device_frame_number = info.frame_number;
    out.trigger_index = info.trigger_index;
    out.stamp = timing.stamp;
    out.receive_stamp = timing.receive_stamp;
    out.device_time_ns = timing.device_time_ns;
    out.device_tick_frequency_hz = timing.device_tick_frequency_hz;
    out.sync = timing.sync;
    // 帧内字段为 0 时退回读回值：SDK 未开 chunk 时这两个字段恒为 0。
    out.exposure_time_us = info.exposure_us > 0.0F ? info.exposure_us : exposure_hint;
    out.gain_db = info.gain_db != 0.0F ? info.gain_db : gain_hint;
    out.lost_packets = info.lost_packets;

    writer.publish();

    frames_received_.fetch_add(1, std::memory_order_relaxed);
    bytes_received_.fetch_add(copy_bytes, std::memory_order_relaxed);
    last_sequence_.store(out.sequence, std::memory_order_relaxed);
    last_frame_stamp_ns_.store(out.stamp.nanoseconds_since_epoch(), std::memory_order_relaxed);
    last_receive_stamp_ns_.store(out.receive_stamp.nanoseconds_since_epoch(),
                                 std::memory_order_relaxed);

    // 设备帧号跳变只作为诊断信号：lost_packets 已经由帧元数据提供。
    last_device_frame_number_ = info.frame_number;
}

Result<void> CameraImpl::read(hal::FrameLease<hal::ImageFrame>& out, time::Duration timeout) {
    const auto current = state_.load(std::memory_order_acquire);
    if (current != hal::DeviceState::kStreaming || pool_ == nullptr) {
        return Result<void>(make_error(
            ErrorCode::kNotOpen,
            "read() 需要相机处于 streaming 状态，当前: " + std::string(hal::to_string(current))));
    }

    const std::int64_t budget_ns = timeout.nanoseconds();
    const time::SteadyClock clock;
    const std::int64_t deadline_ns = clock.now().nanoseconds_since_epoch() + budget_ns;

    for (;;) {
        auto lease = pool_->try_acquire_latest();
        if (lease) {
            frames_delivered_.fetch_add(1, std::memory_order_relaxed);
            out = std::move(lease);
            return Result<void>();
        }

        const auto now_state = state_.load(std::memory_order_acquire);
        if (now_state != hal::DeviceState::kStreaming) {
            return Result<void>(make_error(
                ErrorCode::kNotOpen, "取流已停止，read() 中止，当前状态: " +
                                         std::string(hal::to_string(now_state))));
        }

        if (budget_ns <= 0 || clock.now().nanoseconds_since_epoch() >= deadline_ns) {
            timeouts_.fetch_add(1, std::memory_order_relaxed);
            return Result<void>(make_error(ErrorCode::kTimeout, "等待相机帧超时"));
        }
        sleep_microseconds(kPollMicroseconds);
    }
}

Result<void> CameraImpl::set_exposure(std::optional<double> microseconds) {
    hal::CameraTuning tuning;
    tuning.exposure_us = microseconds;
    return apply(tuning);
}

Result<void> CameraImpl::set_gain(std::optional<double> db) {
    hal::CameraTuning tuning;
    tuning.gain_db = db;
    return apply(tuning);
}

Result<void> CameraImpl::set_frame_rate(std::optional<double> hz) {
    hal::CameraTuning tuning;
    tuning.frame_rate_hz = hz;
    return apply(tuning);
}

Result<void> CameraImpl::apply(const hal::CameraTuning& tuning) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    const auto current = state_.load(std::memory_order_acquire);
    if (current != hal::DeviceState::kOpen && current != hal::DeviceState::kStreaming) {
        return Result<void>(make_error(
            ErrorCode::kNotOpen,
            "apply() 需要相机处于 open 或 streaming 状态，当前: " +
                std::string(hal::to_string(current))));
    }
    if (tuning.empty()) {
        return Result<void>();
    }
    if (handle_ == nullptr) {
        return Result<void>(make_error(ErrorCode::kInternal, "相机句柄已释放，无法调参"));
    }

    if (auto status = apply_tuning(*api_, handle_, tuning); !status.ok()) {
        // 部分字段可能已经生效：不回滚，但要计数并保留原始 vendor_code 供现场定位。
        sdk_errors_.fetch_add(1, std::memory_order_relaxed);
        last_vendor_code_.store(status.error().vendor_code, std::memory_order_relaxed);
        return status;
    }

    {
        std::lock_guard<std::mutex> tuning_lock(tuning_mutex_);
        if (tuning.timestamp_source.has_value()) {
            timestamp_config_.source = *tuning.timestamp_source;
            config_.timestamp_source = *tuning.timestamp_source;
        }
        if (tuning.device_to_host_offset_ns.has_value()) {
            const auto offset = tuning.device_to_host_offset_ns->value_or(0);
            timestamp_config_.device_to_host_offset_ns = offset;
            config_.device_to_host_offset_ns = offset;
        }
        if (tuning.exposure_to_host_latency_ns.has_value()) {
            const auto latency = tuning.exposure_to_host_latency_ns->value_or(0.0);
            timestamp_config_.exposure_to_host_latency_ns =
                static_cast<std::int64_t>(latency);
            config_.exposure_to_host_latency_ns = latency;
        }
        if (tuning.allow_unsynced_stamp.has_value()) {
            timestamp_config_.allow_unsynced_stamp = *tuning.allow_unsynced_stamp;
            config_.allow_unsynced_stamp = *tuning.allow_unsynced_stamp;
        }
        if (tuning.frame_id.has_value()) {
            frame_id_ = *tuning.frame_id;
            config_.frame_id = *tuning.frame_id;
            info_.frame_id = *tuning.frame_id;
        }
        // 这些字段 recover() 之后要沿用，所以同步回 config_。
        if (tuning.exposure_us.has_value()) {
            config_.exposure_us = *tuning.exposure_us;
            // 内层为空表示交回自动曝光，此时设备值由自动算法决定，hint 失效。
            if (tuning.exposure_us->has_value()) {
                exposure_hint_us_ = static_cast<float>(*tuning.exposure_us.value());
            }
        }
        if (tuning.gain_db.has_value()) {
            config_.gain_db = *tuning.gain_db;
            if (tuning.gain_db->has_value()) {
                gain_hint_db_ = static_cast<float>(*tuning.gain_db.value());
            }
        }
        if (tuning.frame_rate_hz.has_value()) {
            config_.frame_rate_hz = *tuning.frame_rate_hz;
        }
        if (tuning.exposure_auto.has_value()) {
            config_.exposure_auto = *tuning.exposure_auto;
        }
        if (tuning.gain_auto.has_value()) {
            config_.gain_auto = *tuning.gain_auto;
        }
        if (tuning.trigger_debounce_us.has_value()) {
            config_.trigger.debounce_us = *tuning.trigger_debounce_us;
        }
    }
    return Result<void>();
}

Result<void> CameraImpl::trigger() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    const auto current = state_.load(std::memory_order_acquire);
    if (current != hal::DeviceState::kStreaming) {
        return Result<void>(make_error(
            ErrorCode::kNotOpen,
            "trigger() 需要相机处于 streaming 状态，当前: " +
                std::string(hal::to_string(current))));
    }
    const auto source = config_.trigger.source.value_or(config_.trigger.mode);
    if (source != hal::TriggerMode::kSoftware) {
        return Result<void>(make_error(
            ErrorCode::kNotSupported, "trigger() 仅在 camera.trigger.mode = software 时可用"));
    }
    if (handle_ == nullptr) {
        return Result<void>(make_error(ErrorCode::kInternal, "相机句柄已释放，无法软触发"));
    }

    if (auto status = api_->trigger_software(handle_); !status.ok()) {
        sdk_errors_.fetch_add(1, std::memory_order_relaxed);
        last_vendor_code_.store(status.error().vendor_code, std::memory_order_relaxed);
        return status;
    }
    return Result<void>();
}

std::vector<std::string> CameraImpl::supported_keys() const {
    // 与 `docs/camera_config.md` §3 的键空间一致；返回空会让配置期跳过逐键校验。
    return {
        "camera.serial",
        "camera.user_defined_name",
        "camera.model",
        "camera.ip_address",
        "camera.transport",
        "camera.index",
        "camera.width",
        "camera.height",
        "camera.offset_x",
        "camera.offset_y",
        "camera.binning_horizontal",
        "camera.binning_vertical",
        "camera.decimation_horizontal",
        "camera.decimation_vertical",
        "camera.pixel_format",
        "camera.frame_rate_hz",
        "camera.frame_rate_enable",
        "camera.exposure_us",
        "camera.exposure_auto",
        "camera.exposure_auto_lower_us",
        "camera.exposure_auto_upper_us",
        "camera.gain_db",
        "camera.gain_auto",
        "camera.gain_auto_lower_db",
        "camera.gain_auto_upper_db",
        "camera.trigger_mode",
        "camera.trigger_activation",
        "camera.trigger_source",
        "camera.trigger_debounce_us",
        "camera.buffer_count",
        "camera.sdk_node_count",
        "camera.pool_memory_limit_bytes",
        "camera.packet_size",
        "camera.inter_packet_delay_us",
        "camera.heartbeat_timeout_ms",
        "camera.net_export",
        "camera.enable_ptp",
        "camera.device_to_host_offset_ns",
        "camera.exposure_to_host_latency_ns",
        "camera.timestamp_source",
        "camera.allow_unsynced_stamp",
        "camera.frame_id",
    };
}

} // namespace foray::platform::driver::hikvision::detail
