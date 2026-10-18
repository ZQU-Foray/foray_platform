#include "foray/platform/hal/config_binder.hpp"

#include <cstdint>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "foray/platform/hal/types.hpp"

namespace foray::platform::hal {

namespace {

param::ParamSpec make_spec(std::string name, param::ParamType type,
                           param::ParamValue default_value) {
    param::ParamSpec spec;
    spec.name = std::move(name);
    spec.type = type;
    spec.default_value = std::move(default_value);
    return spec;
}

void allow(param::ParamSpec& spec, std::initializer_list<const char*> values) {
    for (const char* value : values) {
        spec.allowed_values.emplace_back(value);
    }
}

void add(param::ParamSchema& schema, param::ParamSpec spec) {
    param::ParamResult result = schema.add(std::move(spec));
    if (!result.ok()) {
        throw std::logic_error("invalid built-in HAL schema: " + result.to_string());
    }
}

template <typename T> Result<T> invalid(std::string message) {
    return Result<T>::failure(Error{ErrorCode::kInvalidArgument, std::move(message), 0});
}

std::optional<Transport> parse_transport(std::string_view text) {
    if (text == "any") {
        return Transport::kAny;
    }
    if (text == "gige") {
        return Transport::kGigE;
    }
    if (text == "usb3") {
        return Transport::kUsb3;
    }
    return std::nullopt;
}

std::optional<PixelFormat> parse_pixel_format(std::string_view text) {
    if (text == "mono8") {
        return PixelFormat::kMono8;
    }
    if (text == "bayer_rgg8") {
        return PixelFormat::kBayerRG8;
    }
    if (text == "bayer_grbg8") {
        return PixelFormat::kBayerGR8;
    }
    if (text == "bayer_gbrg8") {
        return PixelFormat::kBayerGB8;
    }
    if (text == "bayer_bggr8") {
        return PixelFormat::kBayerBG8;
    }
    return std::nullopt;
}

std::optional<TriggerMode> parse_trigger_mode(std::string_view text) {
    if (text == "off") {
        return TriggerMode::kOff;
    }
    if (text == "software") {
        return TriggerMode::kSoftware;
    }
    if (text == "line0") {
        return TriggerMode::kLine0;
    }
    if (text == "line1") {
        return TriggerMode::kLine1;
    }
    if (text == "line2") {
        return TriggerMode::kLine2;
    }
    if (text == "line3") {
        return TriggerMode::kLine3;
    }
    return std::nullopt;
}

std::optional<TriggerActivation> parse_trigger_activation(std::string_view text) {
    if (text == "rising_edge") {
        return TriggerActivation::kRisingEdge;
    }
    if (text == "falling_edge") {
        return TriggerActivation::kFallingEdge;
    }
    if (text == "level_high") {
        return TriggerActivation::kLevelHigh;
    }
    if (text == "level_low") {
        return TriggerActivation::kLevelLow;
    }
    return std::nullopt;
}

std::optional<LidarConfig::PointDataType> parse_data_type(std::string_view text) {
    if (text == "high") {
        return LidarConfig::PointDataType::kHigh;
    }
    if (text == "low") {
        return LidarConfig::PointDataType::kLow;
    }
    if (text == "spherical") {
        return LidarConfig::PointDataType::kSpherical;
    }
    if (text == "dual_echo") {
        return LidarConfig::PointDataType::kDualEcho;
    }
    return std::nullopt;
}

std::optional<LidarConfig::ScanPattern> parse_scan_pattern(std::string_view text) {
    if (text == "non_repetitive") {
        return LidarConfig::ScanPattern::kNonRepetitive;
    }
    if (text == "repetitive") {
        return LidarConfig::ScanPattern::kRepetitive;
    }
    return std::nullopt;
}

} // namespace

param::ParamSchema make_camera_param_schema() {
    param::ParamSchema schema;

    param::ParamSpec serial =
        make_spec("camera.serial", param::ParamType::kString, param::ParamValue(""));
    serial.hot_reloadable = false;
    serial.description = "相机序列号（首选设备选择条件）";
    add(schema, std::move(serial));

    param::ParamSpec user_name =
        make_spec("camera.user_defined_name", param::ParamType::kString, param::ParamValue(""));
    user_name.hot_reloadable = false;
    add(schema, std::move(user_name));

    param::ParamSpec model =
        make_spec("camera.model", param::ParamType::kString, param::ParamValue(""));
    model.hot_reloadable = false;
    add(schema, std::move(model));

    param::ParamSpec ip =
        make_spec("camera.ip_address", param::ParamType::kString, param::ParamValue(""));
    ip.hot_reloadable = false;
    ip.description = "GigE 相机 IP";
    add(schema, std::move(ip));

    param::ParamSpec transport =
        make_spec("camera.transport", param::ParamType::kString, param::ParamValue("any"));
    transport.hot_reloadable = false;
    allow(transport, {"any", "gige", "usb3"});
    add(schema, std::move(transport));

    param::ParamSpec index =
        make_spec("camera.index", param::ParamType::kInt, param::ParamValue(0));
    index.minimum = param::ParamValue(0);
    index.maximum = param::ParamValue(255);
    index.hot_reloadable = false;
    add(schema, std::move(index));

    param::ParamSpec width =
        make_spec("camera.width", param::ParamType::kInt, param::ParamValue(0));
    width.minimum = param::ParamValue(0);
    width.maximum = param::ParamValue(65535);
    width.hot_reloadable = false;
    add(schema, std::move(width));

    param::ParamSpec height =
        make_spec("camera.height", param::ParamType::kInt, param::ParamValue(0));
    height.minimum = param::ParamValue(0);
    height.maximum = param::ParamValue(65535);
    height.hot_reloadable = false;
    add(schema, std::move(height));

    param::ParamSpec offset_x =
        make_spec("camera.offset_x", param::ParamType::kInt, param::ParamValue(0));
    offset_x.minimum = param::ParamValue(0);
    offset_x.maximum = param::ParamValue(65535);
    offset_x.hot_reloadable = false;
    add(schema, std::move(offset_x));

    param::ParamSpec offset_y =
        make_spec("camera.offset_y", param::ParamType::kInt, param::ParamValue(0));
    offset_y.minimum = param::ParamValue(0);
    offset_y.maximum = param::ParamValue(65535);
    offset_y.hot_reloadable = false;
    add(schema, std::move(offset_y));

    param::ParamSpec format = make_spec("camera.pixel_format", param::ParamType::kString,
                                        param::ParamValue("bayer_rgg8"));
    format.hot_reloadable = false;
    allow(format, {"mono8", "bayer_rgg8", "bayer_grbg8", "bayer_gbrg8", "bayer_bggr8"});
    add(schema, std::move(format));

    param::ParamSpec frame_rate =
        make_spec("camera.frame_rate_hz", param::ParamType::kDouble, param::ParamValue(0.0));
    frame_rate.minimum = param::ParamValue(0.0);
    frame_rate.maximum = param::ParamValue(1000.0);
    frame_rate.description = "自由运行帧率；0 表示未设置，触发模式下必须为 0";
    add(schema, std::move(frame_rate));

    param::ParamSpec exposure =
        make_spec("camera.exposure_us", param::ParamType::kDouble, param::ParamValue(-1.0));
    exposure.minimum = param::ParamValue(-1.0);
    exposure.maximum = param::ParamValue(10000000.0);
    exposure.description = "曝光时间（微秒）；-1 表示未设置";
    add(schema, std::move(exposure));

    param::ParamSpec gain =
        make_spec("camera.gain_db", param::ParamType::kDouble, param::ParamValue(-1.0));
    gain.minimum = param::ParamValue(-1.0);
    gain.maximum = param::ParamValue(48.0);
    gain.description = "增益（dB）；-1 表示未设置";
    add(schema, std::move(gain));

    param::ParamSpec trigger_mode =
        make_spec("camera.trigger_mode", param::ParamType::kString, param::ParamValue("off"));
    trigger_mode.hot_reloadable = false;
    allow(trigger_mode, {"off", "software", "line0", "line1", "line2", "line3"});
    add(schema, std::move(trigger_mode));

    param::ParamSpec trigger_activation = make_spec(
        "camera.trigger_activation", param::ParamType::kString, param::ParamValue("rising_edge"));
    trigger_activation.hot_reloadable = false;
    allow(trigger_activation, {"rising_edge", "falling_edge", "level_high", "level_low"});
    add(schema, std::move(trigger_activation));

    param::ParamSpec buffer_count =
        make_spec("camera.buffer_count", param::ParamType::kInt, param::ParamValue(4));
    buffer_count.minimum = param::ParamValue(1);
    buffer_count.maximum = param::ParamValue(64);
    buffer_count.hot_reloadable = false;
    add(schema, std::move(buffer_count));

    param::ParamSpec sdk_node_count =
        make_spec("camera.sdk_node_count", param::ParamType::kInt, param::ParamValue(5));
    sdk_node_count.minimum = param::ParamValue(1);
    sdk_node_count.maximum = param::ParamValue(64);
    sdk_node_count.hot_reloadable = false;
    add(schema, std::move(sdk_node_count));

    param::ParamSpec ptp =
        make_spec("camera.enable_ptp", param::ParamType::kBool, param::ParamValue(false));
    ptp.hot_reloadable = false;
    add(schema, std::move(ptp));

    param::ParamSpec unsynced =
        make_spec("camera.allow_unsynced_stamp", param::ParamType::kBool, param::ParamValue(true));
    add(schema, std::move(unsynced));

    param::ParamSpec frame_id =
        make_spec("camera.frame_id", param::ParamType::kString, param::ParamValue("camera"));
    frame_id.hot_reloadable = false;
    add(schema, std::move(frame_id));

    return schema;
}

param::ParamSchema make_lidar_param_schema() {
    param::ParamSchema schema;

    param::ParamSpec serial =
        make_spec("lidar.serial", param::ParamType::kString, param::ParamValue(""));
    serial.hot_reloadable = false;
    add(schema, std::move(serial));

    param::ParamSpec user_name =
        make_spec("lidar.user_defined_name", param::ParamType::kString, param::ParamValue(""));
    user_name.hot_reloadable = false;
    add(schema, std::move(user_name));

    param::ParamSpec model =
        make_spec("lidar.model", param::ParamType::kString, param::ParamValue(""));
    model.hot_reloadable = false;
    add(schema, std::move(model));

    param::ParamSpec ip =
        make_spec("lidar.ip_address", param::ParamType::kString, param::ParamValue(""));
    ip.hot_reloadable = false;
    ip.description = "雷达 IP";
    add(schema, std::move(ip));

    param::ParamSpec transport =
        make_spec("lidar.transport", param::ParamType::kString, param::ParamValue("any"));
    transport.hot_reloadable = false;
    allow(transport, {"any", "gige"});
    add(schema, std::move(transport));

    param::ParamSpec index = make_spec("lidar.index", param::ParamType::kInt, param::ParamValue(0));
    index.minimum = param::ParamValue(0);
    index.maximum = param::ParamValue(255);
    index.hot_reloadable = false;
    add(schema, std::move(index));

    param::ParamSpec host_ip =
        make_spec("lidar.host_ip", param::ParamType::kString, param::ParamValue(""));
    host_ip.hot_reloadable = false;
    host_ip.description = "主机网卡 IP";
    add(schema, std::move(host_ip));

    param::ParamSpec json_path =
        make_spec("lidar.config_json_path", param::ParamType::kString, param::ParamValue(""));
    json_path.hot_reloadable = false;
    add(schema, std::move(json_path));

    param::ParamSpec runtime_dir =
        make_spec("lidar.runtime_dir", param::ParamType::kString, param::ParamValue(""));
    runtime_dir.hot_reloadable = false;
    add(schema, std::move(runtime_dir));

    param::ParamSpec data_type =
        make_spec("lidar.data_type", param::ParamType::kString, param::ParamValue("high"));
    data_type.hot_reloadable = false;
    allow(data_type, {"high", "low", "spherical", "dual_echo"});
    add(schema, std::move(data_type));

    param::ParamSpec scan_pattern = make_spec("lidar.scan_pattern", param::ParamType::kString,
                                              param::ParamValue("non_repetitive"));
    scan_pattern.hot_reloadable = false;
    allow(scan_pattern, {"non_repetitive", "repetitive"});
    add(schema, std::move(scan_pattern));

    param::ParamSpec dual_emit =
        make_spec("lidar.dual_emit", param::ParamType::kBool, param::ParamValue(false));
    dual_emit.hot_reloadable = false;
    add(schema, std::move(dual_emit));

    param::ParamSpec imu_enabled =
        make_spec("lidar.imu_enabled", param::ParamType::kBool, param::ParamValue(true));
    imu_enabled.hot_reloadable = false;
    add(schema, std::move(imu_enabled));

    param::ParamSpec line_num =
        make_spec("lidar.line_num", param::ParamType::kInt, param::ParamValue(0));
    line_num.minimum = param::ParamValue(0);
    line_num.maximum = param::ParamValue(255);
    line_num.hot_reloadable = false;
    add(schema, std::move(line_num));

    param::ParamSpec ring_slots =
        make_spec("lidar.packet_ring_slots", param::ParamType::kInt, param::ParamValue(4096));
    ring_slots.minimum = param::ParamValue(1);
    ring_slots.maximum = param::ParamValue(65536);
    ring_slots.hot_reloadable = false;
    add(schema, std::move(ring_slots));

    param::ParamSpec max_packet =
        make_spec("lidar.max_packet_bytes", param::ParamType::kInt, param::ParamValue(1500));
    max_packet.minimum = param::ParamValue(36);
    max_packet.maximum = param::ParamValue(65535);
    max_packet.hot_reloadable = false;
    add(schema, std::move(max_packet));

    param::ParamSpec point_capacity =
        make_spec("lidar.point_buffer_capacity", param::ParamType::kInt, param::ParamValue(200000));
    point_capacity.minimum = param::ParamValue(1);
    point_capacity.maximum = param::ParamValue(2000000);
    point_capacity.hot_reloadable = false;
    add(schema, std::move(point_capacity));

    param::ParamSpec frame_count =
        make_spec("lidar.frame_buffer_count", param::ParamType::kInt, param::ParamValue(3));
    frame_count.minimum = param::ParamValue(1);
    frame_count.maximum = param::ParamValue(32);
    frame_count.hot_reloadable = false;
    add(schema, std::move(frame_count));

    param::ParamSpec frame_timeout =
        make_spec("lidar.frame_timeout_ms", param::ParamType::kInt, param::ParamValue(200));
    frame_timeout.minimum = param::ParamValue(1);
    frame_timeout.maximum = param::ParamValue(10000);
    frame_timeout.hot_reloadable = false;
    add(schema, std::move(frame_timeout));

    param::ParamSpec frame_id =
        make_spec("lidar.frame_id", param::ParamType::kString, param::ParamValue("lidar"));
    frame_id.hot_reloadable = false;
    add(schema, std::move(frame_id));

    return schema;
}

Result<CameraConfig> bind_camera_config(const param::ParamSnapshot& snapshot) {
    CameraConfig config;

    config.selector.serial = snapshot.get_string("camera.serial", "");
    config.selector.user_defined_name = snapshot.get_string("camera.user_defined_name", "");
    config.selector.model = snapshot.get_string("camera.model", "");
    config.selector.ip_address = snapshot.get_string("camera.ip_address", "");
    config.selector.index = static_cast<std::uint32_t>(snapshot.get_int("camera.index", 0));

    const std::string transport_text = snapshot.get_string("camera.transport", "any");
    const std::optional<Transport> transport = parse_transport(transport_text);
    if (!transport.has_value()) {
        return invalid<CameraConfig>("camera.transport: unknown value '" + transport_text + "'");
    }
    config.selector.transport = *transport;

    const std::int64_t width = snapshot.get_int("camera.width", 0);
    const std::int64_t height = snapshot.get_int("camera.height", 0);
    const std::int64_t offset_x = snapshot.get_int("camera.offset_x", 0);
    const std::int64_t offset_y = snapshot.get_int("camera.offset_y", 0);
    config.width = static_cast<std::uint32_t>(width);
    config.height = static_cast<std::uint32_t>(height);
    config.offset_x = static_cast<std::uint32_t>(offset_x);
    config.offset_y = static_cast<std::uint32_t>(offset_y);

    const std::string format_text = snapshot.get_string("camera.pixel_format", "bayer_rgg8");
    const std::optional<PixelFormat> format = parse_pixel_format(format_text);
    if (!format.has_value()) {
        return invalid<CameraConfig>("camera.pixel_format: unknown value '" + format_text + "'");
    }
    if (!is_supported_pixel_format(*format)) {
        return invalid<CameraConfig>("camera.pixel_format: '" + format_text +
                                     "' is not supported in this phase");
    }
    config.pixel_format = *format;

    const double frame_rate = snapshot.get_double("camera.frame_rate_hz", 0.0);
    if (frame_rate > 0.0) {
        config.frame_rate_hz = frame_rate;
    }

    const double exposure = snapshot.get_double("camera.exposure_us", -1.0);
    if (exposure >= 0.0) {
        config.exposure_us = exposure;
    }

    const double gain = snapshot.get_double("camera.gain_db", -1.0);
    if (gain >= 0.0) {
        config.gain_db = gain;
    }

    const std::string trigger_text = snapshot.get_string("camera.trigger_mode", "off");
    const std::optional<TriggerMode> trigger = parse_trigger_mode(trigger_text);
    if (!trigger.has_value()) {
        return invalid<CameraConfig>("camera.trigger_mode: unknown value '" + trigger_text + "'");
    }
    config.trigger.mode = *trigger;

    const std::string activation_text =
        snapshot.get_string("camera.trigger_activation", "rising_edge");
    const std::optional<TriggerActivation> activation = parse_trigger_activation(activation_text);
    if (!activation.has_value()) {
        return invalid<CameraConfig>("camera.trigger_activation: unknown value '" +
                                     activation_text + "'");
    }
    config.trigger.activation = *activation;

    config.buffer_count = static_cast<std::uint32_t>(snapshot.get_int("camera.buffer_count", 4));
    config.sdk_node_count =
        static_cast<std::uint32_t>(snapshot.get_int("camera.sdk_node_count", 5));
    config.enable_ptp = snapshot.get_bool("camera.enable_ptp", false);
    config.allow_unsynced_stamp = snapshot.get_bool("camera.allow_unsynced_stamp", true);
    config.frame_id = snapshot.get_string("camera.frame_id", "camera");

    if (config.trigger.mode != TriggerMode::kOff && config.frame_rate_hz.has_value()) {
        return invalid<CameraConfig>(
            "camera.frame_rate_hz must be unset (0) when hardware/software trigger is enabled");
    }
    if (config.selector.transport == Transport::kUsb3 && config.selector.serial.empty()) {
        return invalid<CameraConfig>("camera.serial is required for USB3 transport");
    }
    if (config.selector.transport == Transport::kGigE && config.selector.serial.empty() &&
        config.selector.ip_address.empty()) {
        return invalid<CameraConfig>(
            "camera.serial or camera.ip_address is required for GigE transport");
    }
    if (config.selector.transport == Transport::kAny && config.selector.serial.empty() &&
        config.selector.user_defined_name.empty() && config.selector.ip_address.empty() &&
        config.selector.model.empty()) {
        return invalid<CameraConfig>(
            "no camera selector configured (serial / user_defined_name / model / ip_address)");
    }
    if (config.buffer_count == 0 || config.sdk_node_count == 0) {
        return invalid<CameraConfig>(
            "camera.buffer_count and camera.sdk_node_count must be positive");
    }

    return Result<CameraConfig>::success(std::move(config));
}

Result<LidarConfig> bind_lidar_config(const param::ParamSnapshot& snapshot) {
    LidarConfig config;

    config.selector.serial = snapshot.get_string("lidar.serial", "");
    config.selector.user_defined_name = snapshot.get_string("lidar.user_defined_name", "");
    config.selector.model = snapshot.get_string("lidar.model", "");
    config.selector.ip_address = snapshot.get_string("lidar.ip_address", "");
    config.selector.index = static_cast<std::uint32_t>(snapshot.get_int("lidar.index", 0));

    const std::string transport_text = snapshot.get_string("lidar.transport", "any");
    const std::optional<Transport> transport = parse_transport(transport_text);
    if (!transport.has_value()) {
        return invalid<LidarConfig>("lidar.transport: unknown value '" + transport_text + "'");
    }
    if (*transport == Transport::kUsb3) {
        return invalid<LidarConfig>("lidar.transport: Livox is Ethernet-only");
    }
    config.selector.transport = *transport;

    config.host_ip = snapshot.get_string("lidar.host_ip", "");
    config.config_json_path = snapshot.get_string("lidar.config_json_path", "");
    config.runtime_dir = snapshot.get_string("lidar.runtime_dir", "");

    const std::string data_type_text = snapshot.get_string("lidar.data_type", "high");
    const std::optional<LidarConfig::PointDataType> data_type = parse_data_type(data_type_text);
    if (!data_type.has_value()) {
        return invalid<LidarConfig>("lidar.data_type: unknown value '" + data_type_text + "'");
    }
    config.data_type = *data_type;

    const std::string scan_text = snapshot.get_string("lidar.scan_pattern", "non_repetitive");
    const std::optional<LidarConfig::ScanPattern> scan = parse_scan_pattern(scan_text);
    if (!scan.has_value()) {
        return invalid<LidarConfig>("lidar.scan_pattern: unknown value '" + scan_text + "'");
    }
    config.scan_pattern = *scan;

    config.dual_emit = snapshot.get_bool("lidar.dual_emit", false);
    config.imu_enabled = snapshot.get_bool("lidar.imu_enabled", true);

    const std::int64_t line_num = snapshot.get_int("lidar.line_num", 0);
    if (line_num > 0) {
        config.line_num = static_cast<std::uint8_t>(line_num);
    }

    config.packet_ring_slots =
        static_cast<std::uint32_t>(snapshot.get_int("lidar.packet_ring_slots", 4096));
    config.max_packet_bytes =
        static_cast<std::uint32_t>(snapshot.get_int("lidar.max_packet_bytes", 1500));
    config.point_buffer_capacity =
        static_cast<std::uint32_t>(snapshot.get_int("lidar.point_buffer_capacity", 200000));
    config.frame_buffer_count =
        static_cast<std::uint32_t>(snapshot.get_int("lidar.frame_buffer_count", 3));
    config.frame_timeout_ms =
        static_cast<std::uint32_t>(snapshot.get_int("lidar.frame_timeout_ms", 200));
    config.frame_id = snapshot.get_string("lidar.frame_id", "lidar");

    if (config.selector.ip_address.empty()) {
        return invalid<LidarConfig>("lidar.ip_address is required");
    }
    if (config.host_ip.empty()) {
        return invalid<LidarConfig>("lidar.host_ip is required");
    }
    if (config.packet_ring_slots == 0 || config.max_packet_bytes < 36 ||
        config.point_buffer_capacity == 0 || config.frame_buffer_count == 0 ||
        config.frame_timeout_ms == 0) {
        return invalid<LidarConfig>("lidar buffer/timeout values must be positive");
    }

    return Result<LidarConfig>::success(std::move(config));
}

} // namespace foray::platform::hal
