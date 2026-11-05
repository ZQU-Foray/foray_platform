#pragma once

#include <cstdint>
#include <string>

#include "foray/platform/result/result.hpp"

namespace foray::platform::driver::livox::detail {

/// @brief 是否为合法的 IPv4 点分字符串。
bool is_valid_ipv4(const std::string& ip) noexcept;

/// @brief 该 IPv4 是否配置在本机网卡上（用于启动期提示配置错误）。
bool is_local_ipv4(const std::string& ip) noexcept;

/**
 * @brief 生成 Livox SDK2 的设备/网络 JSON。
 *
 * 端口取 SDK sample 的默认值（雷达侧 56100–56500，主机侧 56101–56501），
 * 见 Livox-SDK2 v1.3.1 `samples/livox_lidar_quick_start`。
 */
std::string build_config_json(const std::string& host_ip);

/**
 * @brief 写入 JSON 文件，权限 0600。
 *
 * 目录不存在时自动创建；已存在文件会被覆盖。
 */
Result<void> write_config_json(const std::string& path, const std::string& host_ip);

/// @brief 生成默认配置文件路径：`<runtime_dir>/foray_livox/livox_config_<ip>.json`。
std::string default_config_path(const std::string& runtime_dir, const std::string& host_ip);

} // namespace foray::platform::driver::livox::detail
