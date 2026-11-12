#pragma once

#include <memory>

#include "diesel/platform/net/backend.hpp"
#include "diesel/platform/net/process.hpp"

namespace diesel::platform::net {

/// @brief `nmcli` 可用且 NetworkManager 正在运行时返回 true。
Result<bool> network_manager_available(ProcessRunner& runner);

/**
 * @brief 创建 NetworkManager 后端。
 *
 * 通过持久化 profile（bridge / bridge-slave）完成配置，
 * 掉电重启后 NetworkManager 仍会恢复同一套拓扑；
 * 全部写操作都需要 root（或 polkit 授权）。
 */
Result<std::unique_ptr<NetBackend>> make_network_manager_backend(ProcessRunner& runner);

} // namespace diesel::platform::net
