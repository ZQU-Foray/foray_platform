#pragma once

#include <memory>

#include "foray/platform/net/backend.hpp"

namespace foray::platform::net {

/**
 * @brief 创建 rtnetlink 后端（无 NetworkManager 环境的回退路径）。
 *
 * 需要 `CAP_NET_ADMIN`；无权限时操作返回 `kPermissionDenied`。
 */
Result<std::unique_ptr<NetBackend>> make_netlink_backend();

} // namespace foray::platform::net
