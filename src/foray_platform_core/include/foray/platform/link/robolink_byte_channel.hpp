#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <sys/types.h>

#include "foray/platform/link/robolink_frame_parser.hpp"
#include "foray/platform/link/usb_cdc_port.hpp"

namespace foray::platform::link {

// 连接 USB 字节通道与候选帧解析器 不负责 HELLO、控制频率或执行器使能
class RoboLinkByteChannel {
public:
    explicit RoboLinkByteChannel(UsbCdcPort& port);

    // 一次只容纳一帧待发送；上一帧未发完时返回 false，不覆盖旧数据。
    bool queue_frame(const std::vector<std::uint8_t>& frame);
    bool has_pending_write() const;

    // 非阻塞发送一次；返回本次成功写入的字节数，EAGAIN 返回 0，其它错误返回 -1。
    ssize_t flush_once();

    // 非阻塞读取一次，并返回这次拼出的完整帧；其它读取错误抛 system_error。
    std::vector<std::vector<std::uint8_t>> receive_once();

    // 断线后清除旧帧和未发完的数据，避免重连时发送过期控制。
    void reset();

private:
    UsbCdcPort& port_;
    RoboLinkFrameParser parser_;
    std::vector<std::uint8_t> tx_pending_;
    std::size_t tx_offset_ = 0;
};

}  // namespace foray::platform::link
