#pragma once

#include <string>
#include <cstddef>
#include <cstdint>
#include <sys/types.h>

namespace foray::platform::link {
    class UsbCdcPort
    {
        public:
        explicit UsbCdcPort(std::string device_path);
        UsbCdcPort(const UsbCdcPort& ) = delete; // 禁止创建新对象时候 复制fd_
        UsbCdcPort& operator = (const UsbCdcPort&) = delete; // 禁止覆盖已有
        bool open();
        bool is_open() const;
        void close();
        ~UsbCdcPort(); // 析构函数
        // 读取原始字节，返回实际读取数量
        ssize_t read_bytes(std::uint8_t* buffer, std::size_t capacity);
        // 写入原始字节，返回实际写入数量
        ssize_t write_bytes(const std::uint8_t* data, std::size_t size);
        private:
        std::string device_path_;  // 保存usb路径
        int fd_ = -1; // 判读是否连接
    };


}  // namespace foray::platform::link
