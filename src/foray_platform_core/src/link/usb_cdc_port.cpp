#include "foray/platform/link/usb_cdc_port.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <termios.h>

namespace foray::platform::link {
    UsbCdcPort::UsbCdcPort(std::string device_path)
    : device_path_(device_path)
    {

    }
    // 打开设备后保存文件描述符，后续读写复用它。
    bool UsbCdcPort::open(){
        if(fd_ >= 0) {
            return true;
        }
        fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); // 调用linux打开
        if(fd_ < 0) {
            return false;
        }
        termios settings{};
        if(::tcgetattr(fd_, &settings) != 0) {
            close();
            return false;
        }
        ::cfmakeraw(&settings);
        if (::tcsetattr(fd_, TCSANOW, &settings) != 0) {
            close();
            return false;
        }
        return true;
    }
    bool UsbCdcPort::is_open() const { return fd_ >= 0; }
    // 主动关闭并清除文件描述符。
    void UsbCdcPort::close()
    {
        if(fd_ >= 0){
            ::close(fd_);
            fd_ = -1;
        }
    }
    // 对象析构时自动关闭设备。
    UsbCdcPort::~UsbCdcPort(){
        close();
    }
    // 最多读取调用方缓冲区能够容纳的字节数。
    ssize_t UsbCdcPort::read_bytes(std::uint8_t* buffer, std::size_t capacity)
    {
        if(fd_ < 0){
            errno = EBADF; // 设置错误设备编号无效
            return -1;
        }
        return ::read(fd_, buffer, capacity);
    }
    // 这里只写原始字节；是否构成完整协议帧由上层负责。
    ssize_t UsbCdcPort::write_bytes(const std::uint8_t* data, std::size_t size)
    {
        if(fd_ < 0){
            errno = EBADF;
            return -1;
        }
        return ::write(fd_ , data, size);
    }

}  // namespace foray::platform::link
