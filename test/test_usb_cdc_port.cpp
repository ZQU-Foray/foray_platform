#include <gtest/gtest.h>

#include <cerrno>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include "foray_platform/usb_cdc_port.hpp"

namespace foray_platform {
namespace {

struct TestTerminal {
    int master_fd = -1;
    int slave_fd = -1;
    char path[128]{};

    bool open()
    {
        return ::openpty(&master_fd, &slave_fd, nullptr, nullptr, nullptr) == 0 &&
               ::ttyname_r(slave_fd, path, sizeof(path)) == 0;
    }

    ~TestTerminal()
    {
        if (slave_fd >= 0) ::close(slave_fd);
        if (master_fd >= 0) ::close(master_fd);
    }
};

TEST(UsbCdcPort, MissingDeviceFailsToOpen)
{
    UsbCdcPort port("/dev/foray_platform_test_device_does_not_exist");
    EXPECT_FALSE(port.open());
    port.close();
}

TEST(UsbCdcPort, OpenCloseAndReopen)
{
    TestTerminal terminal;
    ASSERT_TRUE(terminal.open());
    UsbCdcPort port(terminal.path);
    ASSERT_TRUE(port.open());
    EXPECT_TRUE(port.open());
    port.close();
    EXPECT_TRUE(port.open());
}

TEST(UsbCdcPort, ReadBeforeOpenFails)
{
    UsbCdcPort port("/dev/zero");
    std::uint8_t bytes[4]{};
    EXPECT_EQ(port.read_bytes(bytes, sizeof(bytes)), -1);
}

TEST(UsbCdcPort, ReadRespectsCapacity)
{
    TestTerminal terminal;
    ASSERT_TRUE(terminal.open());
    termios settings{};
    ASSERT_EQ(::tcgetattr(terminal.slave_fd, &settings), 0);
    ::cfmakeraw(&settings);
    ASSERT_EQ(::tcsetattr(terminal.slave_fd, TCSANOW, &settings), 0);
    UsbCdcPort port(terminal.path);
    ASSERT_TRUE(port.open());
    const std::uint8_t source[5] = {1, 2, 3, 4, 5};
    ASSERT_EQ(::write(terminal.master_fd, source, sizeof(source)), 5);
    std::uint8_t bytes[5] = {0, 0, 0, 0, 0xAA};
    EXPECT_EQ(port.read_bytes(bytes, 4), 4);
    EXPECT_EQ(bytes[4], 0xAA);
}

TEST(UsbCdcPort, WriteBeforeOpenReportsBadDescriptor)
{
    UsbCdcPort port("/dev/null");
    const std::uint8_t bytes[2] = {0xD3, 0x91};
    errno = 0;
    EXPECT_EQ(port.write_bytes(bytes, sizeof(bytes)), -1);
    EXPECT_EQ(errno, EBADF);
}

TEST(UsbCdcPort, WriteReturnsBytesAcceptedByDevice)
{
    TestTerminal terminal;
    ASSERT_TRUE(terminal.open());
    UsbCdcPort port(terminal.path);
    ASSERT_TRUE(port.open());
    const std::uint8_t bytes[4] = {1, 2, 3, 4};
    EXPECT_EQ(port.write_bytes(bytes, sizeof(bytes)), 4);
}

TEST(UsbCdcPort, WritesBinaryBytesWithoutTextConversion)
{
    TestTerminal terminal;
    ASSERT_TRUE(terminal.open());
    UsbCdcPort port(terminal.path);
    ASSERT_TRUE(port.open());

    const std::uint8_t source[] = {0xD3, 0x00, 0x91, 0xFF};
    ASSERT_EQ(port.write_bytes(source, sizeof(source)), static_cast<ssize_t>(sizeof(source)));

    std::uint8_t received[sizeof(source)]{};
    ASSERT_EQ(::read(terminal.master_fd, received, sizeof(received)),
              static_cast<ssize_t>(sizeof(received)));
    for (std::size_t i = 0; i < sizeof(source); ++i) {
        EXPECT_EQ(received[i], source[i]);
    }
}

TEST(UsbCdcPort, OpensTerminalDeviceForFutureRawModeTest)
{
    TestTerminal terminal;
    ASSERT_TRUE(terminal.open());
    UsbCdcPort port(terminal.path);
    EXPECT_TRUE(port.open());
}

}  // namespace
}  // namespace foray_platform
