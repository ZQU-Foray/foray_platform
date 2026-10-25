#include <array>
#include <cstdint>
#include <pty.h>
#include <unistd.h>
#include <gtest/gtest.h>

#include "foray_platform/robolink_byte_channel.hpp"
#include "foray_platform/robolink_frame.hpp"

namespace {

struct TerminalPair {
    int master = -1;
    int slave = -1;
    char path[128]{};

    bool open()
    {
        return ::openpty(&master, &slave, nullptr, nullptr, nullptr) == 0 &&
               ::ttyname_r(slave, path, sizeof(path)) == 0;
    }

    ~TerminalPair()
    {
        if (master >= 0) ::close(master);
        if (slave >= 0) ::close(slave);
    }
};

TEST(RoboLinkByteChannel, SendsFrameThroughDeviceBytes)
{
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray_platform::UsbCdcPort port(terminal.path);
    ASSERT_TRUE(port.open());
    foray_platform::RoboLinkByteChannel channel(port);
    const auto frame = foray_platform::make_keepalive_frame(1, 100);
    const std::vector<std::uint8_t> bytes(frame.begin(), frame.end());

    ASSERT_TRUE(channel.queue_frame(bytes));
    EXPECT_FALSE(channel.queue_frame(bytes));  // 旧帧尚未发完，不能覆盖。
    EXPECT_EQ(channel.flush_once(), static_cast<ssize_t>(bytes.size()));
    EXPECT_FALSE(channel.has_pending_write());

    std::array<std::uint8_t, 18> received{};
    ASSERT_EQ(::read(terminal.master, received.data(), received.size()),
              static_cast<ssize_t>(received.size()));
    EXPECT_EQ(received, frame);
}

TEST(RoboLinkByteChannel, ReceivesFrameAcrossTwoReads)
{
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray_platform::UsbCdcPort port(terminal.path);
    ASSERT_TRUE(port.open());
    foray_platform::RoboLinkByteChannel channel(port);
    const auto frame = foray_platform::make_keepalive_frame(2, 200);

    ASSERT_EQ(::write(terminal.master, frame.data(), 5), 5);
    EXPECT_TRUE(channel.receive_once().empty());
    ASSERT_EQ(::write(terminal.master, frame.data() + 5, frame.size() - 5),
              static_cast<ssize_t>(frame.size() - 5));
    const auto received = channel.receive_once();
    ASSERT_EQ(received.size(), 1u);
    EXPECT_EQ(received[0], (std::vector<std::uint8_t>{frame.begin(), frame.end()}));
}

TEST(RoboLinkByteChannel, RefusesCorruptedOutgoingFrame)
{
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray_platform::UsbCdcPort port(terminal.path);
    ASSERT_TRUE(port.open());
    foray_platform::RoboLinkByteChannel channel(port);
    const auto frame = foray_platform::make_keepalive_frame(3, 300);
    std::vector<std::uint8_t> bad(frame.begin(), frame.end());
    bad[8] ^= 1;
    EXPECT_FALSE(channel.queue_frame(bad));
}

}  // namespace
