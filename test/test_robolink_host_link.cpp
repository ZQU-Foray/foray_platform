#include <array>
#include <cstdint>
#include <pty.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "foray_platform/robolink_codec.hpp"
#include "foray_platform/robolink_host_link.hpp"

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

void send_slot(int master, const foray_platform::RoboLinkSlot& slot)
{
    const auto frame = foray_platform::encode_single_slot_frame(10, 1000, slot);
    ASSERT_EQ(::write(master, frame.data(), frame.size()),
              static_cast<ssize_t>(frame.size()));
}

TEST(RoboLinkHostLink, HandshakeStatusControlAndTimeout)
{
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray_platform::UsbCdcPort port(terminal.path);
    foray_platform::RoboLinkHostLink link(port);

    EXPECT_EQ(link.step(0), foray_platform::HostLinkState::HelloPending);
    std::array<std::uint8_t, 128> hello_bytes{};
    const ssize_t hello_count = ::read(terminal.master, hello_bytes.data(), hello_bytes.size());
    ASSERT_GT(hello_count, 0);
    const auto hello_slot = foray_platform::decode_single_slot_frame(
        {hello_bytes.begin(), hello_bytes.begin() + hello_count});
    ASSERT_TRUE(hello_slot.has_value());
    EXPECT_EQ(hello_slot->type, 0x01u);
    EXPECT_EQ(hello_slot->flags, 0x04u);
    const auto hello = foray_platform::decode_hello_payload(
        hello_slot->payload.data(), hello_slot->payload.size());
    ASSERT_TRUE(hello.has_value());
    EXPECT_NE(hello->capabilities & (std::uint64_t{1} << 0x20), 0u);
    EXPECT_NE(hello->capabilities & (std::uint64_t{1} << 0x30), 0u);

    foray_platform::ControlSnapshot control;
    control.flags = 0x0002;
    EXPECT_FALSE(link.queue_control(control, 0));  // 握手前不能发送控制。

    foray_platform::HelloAck ack;
    ack.acked_msg_seq = hello_slot->sequence;
    foray_platform::RoboLinkSlot ack_slot;
    ack_slot.type = 0x02;
    ack_slot.flags = 0x0C;
    ack_slot.payload = foray_platform::encode_hello_ack_payload(ack);
    send_slot(terminal.master, ack_slot);
    EXPECT_EQ(link.step(1), foray_platform::HostLinkState::Ready);
    EXPECT_FALSE(link.queue_control(control, 1));  // 状态尚未回读。

    foray_platform::RoboLinkSlot status_slot;
    status_slot.type = 0x10;
    status_slot.payload.resize(28);
    status_slot.payload[22] = 3;
    send_slot(terminal.master, status_slot);
    EXPECT_EQ(link.step(2), foray_platform::HostLinkState::Active);
    ASSERT_TRUE(link.latest_status().has_value());

    foray_platform::RoboLinkSlot attitude_slot;
    attitude_slot.type = 0x30;
    attitude_slot.payload.resize(36);
    attitude_slot.payload[6] = 0x80;
    attitude_slot.payload[7] = 0x3F;  // q_w = 1.0f。
    attitude_slot.payload[32] = 1;
    attitude_slot.payload[33] = 1;
    send_slot(terminal.master, attitude_slot);
    EXPECT_EQ(link.step(2), foray_platform::HostLinkState::Active);
    ASSERT_TRUE(link.latest_attitude().has_value());
    EXPECT_TRUE(link.latest_attitude()->attitude.attitude_valid);
    EXPECT_EQ(link.latest_attitude()->attitude.quaternion_wxyz[0], 1.0f);
    EXPECT_EQ(link.latest_attitude()->frame_timestamp_us, 1000u);
    EXPECT_EQ(link.latest_attitude()->received_at_ms, 2u);

    EXPECT_TRUE(link.queue_control(control, 3));
    EXPECT_EQ(link.step(3), foray_platform::HostLinkState::Active);
    std::array<std::uint8_t, 128> control_bytes{};
    const ssize_t control_count = ::read(terminal.master, control_bytes.data(),
                                         control_bytes.size());
    ASSERT_EQ(control_count, 72);
    const auto sent = foray_platform::decode_single_slot_frame(
        {control_bytes.begin(), control_bytes.begin() + control_count});
    ASSERT_TRUE(sent.has_value());
    EXPECT_EQ(sent->type, 0x20u);
    EXPECT_EQ(sent->payload[0], 0u);  // 本连接第一条控制序号由链路分配为 0。

    EXPECT_EQ(link.step(503), foray_platform::HostLinkState::Disconnected);
    EXPECT_FALSE(port.is_open());
    EXPECT_FALSE(link.latest_status().has_value());
    EXPECT_FALSE(link.latest_attitude().has_value());
    EXPECT_FALSE(link.queue_control(control, 503));
}

TEST(RoboLinkHostLink, InvalidHandshakeDoesNotEnableControl)
{
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray_platform::UsbCdcPort port(terminal.path);
    foray_platform::RoboLinkHostLink link(port);
    ASSERT_EQ(link.step(0), foray_platform::HostLinkState::HelloPending);
    std::array<std::uint8_t, 128> bytes{};
    ASSERT_GT(::read(terminal.master, bytes.data(), bytes.size()), 0);

    foray_platform::HelloAck ack;
    ack.acked_msg_seq = 99;  // 不属于本次 HELLO。
    foray_platform::RoboLinkSlot slot;
    slot.type = 0x02;
    slot.flags = 0x0C;
    slot.payload = foray_platform::encode_hello_ack_payload(ack);
    send_slot(terminal.master, slot);
    EXPECT_EQ(link.step(1), foray_platform::HostLinkState::HelloPending);
}

}  // 匿名命名空间
