#include <array>
#include <cstdint>
#include <pty.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "foray/platform/link/robolink_codec.hpp"
#include "foray/platform/link/robolink_host_link.hpp"

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

void send_slot(int master, const foray::platform::link::RoboLinkSlot& slot)
{
    const auto frame = foray::platform::link::encode_single_slot_frame(10, 1000, slot);
    ASSERT_EQ(::write(master, frame.data(), frame.size()),
              static_cast<ssize_t>(frame.size()));
}

/// @brief 读走主机侧刚写出的 HELLO 帧并发回序号匹配的 ACK，返回它的序号。
std::uint16_t answer_hello(int master, foray::platform::link::HelloAck* ack_out = nullptr) {
    std::array<std::uint8_t, 128> bytes{};
    const ssize_t count = ::read(master, bytes.data(), bytes.size());
    EXPECT_GT(count, 0);
    const auto slot =
        foray::platform::link::decode_single_slot_frame({bytes.begin(), bytes.begin() + count});
    EXPECT_TRUE(slot.has_value());
    if (!slot.has_value())
        return 0;
    EXPECT_EQ(slot->type, 0x01u);
    if (ack_out != nullptr) {
        *ack_out = foray::platform::link::HelloAck{};
        ack_out->acked_msg_seq = slot->sequence;
    }
    return slot->sequence;
}

/// @brief 读走主机侧刚写出的 HELLO 帧，只返回它的序号。
std::uint16_t read_hello_sequence(int master) {
    return answer_hello(master);
}

void send_hello_ack(int master, std::uint16_t acked_seq) {
    foray::platform::link::HelloAck ack;
    ack.acked_msg_seq = acked_seq;
    foray::platform::link::RoboLinkSlot slot;
    slot.type = 0x02;
    slot.flags = 0x0C;
    slot.payload = foray::platform::link::encode_hello_ack_payload(ack);
    send_slot(master, slot);
}

/// @brief 与握手无关的合法帧，用来把链路的「最近收到」时刻往前推。
void send_keep_alive(int master) {
    foray::platform::link::RoboLinkSlot slot;
    slot.type = 0x30;
    slot.payload.resize(36);
    send_slot(master, slot);
}

TEST(RoboLinkHostLink, HandshakeStatusControlAndTimeout)
{
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray::platform::link::UsbCdcPort port(terminal.path);
    foray::platform::link::RoboLinkHostLink link(port);

    EXPECT_EQ(link.step(0), foray::platform::link::HostLinkState::HelloPending);
    std::array<std::uint8_t, 128> hello_bytes{};
    const ssize_t hello_count = ::read(terminal.master, hello_bytes.data(), hello_bytes.size());
    ASSERT_GT(hello_count, 0);
    const auto hello_slot = foray::platform::link::decode_single_slot_frame(
        {hello_bytes.begin(), hello_bytes.begin() + hello_count});
    ASSERT_TRUE(hello_slot.has_value());
    EXPECT_EQ(hello_slot->type, 0x01u);
    EXPECT_EQ(hello_slot->flags, 0x04u);
    const auto hello = foray::platform::link::decode_hello_payload(
        hello_slot->payload.data(), hello_slot->payload.size());
    ASSERT_TRUE(hello.has_value());
    EXPECT_NE(hello->capabilities & (std::uint64_t{1} << 0x20), 0u);
    EXPECT_NE(hello->capabilities & (std::uint64_t{1} << 0x30), 0u);

    foray::platform::link::ControlSnapshot control;
    control.flags = 0x0002;
    EXPECT_FALSE(link.queue_control(control, 0));  // 握手前不能发送控制。

    foray::platform::link::HelloAck ack;
    ack.acked_msg_seq = hello_slot->sequence;
    foray::platform::link::RoboLinkSlot ack_slot;
    ack_slot.type = 0x02;
    ack_slot.flags = 0x0C;
    ack_slot.payload = foray::platform::link::encode_hello_ack_payload(ack);
    send_slot(terminal.master, ack_slot);
    EXPECT_EQ(link.step(1), foray::platform::link::HostLinkState::Ready);
    EXPECT_FALSE(link.queue_control(control, 1));  // 状态尚未回读。

    foray::platform::link::RoboLinkSlot status_slot;
    status_slot.type = 0x10;
    status_slot.payload.resize(28);
    status_slot.payload[22] = 3;
    send_slot(terminal.master, status_slot);
    EXPECT_EQ(link.step(2), foray::platform::link::HostLinkState::Active);
    ASSERT_TRUE(link.latest_status().has_value());

    foray::platform::link::RoboLinkSlot attitude_slot;
    attitude_slot.type = 0x30;
    attitude_slot.payload.resize(36);
    attitude_slot.payload[6] = 0x80;
    attitude_slot.payload[7] = 0x3F;  // q_w = 1.0f。
    attitude_slot.payload[32] = 1;
    attitude_slot.payload[33] = 1;
    send_slot(terminal.master, attitude_slot);
    EXPECT_EQ(link.step(2), foray::platform::link::HostLinkState::Active);
    ASSERT_TRUE(link.latest_attitude().has_value());
    EXPECT_TRUE(link.latest_attitude()->attitude.attitude_valid);
    EXPECT_EQ(link.latest_attitude()->attitude.quaternion_wxyz[0], 1.0f);
    EXPECT_EQ(link.latest_attitude()->frame_timestamp_us, 1000u);
    EXPECT_EQ(link.latest_attitude()->received_at_ms, 2u);

    EXPECT_TRUE(link.queue_control(control, 3));
    EXPECT_EQ(link.step(3), foray::platform::link::HostLinkState::Active);
    std::array<std::uint8_t, 128> control_bytes{};
    const ssize_t control_count = ::read(terminal.master, control_bytes.data(),
                                         control_bytes.size());
    ASSERT_EQ(control_count, 72);
    const auto sent = foray::platform::link::decode_single_slot_frame(
        {control_bytes.begin(), control_bytes.begin() + control_count});
    ASSERT_TRUE(sent.has_value());
    EXPECT_EQ(sent->type, 0x20u);
    EXPECT_EQ(sent->payload[0], 0u);  // 本连接第一条控制序号由链路分配为 0。

    EXPECT_EQ(link.step(503), foray::platform::link::HostLinkState::Disconnected);
    EXPECT_FALSE(port.is_open());
    EXPECT_FALSE(link.latest_status().has_value());
    EXPECT_FALSE(link.latest_attitude().has_value());
    EXPECT_FALSE(link.queue_control(control, 503));
}

TEST(RoboLinkHostLink, InvalidHandshakeDoesNotEnableControl)
{
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray::platform::link::UsbCdcPort port(terminal.path);
    foray::platform::link::RoboLinkHostLink link(port);
    ASSERT_EQ(link.step(0), foray::platform::link::HostLinkState::HelloPending);
    std::array<std::uint8_t, 128> bytes{};
    ASSERT_GT(::read(terminal.master, bytes.data(), bytes.size()), 0);

    foray::platform::link::HelloAck ack;
    ack.acked_msg_seq = 99;  // 不属于本次 HELLO。
    foray::platform::link::RoboLinkSlot slot;
    slot.type = 0x02;
    slot.flags = 0x0C;
    slot.payload = foray::platform::link::encode_hello_ack_payload(ack);
    send_slot(terminal.master, slot);
    EXPECT_EQ(link.step(1), foray::platform::link::HostLinkState::HelloPending);
}

/**
 * @brief HELLO 的消息序号必须逐次自增，且 ACK 要按序号匹配。
 *
 * 审计 H1：旧实现里 `hello_seq_` 只在开局置 0、从不自增，于是每次重发的 HELLO 都带
 * 序号 0，握手判定退化成「收到任何 HELLO_ACK 就算握手成功」——旧用例直接照抄
 * `hello_slot->sequence` 当 ACK 序号，把同一个错误假设复制进了测试。
 */
TEST(RoboLinkHostLink, HelloSequenceIncrementsOnEveryRetry) {
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray::platform::link::UsbCdcPort port(terminal.path);
    foray::platform::link::RoboLinkHostLink link(port);

    ASSERT_EQ(link.step(0), foray::platform::link::HostLinkState::HelloPending);
    EXPECT_EQ(read_hello_sequence(terminal.master), 0u);
    // 200 ms 未收到 ACK，重发下一次 HELLO。
    ASSERT_EQ(link.step(200), foray::platform::link::HostLinkState::HelloPending);
    EXPECT_EQ(read_hello_sequence(terminal.master), 1u);
    ASSERT_EQ(link.step(400), foray::platform::link::HostLinkState::HelloPending);
    EXPECT_EQ(read_hello_sequence(terminal.master), 2u);

    // 慢到的 ACK 应答的是第一次 HELLO（序号 0），仍在窗口内，应当被采纳。
    send_hello_ack(terminal.master, 0);
    EXPECT_EQ(link.step(401), foray::platform::link::HostLinkState::Ready);
}

/// @brief 应答一个从未发出过的序号：不能算握手成功。
TEST(RoboLinkHostLink, HelloAckForUnsentSequenceIsIgnored) {
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray::platform::link::UsbCdcPort port(terminal.path);
    foray::platform::link::RoboLinkHostLink link(port);

    ASSERT_EQ(link.step(0), foray::platform::link::HostLinkState::HelloPending);
    EXPECT_EQ(read_hello_sequence(terminal.master), 0u);
    send_hello_ack(terminal.master, 7);
    EXPECT_EQ(link.step(1), foray::platform::link::HostLinkState::HelloPending);
}

/// @brief 超出匹配窗口的陈旧 ACK 不算握手成功，窗口内的照常采纳。
TEST(RoboLinkHostLink, HelloAckOutsideWindowIsIgnored) {
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray::platform::link::UsbCdcPort port(terminal.path);
    foray::platform::link::RoboLinkHostLink link(port);

    ASSERT_EQ(link.step(0), foray::platform::link::HostLinkState::HelloPending);
    EXPECT_EQ(read_hello_sequence(terminal.master), 0u);

    // 每 200 ms 发一次 keep-alive 保证链路不超时，同时观察 HELLO 序号连号。
    std::uint64_t now = 0;
    for (std::uint16_t expected = 1; expected <= 5; ++expected) {
        send_keep_alive(terminal.master);
        now += 200;
        ASSERT_EQ(link.step(now), foray::platform::link::HostLinkState::HelloPending);
        EXPECT_EQ(read_hello_sequence(terminal.master), expected);
    }
    // 已发过 0..5，窗口是最近 4 个（2..5）：序号 1 太旧，序号 3 还在窗口内。
    send_hello_ack(terminal.master, 1);
    EXPECT_EQ(link.step(now + 1), foray::platform::link::HostLinkState::HelloPending);
    send_hello_ack(terminal.master, 3);
    EXPECT_EQ(link.step(now + 2), foray::platform::link::HostLinkState::Ready);
}

/**
 * @brief 假电控回环跑 100 次完整握手：每次重连都应从本连接的第 0 条 HELLO 重新开始。
 *
 * 审计 H1 的验收口径（100 次握手）；同时覆盖「断链重连后序号复位」的协议约定。
 */
TEST(RoboLinkHostLink, HundredReconnectsAllReachReady) {
    TerminalPair terminal;
    ASSERT_TRUE(terminal.open());
    foray::platform::link::UsbCdcPort port(terminal.path);
    foray::platform::link::RoboLinkHostLink link(port);

    std::uint64_t now = 0;
    for (int cycle = 0; cycle < 100; ++cycle) {
        ASSERT_EQ(link.step(now), foray::platform::link::HostLinkState::HelloPending)
            << "cycle " << cycle;
        EXPECT_EQ(read_hello_sequence(terminal.master), 0u) << "cycle " << cycle;
        send_hello_ack(terminal.master, 0);
        ASSERT_EQ(link.step(now + 1), foray::platform::link::HostLinkState::Ready)
            << "cycle " << cycle;
        // 超过 500 ms 无合法帧 → 断链，下一轮重新打开设备。
        ASSERT_EQ(link.step(now + 502), foray::platform::link::HostLinkState::Disconnected)
            << "cycle " << cycle;
        // 断链后要等 kOpenRetryMs 才允许重新打开设备，这里跨过整个等待窗口。
        now += 800;
    }
}

}  // 匿名命名空间
