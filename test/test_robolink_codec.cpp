#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <gtest/gtest.h>

#include "foray_platform/robolink_codec.hpp"
#include "foray_platform/robolink_frame_parser.hpp"

TEST(RoboLinkCodec, ControlSnapshotUsesDocumentedOffsets)
{
    foray_platform::ControlSnapshot control;
    control.control_seq = 0x12345678;
    control.flags = 0x0002;  // 仅云台使能。
    control.gimbal_yaw_rad = 1.5f;
    control.gimbal_pitch_rad = -0.25f;
    control.validity_hint_ms = 80;  // 按候选规范钳制为 50 ms。

    const auto payload = foray_platform::encode_control_snapshot_payload(control);
    ASSERT_EQ(payload.size(), 48u);
    EXPECT_EQ((std::vector<std::uint8_t>{payload.begin(), payload.begin() + 8}),
              (std::vector<std::uint8_t>{0x78, 0x56, 0x34, 0x12, 0x02, 0x02, 0x02, 0x00}));
    EXPECT_EQ((std::vector<std::uint8_t>{payload.begin() + 20, payload.begin() + 24}),
              (std::vector<std::uint8_t>{0x00, 0x00, 0xC0, 0x3F}));  // 1.5f rad 的小端表示。
    EXPECT_EQ(payload[44], 50u);
    EXPECT_EQ(payload[45], 0u);
    EXPECT_EQ(payload[46], 0u);
    EXPECT_EQ(payload[47], 0u);
}

TEST(RoboLinkCodec, SingleSlotFrameRoundTripsThroughStreamParser)
{
    foray_platform::ControlSnapshot control;
    control.flags = 0x0002;
    control.gimbal_yaw_rad = 0.5f;
    foray_platform::RoboLinkSlot slot;
    slot.type = 0x20;
    slot.flags = 0x02;  // LATEST_ONLY。
    slot.sequence = 7;
    slot.payload = foray_platform::encode_control_snapshot_payload(control);

    const auto bytes = foray_platform::encode_single_slot_frame(4, 123456, slot);
    ASSERT_EQ(bytes.size(), 72u);
    foray_platform::RoboLinkFrameParser parser;
    EXPECT_TRUE(parser.feed(bytes.data(), 9).empty());
    const auto frames = parser.feed(bytes.data() + 9, bytes.size() - 9);
    ASSERT_EQ(frames.size(), 1u);
    const auto decoded = foray_platform::decode_single_slot_frame(frames.front());
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->type, 0x20);
    EXPECT_EQ(decoded->sequence, 7);
    EXPECT_EQ(decoded->payload, slot.payload);
}

TEST(RoboLinkCodec, MultipleSlotsAndKeepaliveRoundTrip)
{
    foray_platform::RoboLinkSlot heartbeat;
    heartbeat.type = 0x03;
    heartbeat.sequence = 11;
    foray_platform::RoboLinkSlot ack;
    ack.type = 0x04;
    ack.flags = 0x08;
    ack.sequence = 12;
    ack.payload = foray_platform::encode_ack_payload({0x01, 7, 0, 0});
    const auto frame = foray_platform::encode_frame(8, 100, {heartbeat, ack});
    ASSERT_EQ(frame[3], 0x01u);  // 两槽位中包含 ACK。
    const auto decoded = foray_platform::decode_frame(frame);
    ASSERT_TRUE(decoded.has_value());
    ASSERT_EQ(decoded->size(), 2u);
    EXPECT_EQ((*decoded)[0].sequence, 11u);
    EXPECT_EQ((*decoded)[1].payload, ack.payload);
    EXPECT_FALSE(foray_platform::decode_single_slot_frame(frame).has_value());

    const auto keepalive = foray_platform::encode_frame(8, 100, {});
    const auto decoded_keepalive = foray_platform::decode_frame(keepalive);
    ASSERT_TRUE(decoded_keepalive.has_value());
    EXPECT_TRUE(decoded_keepalive->empty());
}

TEST(RoboLinkCodec, VisualControlConvenienceBuildsSeventyTwoByteFrame)
{
    foray_platform::ControlSnapshot control;
    control.control_seq = 9;
    control.flags = 0x0002;
    control.gimbal_yaw_rad = 0.75f;
    const auto frame = foray_platform::make_control_snapshot_frame(1, 1000, 2, control);
    ASSERT_EQ(frame.size(), 72u);
    const auto slot = foray_platform::decode_single_slot_frame(frame);
    ASSERT_TRUE(slot.has_value());
    EXPECT_EQ(slot->type, 0x20);
    EXPECT_EQ(slot->flags, 0x02);
    EXPECT_EQ(slot->payload[0], 9u);
}

TEST(RoboLinkCodec, RejectsCorruptedSlotFrame)
{
    foray_platform::RoboLinkSlot slot;
    slot.type = 0x03;
    const auto valid = foray_platform::encode_single_slot_frame(1, 1, slot);
    auto bad = valid;
    bad[16] ^= 1;
    EXPECT_FALSE(foray_platform::decode_single_slot_frame(bad).has_value());
}

TEST(RoboLinkCodec, RejectsInvalidControlValues)
{
    foray_platform::ControlSnapshot control;
    control.flags = 0x8000;
    EXPECT_THROW(foray_platform::encode_control_snapshot_payload(control),
                 std::invalid_argument);
}

TEST(RoboLinkCodec, DecodesCoreSystemStatus)
{
    std::array<std::uint8_t, 28> payload{};
    payload[0] = 0xE8;  // uptime_ms = 1000。
    payload[1] = 0x03;
    payload[4] = 2;     // VISION_TRACK。
    payload[5] = 2;     // PC。
    payload[8] = 9;     // 最近收到 9。
    payload[12] = 8;    // 最近接受 8。
    payload[16] = 7;    // 最近执行 7。
    payload[20] = 12;   // 控制消息年龄 12 ms。
    payload[22] = 3;    // ACTIVE。

    const auto status = foray_platform::decode_system_status_payload(
        payload.data(), payload.size());
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->uptime_ms, 1000u);
    EXPECT_EQ(status->last_rx_control_seq, 9u);
    EXPECT_EQ(status->last_accepted_control_seq, 8u);
    EXPECT_EQ(status->last_applied_control_seq, 7u);
    EXPECT_EQ(status->control_msg_age_ms, 12u);
    EXPECT_FALSE(status->diagnostics.has_value());

    payload[6] = 99;  // v0.2 §4.6：未知安全状态按控制失联处理。
    const auto unknown_safety = foray_platform::decode_system_status_payload(
        payload.data(), payload.size());
    ASSERT_TRUE(unknown_safety.has_value());
    EXPECT_EQ(unknown_safety->safety_state, 2u);
}

TEST(RoboLinkCodec, DecodesGimbalAttitudeInWxyzOrder)
{
    // RoboLink v0.2 §4.8：采样时刻 0x12345678，单位四元数 (0.5,0.5,-0.5,0.5)。
    std::array<std::uint8_t, 36> payload{};
    payload[0] = 0x78;
    payload[1] = 0x56;
    payload[2] = 0x34;
    payload[3] = 0x12;
    payload[7] = 0x3F;   // q_w = 0.5f。
    payload[11] = 0x3F;  // q_x = 0.5f。
    payload[15] = 0xBF;  // q_y = -0.5f；能检出 y/z 顺序颠倒。
    payload[19] = 0x3F;  // q_z = 0.5f。
    payload[32] = 1;   // IMU READY。
    payload[33] = 1;   // ATTITUDE_VALID。

    foray_platform::RoboLinkSlot slot;
    slot.type = 0x30;
    slot.payload.assign(payload.begin(), payload.end());
    const auto frame = foray_platform::encode_single_slot_frame(1, 100, slot);
    EXPECT_EQ(frame.size(), 60u);  // 14 B 帧头 + 6 B 槽位头 + 36 B 载荷 + 4 B CRC。
    const auto decoded = foray_platform::decode_single_slot_frame(frame);
    ASSERT_TRUE(decoded.has_value());
    const auto attitude = foray_platform::decode_gimbal_attitude_payload(
        decoded->payload.data(), decoded->payload.size());
    ASSERT_TRUE(attitude.has_value());
    EXPECT_EQ(attitude->sample_time_us, 0x12345678u);
    EXPECT_EQ(attitude->quaternion_wxyz[0], 0.5f);
    EXPECT_EQ(attitude->quaternion_wxyz[1], 0.5f);
    EXPECT_EQ(attitude->quaternion_wxyz[2], -0.5f);
    EXPECT_EQ(attitude->quaternion_wxyz[3], 0.5f);
    EXPECT_EQ(attitude->imu_state, 1u);
    EXPECT_TRUE(attitude->attitude_valid);

    payload[33] = 0;
    const auto unavailable = foray_platform::decode_gimbal_attitude_payload(
        payload.data(), payload.size());
    ASSERT_TRUE(unavailable.has_value());
    EXPECT_FALSE(unavailable->attitude_valid);
    EXPECT_FALSE(foray_platform::decode_gimbal_attitude_payload(
        payload.data(), payload.size() - 1).has_value());
}

TEST(RoboLinkCodec, DecodesSystemStatusFromWholeFrame)
{
    foray_platform::RoboLinkSlot slot;
    slot.type = 0x10;
    slot.payload.resize(28);
    slot.payload[8] = 17;
    slot.payload[22] = 3;
    const auto frame = foray_platform::encode_single_slot_frame(1, 100, slot);
    const auto status = foray_platform::decode_system_status_frame(frame);
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->last_rx_control_seq, 17u);
}

TEST(RoboLinkCodec, RejectsTruncatedSystemStatus)
{
    std::array<std::uint8_t, 27> payload{};
    EXPECT_FALSE(foray_platform::decode_system_status_payload(
        payload.data(), payload.size()).has_value());
}

TEST(RoboLinkCodec, DecodesStatusDiagnosticTail)
{
    std::array<std::uint8_t, 52> payload{};
    payload[22] = 2;
    payload[28] = 25;  // 接收帧数。
    payload[36] = 3;   // CRC 错误数。
    payload[46] = 4;   // 事件队列深度。

    const auto status = foray_platform::decode_system_status_payload(
        payload.data(), payload.size());
    ASSERT_TRUE(status.has_value());
    ASSERT_TRUE(status->diagnostics.has_value());
    EXPECT_EQ(status->diagnostics->rx_frames, 25u);
    EXPECT_EQ(status->diagnostics->crc_errors, 3u);
    EXPECT_EQ(status->diagnostics->event_queue_depth, 4u);
}

TEST(RoboLinkCodec, HelloPayloadRoundTrips)
{
    foray_platform::Hello hello;
    hello.git_short = 0x12345678;
    hello.capabilities = 0x000000010010001Full;
    const auto payload = foray_platform::encode_hello_payload(hello);
    ASSERT_EQ(payload.size(), 26u);
    const auto decoded = foray_platform::decode_hello_payload(payload.data(), payload.size());
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->git_short, hello.git_short);
    EXPECT_EQ(decoded->capabilities, hello.capabilities);
    EXPECT_EQ(decoded->max_frame_bytes, 512u);
}

TEST(RoboLinkCodec, HelloAckPayloadRoundTrips)
{
    foray_platform::HelloAck ack;
    ack.acked_msg_seq = 42;
    const auto payload = foray_platform::encode_hello_ack_payload(ack);
    ASSERT_EQ(payload.size(), 8u);
    const auto decoded = foray_platform::decode_hello_ack_payload(
        payload.data(), payload.size());
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->acked_msg_seq, 42u);
}

TEST(RoboLinkCodec, AckPayloadRoundTrips)
{
    foray_platform::Ack ack;
    ack.acked_msg_type = 0x01;
    ack.acked_msg_seq = 7;
    ack.ack_state = 2;
    const auto payload = foray_platform::encode_ack_payload(ack);
    ASSERT_EQ(payload.size(), 7u);
    const auto decoded = foray_platform::decode_ack_payload(payload.data(), payload.size());
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->acked_msg_seq, 7u);
    EXPECT_EQ(decoded->ack_state, 2u);
}

TEST(RoboLinkCodec, ErrorPayloadRoundTrips)
{
    foray_platform::ErrorMessage error;
    error.error_code = 5;
    error.detail = {'x', 'y'};
    const auto payload = foray_platform::encode_error_payload(error);
    const auto decoded = foray_platform::decode_error_payload(
        payload.data(), payload.size());
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->error_code, 5u);
    EXPECT_EQ(decoded->detail, error.detail);
}

TEST(RoboLinkCodec, RejectsUnknownMessageType)
{
    foray_platform::RoboLinkSlot slot;
    slot.type = 0x7F;
    EXPECT_THROW(foray_platform::encode_single_slot_frame(0, 0, slot),
                 std::invalid_argument);
}
