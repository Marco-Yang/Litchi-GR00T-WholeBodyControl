// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// The oracle for the bridge is the receiver itself: a frame sent through
// PackA3RefMessage -> ParseA3RefMessage -> BuildTaWholeBodyCommand -> protobuf wire ->
// ConvertTaWholeBodyCommand (a3_teleop_reference.cpp) must come out bit-identical.
#include "a3_deploy/a3_ta_bridge_codec.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

a3_deploy::A3TeleopFrame MakeFrame(bool head) {
  a3_deploy::A3TeleopFrame f{};
  f.stamp_ns = 1'700'000'000'123'456'789LL;
  for (std::size_t i = 0; i < 29; ++i) {
    f.q_mujoco[i] = 0.1 * static_cast<double>(i) - 1.3;  // every slot distinct
    f.dq_mujoco[i] = 0.01 * static_cast<double>(i) + 5.0;
  }
  const double half = 0.5 * 0.7;
  f.pelvis_quat_wxyz = {std::cos(half) * std::cos(0.2), std::sin(0.2) * std::cos(half), 0.0,
                        std::sin(half)};
  double n = 0.0;
  for (double v : f.pelvis_quat_wxyz) n += v * v;
  for (double& v : f.pelvis_quat_wxyz) v /= std::sqrt(n);
  f.has_head_command = head;
  if (head) {
    f.head_q_rad = {0.31, -0.42};
    f.head_dq_rad_s = {1.5, -2.5};
  }
  return f;
}

}  // namespace

TEST(A3TaBridgeCodec, WireRoundTripAndSize) {
  const auto f = MakeFrame(true);
  const std::string wire = a3_deploy::PackA3RefMessage(f);
  ASSERT_EQ(wire.size(), a3_deploy::kA3RefTopicSize + a3_deploy::kA3RefBodySize);
  a3_deploy::A3TeleopFrame back{};
  std::string err;
  ASSERT_TRUE(a3_deploy::ParseA3RefMessage(wire.data(), wire.size(), back, &err)) << err;
  EXPECT_EQ(back.stamp_ns, f.stamp_ns);
  EXPECT_EQ(back.q_mujoco, f.q_mujoco);
  EXPECT_EQ(back.dq_mujoco, f.dq_mujoco);
  EXPECT_EQ(back.pelvis_quat_wxyz, f.pelvis_quat_wxyz);
  EXPECT_TRUE(back.has_head_command);
  EXPECT_EQ(back.head_q_rad, f.head_q_rad);
  EXPECT_EQ(back.head_dq_rad_s, f.head_dq_rad_s);
}

TEST(A3TaBridgeCodec, RejectsMalformedMessages) {
  const auto f = MakeFrame(false);
  std::string wire = a3_deploy::PackA3RefMessage(f);
  a3_deploy::A3TeleopFrame out{};
  EXPECT_FALSE(a3_deploy::ParseA3RefMessage(wire.data(), wire.size() - 1, out, nullptr));
  std::string bad_topic = wire;
  bad_topic[0] = 'x';
  EXPECT_FALSE(a3_deploy::ParseA3RefMessage(bad_topic.data(), bad_topic.size(), out, nullptr));
  std::string bad_magic = wire;
  bad_magic[a3_deploy::kA3RefTopicSize] ^= 0x7f;
  EXPECT_FALSE(a3_deploy::ParseA3RefMessage(bad_magic.data(), bad_magic.size(), out, nullptr));
  auto nan_frame = f;
  nan_frame.q_mujoco[7] = std::nan("");
  const std::string nan_wire = a3_deploy::PackA3RefMessage(nan_frame);
  EXPECT_FALSE(a3_deploy::ParseA3RefMessage(nan_wire.data(), nan_wire.size(), out, nullptr));
  auto bad_quat = f;
  bad_quat.pelvis_quat_wxyz = {2.0, 0.0, 0.0, 0.0};
  const std::string quat_wire = a3_deploy::PackA3RefMessage(bad_quat);
  EXPECT_FALSE(a3_deploy::ParseA3RefMessage(quat_wire.data(), quat_wire.size(), out, nullptr));
}

TEST(A3TaBridgeCodec, Ros2TopicNameMatchesAimrtEncoding) {
  EXPECT_EQ(a3_deploy::TaRos2TopicName("/ta/whole_body_command"),
            "/ta/whole_body_command/pb_3Aaimdk_2Eprotocol_2ETaWholeBodyCommandChannel");
}

#ifdef HAS_A3_TA_PROTO
namespace {
void ExpectRoundTripThroughReceiver(bool head) {
  const auto f = MakeFrame(head);
  aimdk::protocol::TaWholeBodyCommandChannel msg;
  a3_deploy::BuildTaWholeBodyCommand(f, 7, 1'700'000'000'500'000'000LL, msg);

  // protobuf wire round trip (what actually travels inside RosMsgWrapper.data)
  std::string bytes;
  ASSERT_TRUE(msg.SerializeToString(&bytes));
  aimdk::protocol::TaWholeBodyCommandChannel received;
  ASSERT_TRUE(received.ParseFromString(bytes));

  a3_deploy::A3TeleopFrame out{};
  std::string err;
  ASSERT_TRUE(a3_deploy::ConvertTaWholeBodyCommand(received, 0, out, &err)) << err;
  EXPECT_EQ(out.stamp_ns, 1'700'000'000'500'000'000LL);
  for (std::size_t i = 0; i < 29; ++i) {
    EXPECT_EQ(out.q_mujoco[i], f.q_mujoco[i]) << "q[" << i << "]";
    EXPECT_EQ(out.dq_mujoco[i], f.dq_mujoco[i]) << "dq[" << i << "]";
  }
  EXPECT_EQ(out.pelvis_quat_wxyz, f.pelvis_quat_wxyz);
  EXPECT_EQ(out.has_head_command, head);
  if (head) {
    EXPECT_EQ(out.head_q_rad, f.head_q_rad);
    EXPECT_EQ(out.head_dq_rad_s, f.head_dq_rad_s);
  }
  EXPECT_EQ(received.header().seq(), 7u);
  EXPECT_EQ(received.data().joint_velocities().velocities_rad_s_size(), 31);
}
}  // namespace

TEST(A3TaBridgeCodec, ReceiverConvertsBridgeOutputLosslessly_WithHead) {
  ExpectRoundTripThroughReceiver(true);
}
TEST(A3TaBridgeCodec, ReceiverConvertsBridgeOutputLosslessly_NoHead) {
  ExpectRoundTripThroughReceiver(false);
}
#endif
