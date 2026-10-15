// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Codec for the A3 real-time teleop bridge (`a3_ta_bridge`).
//
//   Pico -> a3_teleop_retarget_node.py --(ZMQ "a3_ref")--> a3_ta_bridge
//        --(ROS2 RosMsgWrapper, pb)--> /ta/whole_body_command --> a3_deploy_onnx_ref
//
// The wire layout is the one defined by gear_sonic/utils/teleop/a3/frames.py
// (little-endian, packed):  "a3_ref" topic + <IIq 29d 29d 4d B 2d 2d> (545 bytes).
// Joint order on the wire is the policy/"mj29" order: waist(3) + left arm(7) +
// right arm(7) + left leg(6) + right leg(6). This is the inverse of
// ConvertTaWholeBodyCommand() (a3_teleop_reference.cpp), which the unit test uses
// as the oracle.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "a3_deploy/a3_teleop_reference.hpp"

#ifdef HAS_A3_TA_PROTO
#include "aimdk/protocol/ta/ta_channel.pb.h"
#endif

namespace a3_deploy {

inline constexpr char kA3RefTopic[] = "a3_ref";
inline constexpr std::size_t kA3RefTopicSize = 6;
inline constexpr std::uint32_t kA3RefMagic = 0x46335441u;
inline constexpr std::uint32_t kA3RefVersion = 1u;
inline constexpr std::size_t kA3RefBodySize = 545;

// Parses one ZMQ message (topic prefix + body). Rejects wrong topic/size/magic/
// version and non-finite values. On success `out.stamp_ns` is the sender stamp.
bool ParseA3RefMessage(const void* data, std::size_t size, A3TeleopFrame& out,
                       std::string* error);

// Serialises a frame exactly like frames.pack_frame() (used by the unit test).
std::string PackA3RefMessage(const A3TeleopFrame& frame);

#ifdef HAS_A3_TA_PROTO
// Fills a TaWholeBodyCommandChannel (BODY_31 layout, velocities leg12+waist3+head2+arm14).
void BuildTaWholeBodyCommand(const A3TeleopFrame& frame, std::uint32_t seq,
                             std::int64_t stamp_ns,
                             aimdk::protocol::TaWholeBodyCommandChannel& msg);
#endif

// ROS2 topic AimRT's ros2 backend uses for a protobuf channel:
//   <topic>/<Ros2NameEncode("pb:aimdk.protocol.TaWholeBodyCommandChannel")>
std::string Ros2EncodeName(const std::string& s);
std::string TaRos2TopicName(const std::string& channel_topic);

}  // namespace a3_deploy
