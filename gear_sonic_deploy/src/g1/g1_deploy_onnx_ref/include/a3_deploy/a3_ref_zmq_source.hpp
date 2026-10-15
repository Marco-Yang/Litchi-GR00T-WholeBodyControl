// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// ZMQ teleop-reference input of the A3 deploy: the operator side's `a3_ref` stream (a3_teleop_retarget_node.py, see
// a3_ta_bridge_codec.hpp for the wire layout) read directly by the deploy process, with no ROS 2 / AimRT /ta hop.
//
//   Pico -> a3_teleop_retarget_node.py --(ZMQ "a3_ref", tcp)--> A3RefZmqSource --> A3TeleopReferenceBuffer
//
// This is the alternative to the default `/ta/whole_body_command` subscription (`--teleop-input zmq`): nothing is published on the
// robot's ROS 2 network, so no other subscriber of /ta/whole_body_command can ever receive the frames.  Frames are re-stamped with
// the local system clock on arrival (like the AimRT path), so the sender's clock does not matter.
#pragma once

#include "a3_deploy/a3_teleop_reference.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace a3_deploy {

struct A3RefZmqSourceOptions {
  std::string host{"localhost"};
  int port = 5560;
  int rcv_timeout_ms = 100;  // poll interval of the receive thread (also the stop latency)
  int rcv_hwm = 200;         // frames queued in ZMQ before it drops
};

class A3RefZmqSource {
 public:
  using FrameCallback = std::function<void(const A3TeleopFrame&)>;

  A3RefZmqSource(A3RefZmqSourceOptions options, FrameCallback callback);
  ~A3RefZmqSource();
  A3RefZmqSource(const A3RefZmqSource&) = delete;
  A3RefZmqSource& operator=(const A3RefZmqSource&) = delete;

  // Connects and starts the receive thread. False (with a message on stderr) if the endpoint is invalid.
  bool Start();
  void Stop();

  std::string Endpoint() const;
  std::uint64_t Received() const noexcept { return received_.load(std::memory_order_relaxed); }
  std::uint64_t Rejected() const noexcept { return rejected_.load(std::memory_order_relaxed); }

 private:
  void Loop_();

  A3RefZmqSourceOptions options_;
  FrameCallback callback_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> received_{0};
  std::atomic<std::uint64_t> rejected_{0};
  std::thread thread_;
};

}  // namespace a3_deploy
