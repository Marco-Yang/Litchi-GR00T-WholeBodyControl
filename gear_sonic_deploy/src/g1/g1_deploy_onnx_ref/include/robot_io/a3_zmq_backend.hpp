// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// A3ZmqBackend: RobotIOBackend that talks to the robot through the ZMQ relay (a3_zmq_relay on the MDU) instead of AimRT.  This is
// what lets the deploy process run on the operator PC with no ROS 2 / DDS and no vendor config change: the relay owns the robot-side
// transport and the safety failsafe, this class only turns its state frames into RobotState and its RobotCommand into command frames.
//
//   config (comma-separated key=value, like A3AimrtBackend):
//     relay_host=10.42.10.12          (required)
//     state_port=5570  command_port=5571
//     max_frame_age_ms=50             a state frame older than this at pull time is "no state" (the driver's watchdog then halts)
//     connect_timeout_s=5             how long Start() waits for the first valid state frame (it proceeds with a warning)
//     state_hz=100                    reported by StateRateHz() (the relay's real rate is --state-hz there)
//     publish_enabled=true            false (dry-run / probe): SendCommand sends nothing
//
// Pull mode only (UsesLatestStatePull() is true; the state callback is never invoked), i.e. the deploy must run
// `backend.sync_mode: latest_cache`.  The robot clock never leaves the robot: every timestamp handed to the deploy is on THIS
// machine's system clock (frame arrival time minus the ages the relay measured on its own clock).
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "a3_relay/a3_relay_wire.hpp"
#include "robot_io/robot_io_backend.hpp"

namespace zmq {
class context_t;
class socket_t;
}  // namespace zmq

namespace robot_io {

class A3ZmqBackend : public RobotIOBackend {
 public:
  A3ZmqBackend();
  ~A3ZmqBackend() override;

  A3ZmqBackend(const A3ZmqBackend&) = delete;
  A3ZmqBackend& operator=(const A3ZmqBackend&) = delete;

  bool Init(const std::string& config) override;
  bool Start() override;
  void Stop() override;

  const JointLayout& GetLayout() const override;
  void RegisterStateCallback(StateCallback cb) override { user_cb_ = std::move(cb); }
  bool UsesLatestStatePull() const noexcept override { return true; }
  bool TryReadLatestState(RobotState& state) override;
  bool SendCommand(const RobotCommand& cmd) override;
  std::string Name() const override { return "a3_zmq"; }
  void NotifyEnable() override;
  bool LinkFault(std::string* reason) override;
  double StateRateHz() const override { return state_hz_; }

  struct Stats {
    std::uint64_t frames = 0, bad_frames = 0, seq_gaps = 0, commands_sent = 0, send_failures = 0;
    a3_relay::Mode relay_mode = a3_relay::Mode::kDisarmed;
    a3_relay::Reject relay_last_reject = a3_relay::Reject::kNone;
    std::uint32_t relay_accepted = 0, relay_rejected = 0;
    bool relay_forwarding = false;
  };
  Stats GetStats() const;
  std::uint64_t session_id() const { return session_.load(std::memory_order_acquire); }

 private:
  struct Rx {
    a3_relay::StateFrame frame;
    std::int64_t recv_sys_ns = 0;
    bool have = false;
  };
  bool ParseConfig_(const std::string& config);
  void RxLoop_();

  std::string host_;
  int state_port_ = 5570;
  int command_port_ = 5571;
  double max_frame_age_ms_ = 50.0;
  double connect_timeout_s_ = 5.0;
  double state_hz_ = 100.0;
  bool publish_enabled_ = true;

  std::unique_ptr<zmq::context_t> ctx_;
  std::unique_ptr<zmq::socket_t> state_sub_;
  std::unique_ptr<zmq::socket_t> cmd_pub_;
  std::mutex cmd_mtx_;
  std::thread rx_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> inited_{false};

  mutable std::mutex rx_mtx_;
  Rx rx_;
  Stats stats_;
  std::uint64_t last_relay_id_ = 0;
  std::uint32_t last_seq_ = 0;
  std::int64_t tick_ = 0;

  std::atomic<std::uint64_t> session_{0};
  std::uint32_t cmd_seq_ = 0;
  // arming bookkeeping for LinkFault(): a new session gets a grace period to be accepted by the relay
  std::atomic<std::int64_t> enable_ns_{0};
  bool armed_seen_ = false;  // under rx_mtx_: the relay reported ACTIVE with OUR session since the last NotifyEnable
  std::atomic<std::int64_t> last_log_ns_{0};
  std::atomic<std::uint64_t> commands_sent_{0};
  std::atomic<std::uint64_t> send_failures_{0};
  StateCallback user_cb_{};
};

}  // namespace robot_io
