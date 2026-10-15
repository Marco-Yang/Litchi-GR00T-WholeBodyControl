// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Subscribe to pico_manager's ZMQ "command" topic (same wire format as G1 zmq_manager) and map the G1 operator flow onto the A3
// manual state machine (a3_pico_command_logic.hpp):
//   A+B+X+Y  disabled -> PD stand (hardens, default standing pose)   enabled -> passive (disabled)
//   A+X      PD stand -> teleop follow
//   A+X      teleop follow -> policy stand (keep a3_fast, default-stand reference; G1 POSE -> PLANNER)
//   A+X      policy stand -> teleop follow again
#pragma once

#include "a3_deploy/a3_manual_control.hpp"
#include "a3_deploy/a3_pico_command_logic.hpp"
#include "input_interface/zmq_packed_message_subscriber.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace a3_deploy {

class A3PicoZmqCommandWatcher {
 public:
  A3PicoZmqCommandWatcher(ManualControlState& control,
                          std::atomic<bool>& stop_requested,
                          std::string host,
                          int port,
                          std::string topic = "command")
      : control_(control),
        stop_requested_(stop_requested),
        host_(std::move(host)),
        port_(port),
        topic_(std::move(topic)) {}

  ~A3PicoZmqCommandWatcher() { Stop(); }

  bool Start() {
    if (running_.exchange(true)) return true;

    // ctor: host, port, topic, timeout_ms, verbose, conflate
    sub_ = std::make_unique<ZMQPackedMessageSubscriber>(
        host_, port_, topic_, /*timeout_ms=*/200, /*verbose=*/false,
        /*conflate=*/true);
    sub_->SetOnDecodedMessage(
        [this](const std::string& /*topic*/,
               const ZMQPackedMessageSubscriber::DecodedHeader& hdr,
               const std::vector<ZMQPackedMessageSubscriber::BufferView>& bufs) {
          OnCommand(hdr, bufs);
        });
    try {
      sub_->Start();
    } catch (const std::exception& e) {
      running_.store(false);
      sub_.reset();
      std::cerr << "[pico-cmd] failed to subscribe to tcp://" << host_ << ":"
                << port_ << " topic '" << topic_ << "': " << e.what() << "\n";
      return false;
    }

    helper_ = std::thread([this] { HelperLoop(); });
    std::cout << "[pico-cmd] listening tcp://" << host_ << ":" << port_
              << " topic '" << topic_
              << "' (A+B+X+Y: harden to PD stand / disable, A+X: follow <-> policy stand; same flow as G1)\n";
    return true;
  }

  void Stop() {
    if (!running_.exchange(false)) return;
    pending_teleop_.store(false, std::memory_order_release);
    if (sub_) {
      sub_->Stop();
      sub_.reset();
    }
    if (helper_.joinable()) helper_.join();
  }

 private:
  static bool DecodeBoolField(
      const ZMQPackedMessageSubscriber::DecodedHeader& hdr,
      const std::vector<ZMQPackedMessageSubscriber::BufferView>& bufs,
      const char* name,
      bool* out) {
    for (size_t i = 0; i < hdr.fields.size() && i < bufs.size(); ++i) {
      if (hdr.fields[i].name != name) continue;
      const auto& field = hdr.fields[i];
      const auto& buf = bufs[i];
      if (field.dtype == "bool" || field.dtype == "u8") {
        if (buf.size < 1) return false;
        uint8_t val = 0;
        std::memcpy(&val, buf.data, 1);
        *out = val != 0;
        return true;
      }
      if (field.dtype == "i32" && buf.size >= sizeof(int32_t)) {
        int32_t val = 0;
        std::memcpy(&val, buf.data, sizeof(int32_t));
        if (hdr.NeedsByteSwap()) val = byte_swap(val);
        *out = val != 0;
        return true;
      }
      return false;
    }
    return false;
  }

  void OnCommand(
      const ZMQPackedMessageSubscriber::DecodedHeader& hdr,
      const std::vector<ZMQPackedMessageSubscriber::BufferView>& bufs) {
    PicoCommand cmd;
    if (!DecodeBoolField(hdr, bufs, "start", &cmd.start) || !DecodeBoolField(hdr, bufs, "stop", &cmd.stop)) {
      return;
    }
    if (!DecodeBoolField(hdr, bufs, "planner", &cmd.planner)) cmd.planner = true;  // an older sender without the bit: "stand"

    std::lock_guard<std::mutex> lock(mu_);
    const DeployMode mode = LoadDeployMode(control_);
    const bool policy_stand =
        control_.teleop_policy_stand.load(std::memory_order_acquire);
    switch (DecidePicoAction(mode,
                             control_.pd_stand_ready.load(std::memory_order_acquire),
                             policy_stand, cmd)) {
      case PicoAction::kNone:
        return;
      case PicoAction::kEnterPassive:
        pending_teleop_.store(false, std::memory_order_release);
        control_.teleop_policy_stand.store(false, std::memory_order_release);
        RequestDeployMode(control_, DeployMode::kPassive, std::cout);
        std::cout << "[pico-cmd] A+B+X+Y -> passive: robot disabled (kp = kd = 0)\n";
        return;
      case PicoAction::kEnterPdStand:
        pending_teleop_.store(false, std::memory_order_release);
        control_.teleop_policy_stand.store(false, std::memory_order_release);
        RequestDeployMode(control_, DeployMode::kPdStand, std::cout);
        std::cout << "[pico-cmd] A+B+X+Y -> PD stand: robot hardens and holds the default standing pose (Init Done when the ramp is "
                     "finished); then A+X to follow, A+B+X+Y to disable\n";
        return;
      case PicoAction::kEnterPolicyStand:
        pending_teleop_.store(false, std::memory_order_release);
        EnterPolicyStand();
        std::cout << "[pico-cmd] A+X -> policy stand: a3_fast keeps running on the default standing pose "
                     "(G1 PLANNER); A+X again to follow, A+B+X+Y to disable\n";
        return;
      case PicoAction::kEnterTeleop:
        StartTeleop();
        std::cout << "[pico-cmd] A+X -> teleop: following the operator (A+X again: policy stand)\n";
        return;
      case PicoAction::kEnterTeleopWhenReady:
        pending_teleop_.store(true, std::memory_order_release);
        std::cout << "[pico-cmd] A+X while the PD stand ramp is still running: teleop starts as soon as the standing pose is reached\n";
        return;
      case PicoAction::kIgnoredNotEnabled:
        std::cout << "[pico-cmd] A+X ignored: the robot is disabled; press A+B+X+Y first\n";
        return;
    }
  }

  void EnterPolicyStand() {
    control_.teleop_input_paused.store(false, std::memory_order_release);
    control_.teleop_policy_stand.store(true, std::memory_order_release);
    control_.teleop_command_epoch.fetch_add(1, std::memory_order_acq_rel);
    // Stay in TELEOP so a3_fast keeps evaluating; only the reference changes.
    // Do not RequestDeployMode if already teleop: that would Reset() the buffers
    // and reprint "teleop -> teleop (restart)" for a reference-only switch.
    if (LoadDeployMode(control_) != DeployMode::kTeleop) {
      RequestDeployMode(control_, DeployMode::kTeleop, std::cout);
    }
  }

  void StartTeleop() {
    control_.teleop_input_paused.store(false, std::memory_order_release);
    control_.teleop_policy_stand.store(false, std::memory_order_release);
    control_.teleop_command_epoch.fetch_add(1, std::memory_order_acq_rel);
    RequestDeployMode(control_, DeployMode::kTeleop, std::cout);
  }

  void HelperLoop() {
    bool printed_init_done = false;
    while (running_.load(std::memory_order_acquire) &&
           !stop_requested_.load(std::memory_order_acquire)) {
      const DeployMode mode = LoadDeployMode(control_);
      const bool ready = control_.pd_stand_ready.load(std::memory_order_acquire);
      if (mode == DeployMode::kPdStand && ready && !printed_init_done) {
        std::cout << "Init Done\n";
        std::cout << "[pico-cmd] standing at the default pose: A+X to follow the operator, A+X again for policy stand, "
                     "A+B+X+Y to disable (simulator: MuJoCo window key 9 releases the elastic band)\n";
        printed_init_done = true;
      }
      if (mode != DeployMode::kPdStand) printed_init_done = false;

      if (pending_teleop_.load(std::memory_order_acquire) && mode == DeployMode::kPdStand && ready) {
        std::lock_guard<std::mutex> lock(mu_);
        if (pending_teleop_.exchange(false, std::memory_order_acq_rel) &&
            LoadDeployMode(control_) == DeployMode::kPdStand &&
            control_.pd_stand_ready.load(std::memory_order_acquire)) {
          StartTeleop();
          std::cout << "[pico-cmd] standing pose reached + pending A+X -> teleop\n";
        }
      } else if (mode != DeployMode::kPdStand) {
        pending_teleop_.store(false, std::memory_order_release);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  ManualControlState& control_;
  std::atomic<bool>& stop_requested_;
  std::string host_;
  int port_;
  std::string topic_;
  std::atomic<bool> running_{false};
  std::atomic<bool> pending_teleop_{false};
  std::mutex mu_;
  std::unique_ptr<ZMQPackedMessageSubscriber> sub_;
  std::thread helper_;
};

}  // namespace a3_deploy
