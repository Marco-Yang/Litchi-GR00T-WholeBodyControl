// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Subscribe to pico_manager's ZMQ "command" topic (same wire format as G1
// zmq_manager) and map A+B+X+Y start/stop onto the A3 manual state machine:
//   start -> PD stand (if needed) then teleop
//   stop  -> passive
// A3 has no planner policy; the planner bit is ignored for mode selection.
#pragma once

#include "a3_deploy/a3_manual_control.hpp"
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
              << "' (A+B+X+Y start->teleop, stop->passive; same as G1)\n";
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
    bool start = false;
    bool stop = false;
    if (!DecodeBoolField(hdr, bufs, "start", &start) ||
        !DecodeBoolField(hdr, bufs, "stop", &stop)) {
      return;
    }

    std::lock_guard<std::mutex> lock(mu_);
    if (stop) {
      pending_teleop_.store(false, std::memory_order_release);
      const DeployMode mode = LoadDeployMode(control_);
      if (mode != DeployMode::kPassive && mode != DeployMode::kIdle) {
        RequestDeployMode(control_, DeployMode::kPassive, std::cout);
        std::cout << "[pico-cmd] A+B+X+Y stop -> passive\n";
      }
      return;
    }
    if (!start) return;

    const DeployMode mode = LoadDeployMode(control_);
    if (mode == DeployMode::kTeleop) {
      // Already tracking; POSE/PLANNER toggles only change the Pico stream.
      return;
    }
    if (mode == DeployMode::kPdStand &&
        control_.pd_stand_ready.load(std::memory_order_acquire)) {
      control_.teleop_input_paused.store(false, std::memory_order_release);
      control_.teleop_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      RequestDeployMode(control_, DeployMode::kTeleop, std::cout);
      std::cout << "[pico-cmd] A+B+X+Y start -> teleop\n";
      return;
    }
    if (mode == DeployMode::kIdle || mode == DeployMode::kPassive) {
      RequestDeployMode(control_, DeployMode::kPdStand, std::cout);
      std::cout << "[pico-cmd] A+B+X+Y start -> pd_stand (waiting Init Done)\n";
    }
    pending_teleop_.store(true, std::memory_order_release);
  }

  void HelperLoop() {
    bool printed_init_done = false;
    while (running_.load(std::memory_order_acquire) &&
           !stop_requested_.load(std::memory_order_acquire)) {
      const DeployMode mode = LoadDeployMode(control_);
      const bool ready =
          control_.pd_stand_ready.load(std::memory_order_acquire);
      if (mode == DeployMode::kPdStand && ready && !printed_init_done) {
        std::cout << "Init Done\n";
        std::cout << "[pico-cmd] PD stand ready — press A+B+X+Y to start "
                     "teleop, then A+X for POSE; MuJoCo window key 9 releases "
                     "the elastic band\n";
        printed_init_done = true;
      }
      if (mode != DeployMode::kPdStand) printed_init_done = false;

      if (pending_teleop_.load(std::memory_order_acquire) &&
          mode == DeployMode::kPdStand && ready) {
        std::lock_guard<std::mutex> lock(mu_);
        if (pending_teleop_.exchange(false, std::memory_order_acq_rel) &&
            LoadDeployMode(control_) == DeployMode::kPdStand &&
            control_.pd_stand_ready.load(std::memory_order_acquire)) {
          control_.teleop_input_paused.store(false, std::memory_order_release);
          control_.teleop_command_epoch.fetch_add(1, std::memory_order_acq_rel);
          RequestDeployMode(control_, DeployMode::kTeleop, std::cout);
          std::cout << "[pico-cmd] Init Done + pending start -> teleop\n";
        }
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
