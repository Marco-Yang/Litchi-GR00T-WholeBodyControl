// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "a3_deploy/a3_ref_zmq_source.hpp"

#include "a3_deploy/a3_ta_bridge_codec.hpp"

#include <zmq.hpp>

#include <cerrno>
#include <chrono>
#include <iostream>
#include <utility>

namespace a3_deploy {
namespace {

std::int64_t NowSystemNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace

A3RefZmqSource::A3RefZmqSource(A3RefZmqSourceOptions options, FrameCallback callback)
    : options_(std::move(options)), callback_(std::move(callback)) {}

A3RefZmqSource::~A3RefZmqSource() { Stop(); }

std::string A3RefZmqSource::Endpoint() const {
  return "tcp://" + options_.host + ":" + std::to_string(options_.port);
}

bool A3RefZmqSource::Start() {
  if (running_.exchange(true)) return true;
  if (options_.host.empty() || options_.port <= 0 || options_.port > 65535 || !callback_) {
    running_.store(false);
    std::cerr << "[a3-ref-zmq] invalid endpoint '" << Endpoint() << "'\n";
    return false;
  }
  thread_ = std::thread([this] { Loop_(); });
  return true;
}

void A3RefZmqSource::Stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
}

void A3RefZmqSource::Loop_() {
  try {
    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, zmq::socket_type::sub);
    sub.set(zmq::sockopt::subscribe, kA3RefTopic);
    sub.set(zmq::sockopt::rcvtimeo, options_.rcv_timeout_ms);
    sub.set(zmq::sockopt::rcvhwm, options_.rcv_hwm);
    sub.set(zmq::sockopt::linger, 0);
    sub.connect(Endpoint());
    std::cout << "[a3-ref-zmq] teleop reference input: ZMQ SUB " << Endpoint() << " topic '" << kA3RefTopic << "'\n";
    while (running_.load(std::memory_order_acquire)) {
      zmq::message_t raw;
      zmq::recv_result_t res;
      try {
        res = sub.recv(raw, zmq::recv_flags::none);
      } catch (const zmq::error_t& e) {
        if (e.num() == EINTR) continue;
        std::cerr << "[a3-ref-zmq] zmq error: " << e.what() << "\n";
        break;
      }
      if (!res) continue;  // source quiet: nothing to do, the reference buffer handles the gap
      const std::int64_t arrival_ns = NowSystemNs();
      A3TeleopFrame frame{};
      std::string error;
      if (!ParseA3RefMessage(raw.data(), raw.size(), frame, &error)) {
        if ((rejected_.fetch_add(1, std::memory_order_relaxed) % 100) == 0)
          std::cerr << "[a3-ref-zmq] WARN: reject frame: " << error << "\n";
        continue;
      }
      frame.stamp_ns = arrival_ns;  // local clock, same as the AimRT /ta path
      received_.fetch_add(1, std::memory_order_relaxed);
      callback_(frame);
    }
  } catch (const std::exception& e) {
    std::cerr << "[a3-ref-zmq] receive thread failed: " << e.what() << "\n";
  }
  running_.store(false, std::memory_order_release);
}

}  // namespace a3_deploy
