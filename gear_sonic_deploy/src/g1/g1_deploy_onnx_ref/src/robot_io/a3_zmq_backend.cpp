// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "robot_io/a3_zmq_backend.hpp"

#include <zmq.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <random>
#include <sstream>
#include <thread>
#include <vector>

#include "robot_io/a3_layout_extra.hpp"

namespace robot_io {
namespace {

std::int64_t NowSystemNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void Trim(std::string& s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  const auto e = s.find_last_not_of(" \t\r\n");
  s = (b == std::string::npos) ? std::string{} : s.substr(b, e - b + 1);
}

}  // namespace

A3ZmqBackend::A3ZmqBackend() = default;
A3ZmqBackend::~A3ZmqBackend() { Stop(); }

bool A3ZmqBackend::ParseConfig_(const std::string& config) {
  std::stringstream ss(config);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    const auto eq = tok.find('=');
    if (eq == std::string::npos) continue;
    std::string key = tok.substr(0, eq), val = tok.substr(eq + 1);
    Trim(key);
    Trim(val);
    try {
      if (key == "relay_host") host_ = val;
      else if (key == "state_port") state_port_ = std::stoi(val);
      else if (key == "command_port") command_port_ = std::stoi(val);
      else if (key == "max_frame_age_ms") max_frame_age_ms_ = std::stod(val);
      else if (key == "connect_timeout_s") connect_timeout_s_ = std::stod(val);
      else if (key == "state_hz") state_hz_ = std::stod(val);
      else if (key == "publish_enabled") publish_enabled_ = !(val == "false" || val == "0");
      else std::cerr << "[a3_zmq] ignoring unknown config key '" << key << "'\n";
    } catch (const std::exception&) {
      std::cerr << "[a3_zmq] bad value for '" << key << "': '" << val << "'\n";
      return false;
    }
  }
  if (host_.empty()) {
    std::cerr << "[a3_zmq] relay_host is required\n";
    return false;
  }
  if (state_port_ <= 0 || state_port_ > 65535 || command_port_ <= 0 || command_port_ > 65535 || max_frame_age_ms_ <= 0.0 ||
      state_hz_ <= 0.0) {
    std::cerr << "[a3_zmq] port / age / rate out of range\n";
    return false;
  }
  return true;
}

bool A3ZmqBackend::Init(const std::string& config) {
  if (!ParseConfig_(config)) return false;
  std::mt19937_64 rng{std::random_device{}()};
  session_.store(rng() | 1ULL);  // 0 is "no session" on the wire
  inited_.store(true);
  return true;
}

bool A3ZmqBackend::Start() {
  if (!inited_.load()) {
    std::cerr << "[a3_zmq] Start() before Init()\n";
    return false;
  }
  if (running_.load()) return true;
  try {
    ctx_ = std::make_unique<zmq::context_t>(1);
    state_sub_ = std::make_unique<zmq::socket_t>(*ctx_, zmq::socket_type::sub);
    cmd_pub_ = std::make_unique<zmq::socket_t>(*ctx_, zmq::socket_type::pub);
    state_sub_->set(zmq::sockopt::subscribe, "");
    state_sub_->set(zmq::sockopt::rcvhwm, 2);
    state_sub_->set(zmq::sockopt::conflate, 1);  // only the newest state matters
    state_sub_->set(zmq::sockopt::rcvtimeo, 50);
    state_sub_->set(zmq::sockopt::linger, 0);
    cmd_pub_->set(zmq::sockopt::sndhwm, 4);
    cmd_pub_->set(zmq::sockopt::immediate, 1);  // never queue commands for a connection that is not up yet
    cmd_pub_->set(zmq::sockopt::linger, 0);
    state_sub_->connect("tcp://" + host_ + ":" + std::to_string(state_port_));
    cmd_pub_->connect("tcp://" + host_ + ":" + std::to_string(command_port_));
  } catch (const zmq::error_t& e) {
    std::cerr << "[a3_zmq] socket setup failed: " << e.what() << "\n";
    return false;
  }
  running_.store(true);
  rx_thread_ = std::thread([this] { RxLoop_(); });
  std::cerr << "[a3_zmq] relay " << host_ << ": state tcp port " << state_port_ << ", commands tcp port " << command_port_
            << ", session " << std::hex << session_.load() << std::dec << "\n";

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(connect_timeout_s_ * 1000));
  bool got = false;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(rx_mtx_);
      got = rx_.have && rx_.frame.state_valid();
    }
    if (got) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (got) {
    const Stats s = GetStats();
    std::cerr << "[a3_zmq] relay state is flowing (relay mode " << a3_relay::ModeName(s.relay_mode)
              << (s.relay_forwarding ? "" : ", LISTEN-ONLY: it will refuse commands") << ")\n";
  } else {
    std::cerr << "[a3_zmq] WARN: no valid state frame from the relay within " << connect_timeout_s_
              << " s (wrong --relay-host / ports, relay not running, or the robot-side topics are stale); proceeding: the "
                 "driver's watchdog treats missing state as unsafe, so nothing will be commanded until it arrives\n";
  }
  return true;
}

void A3ZmqBackend::Stop() {
  if (running_.exchange(false)) {
    if (rx_thread_.joinable()) rx_thread_.join();
    const Stats s = GetStats();
    std::cerr << "[a3_zmq] stopped: state frames " << s.frames << " (bad " << s.bad_frames << ", gaps " << s.seq_gaps
              << "), commands sent " << s.commands_sent << " (send failures " << s.send_failures << "), relay accepted "
              << s.relay_accepted << " refused " << s.relay_rejected << "\n";
  }
  state_sub_.reset();
  cmd_pub_.reset();
  ctx_.reset();
}

const JointLayout& A3ZmqBackend::GetLayout() const { return MakeA3Layout31(); }

void A3ZmqBackend::RxLoop_() {
  std::vector<std::uint8_t> buf;
  while (running_.load(std::memory_order_acquire)) {
    zmq::message_t msg;
    zmq::recv_result_t r;
    try {
      r = state_sub_->recv(msg, zmq::recv_flags::none);  // rcvtimeo 50 ms
    } catch (const zmq::error_t&) {
      continue;
    }
    if (!r) continue;
    const std::int64_t recv_ns = NowSystemNs();
    a3_relay::StateFrame f;
    std::lock_guard<std::mutex> lk(rx_mtx_);
    if (a3_relay::ParseState(msg.data(), msg.size(), f) != a3_relay::ParseResult::kOk) {
      ++stats_.bad_frames;
      continue;
    }
    if (f.relay_id == last_relay_id_ && rx_.have) {
      const auto gap = static_cast<std::int32_t>(f.seq - last_seq_);
      if (gap <= 0) continue;  // stale / duplicate: keep the newer one we already have
      if (gap > 1) stats_.seq_gaps += static_cast<std::uint64_t>(gap - 1);
    } else if (rx_.have) {
      std::cerr << "[a3_zmq] relay restarted (new relay id)\n";
    }
    const bool mode_changed = !rx_.have || f.mode != rx_.frame.mode || f.last_reject != rx_.frame.last_reject;
    if (mode_changed) {
      std::cerr << "[a3_zmq] relay mode " << a3_relay::ModeName(f.mode) << ", last refused: " << a3_relay::RejectName(f.last_reject)
                << " (accepted " << f.accepted_total << ", refused " << f.rejected_total << ")"
                << (f.mode == a3_relay::Mode::kDamp ? "  -- relay is damping; the next enable (A+B+X+Y or S) takes over again" : "") << "\n";
    }
    last_relay_id_ = f.relay_id;
    last_seq_ = f.seq;
    rx_.frame = f;
    rx_.recv_sys_ns = recv_ns;
    rx_.have = true;
    if (f.mode == a3_relay::Mode::kActive && f.owner_session == session_.load(std::memory_order_acquire)) armed_seen_ = true;
    ++stats_.frames;
    stats_.relay_mode = f.mode;
    stats_.relay_last_reject = f.last_reject;
    stats_.relay_accepted = f.accepted_total;
    stats_.relay_rejected = f.rejected_total;
    stats_.relay_forwarding = f.forwarding();
  }
}

A3ZmqBackend::Stats A3ZmqBackend::GetStats() const {
  std::lock_guard<std::mutex> lk(rx_mtx_);
  Stats s = stats_;
  s.commands_sent = commands_sent_.load(std::memory_order_relaxed);
  s.send_failures = send_failures_.load(std::memory_order_relaxed);
  return s;
}

bool A3ZmqBackend::TryReadLatestState(RobotState& out) {
  a3_relay::StateFrame f;
  std::int64_t recv_ns = 0;
  {
    std::lock_guard<std::mutex> lk(rx_mtx_);
    if (!rx_.have) return false;
    f = rx_.frame;
    recv_ns = rx_.recv_sys_ns;
  }
  const std::int64_t now = NowSystemNs();
  if (!f.state_valid()) return false;                                       // the relay says a robot-side channel is stale
  if (now - recv_ns > static_cast<std::int64_t>(max_frame_age_ms_ * 1e6)) return false;  // link is late / gone

  out.timestamp_ns = recv_ns - f.state_age_ns;
  out.tick = ++tick_;
  out.state_data_ready_ns = recv_ns - f.data_age_ns;
  out.state_sync_ready_ns = now;
  out.sync_complete = true;
  out.sync_aligned = true;
  out.sync_skew_ns = 0;
  out.q.resize(a3_relay::kDof);
  out.dq.resize(a3_relay::kDof);
  out.tau_est.resize(a3_relay::kDof);
  for (int i = 0; i < a3_relay::kDof; ++i) {
    out.q[i] = f.q[i];
    out.dq[i] = f.dq[i];
    out.tau_est[i] = f.tau[i];
  }
  for (int k = 0; k < 4; ++k) {
    out.imu_quat_wxyz[k] = f.pelvis.quat_wxyz[k];
    out.sec_imu_quat_wxyz[k] = f.torso.quat_wxyz[k];
  }
  for (int k = 0; k < 3; ++k) {
    out.imu_gyro[k] = f.pelvis.gyro[k];
    out.imu_accel[k] = f.pelvis.accel[k];
    out.sec_imu_gyro[k] = f.torso.gyro[k];
    out.sec_imu_accel[k] = f.torso.accel[k];
  }
  out.has_secondary_imu = f.has_secondary_imu();
  return true;
}

void A3ZmqBackend::NotifyEnable() {
  std::mt19937_64 rng{std::random_device{}()};
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    session_.store(rng() | 1ULL, std::memory_order_release);
    cmd_seq_ = 0;
  }
  {
    std::lock_guard<std::mutex> lk(rx_mtx_);
    armed_seen_ = false;
  }
  enable_ns_.store(NowSystemNs(), std::memory_order_release);
  std::cerr << "[a3_zmq] robot enabled: new session " << std::hex << session_.load() << std::dec
            << " (a relay that latched its failsafe accepts it if the first command is near the measured pose)\n";
}

bool A3ZmqBackend::LinkFault(std::string* reason) {
  const std::int64_t enabled_for = NowSystemNs() - enable_ns_.load(std::memory_order_acquire);
  std::lock_guard<std::mutex> lk(rx_mtx_);
  if (!rx_.have) return false;  // no state at all: the driver's watchdog handles that
  const auto& f = rx_.frame;
  constexpr std::int64_t kArmGraceNs = 1'500'000'000LL;  // time the relay gets to accept a new session
  if (f.mode == a3_relay::Mode::kDamp) {
    // A relay that still shows the DAMP of an earlier session gets a grace period to accept our new one.
    const bool arming_in_progress = !armed_seen_ && enabled_for < kArmGraceNs;
    if (!arming_in_progress) {
      if (reason) {
        *reason = std::string("the relay is damping (failsafe latched, last refused: ") + a3_relay::RejectName(f.last_reject) + ")";
      }
      return true;
    }
  }
  if (!f.forwarding() && enabled_for > kArmGraceNs) {
    if (reason) *reason = "the relay is in --listen-only mode: it refuses every command";
    return true;
  }
  return false;
}

bool A3ZmqBackend::SendCommand(const RobotCommand& cmd) {
  const int n = a3_relay::kDof;
  if (cmd.q_des.size() != n || cmd.dq_des.size() != n || cmd.tau_ff.size() != n || cmd.kp.size() != n || cmd.kd.size() != n) return false;
  if (!publish_enabled_ || !running_.load(std::memory_order_acquire)) return false;
  a3_relay::Command c;
  c.session = session_.load(std::memory_order_acquire);
  c.send_ns = NowSystemNs();
  for (int i = 0; i < n; ++i) {
    c.q_des[i] = cmd.q_des[i];
    c.dq_des[i] = cmd.dq_des[i];
    c.tau_ff[i] = cmd.tau_ff[i];
    c.kp[i] = cmd.kp[i];
    c.kd[i] = cmd.kd[i];
  }
  std::uint8_t buf[a3_relay::kCmdBytes];
  bool ok = false;
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    c.seq = ++cmd_seq_;
    a3_relay::PackCommand(c, buf);
    try {
      ok = cmd_pub_ && cmd_pub_->send(zmq::buffer(buf, sizeof(buf)), zmq::send_flags::dontwait).has_value();
    } catch (const zmq::error_t&) {
      ok = false;
    }
  }
  (ok ? commands_sent_ : send_failures_).fetch_add(1, std::memory_order_relaxed);
  // A relay that latched its failsafe refuses these frames; the deploy's link supervisor (LinkFault) turns that into "passive" and says why.
  if (ok && LinkFault(nullptr)) return false;
  return ok;
}

}  // namespace robot_io
