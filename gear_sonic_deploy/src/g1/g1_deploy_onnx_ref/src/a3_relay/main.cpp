// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// a3_zmq_relay: the robot-side half of "run SONIC on the operator PC, not on the MDU".
//
// It runs on the MDU next to body-drive and speaks the robot's own transport (iceoryx; the same A3AimrtBackend and
// a3_aimrt_config.iceoryx.yaml the full deploy uses) on one side and ZMQ on the other:
//
//   hal_ethercat --iceoryx--> relay --ZMQ PUB (state, A3ST)--------> PC: a3_deploy_onnx_ref --robot-link zmq
//   hal_ethercat <--iceoryx-- relay <--ZMQ SUB (commands, A3CM)----- PC
//
// The relay does no inference and holds no policy.  What it adds is the part that has to keep working when the PC or the network
// does not (RelayGuard, include/a3_relay/a3_relay_guard.hpp): commands are validated (finite, gain / torque / velocity limits,
// distance to the measured pose, step size, one owner session, increasing sequence), the first frame must be close to the measured
// pose, and if the PC goes quiet the relay holds the last command briefly and then ramps to pure damping (latched).
//
// It never starts motion by itself: nothing is published until the PC's first valid frame arrives, and `--listen-only` keeps
// the command publishers from even being created (state is forwarded, commands are refused).  The vendor motion_control (`mc`)
// must not run at the same time (sm_config Motion group = ["agent"], docs/notes/a3/pico_real_teleop.md).
//
//   a3_zmq_relay --aimrt-cfg config/a3_aimrt_config.iceoryx.yaml --bind 10.42.10.12 [--allow-peer 10.42.10.99] [--listen-only]
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <zmq.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "a3_deploy/expand_to_backend.hpp"
#include "a3_policy_parameters.hpp"
#include "a3_relay/a3_relay_guard.hpp"
#include "a3_relay/a3_relay_wire.hpp"
#include "robot_io/a3_aimrt_backend.hpp"
#include "robot_io/a3_layout_extra.hpp"

namespace {

volatile std::sig_atomic_t g_signals = 0;
void OnSignal(int) { g_signals = g_signals + 1; }

std::int64_t MonoNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::int64_t SystemNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

bool HasFlag(int argc, char** argv, const char* name) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == name) return true;
  }
  return false;
}
std::string StrArg(int argc, char** argv, const char* name, const std::string& dflt) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == name) return argv[i + 1];
  }
  return dflt;
}
double NumArg(int argc, char** argv, const char* name, double dflt) {
  const std::string s = StrArg(argc, argv, name, "");
  if (s.empty()) return dflt;
  try {
    return std::stod(s);
  } catch (const std::exception&) {
    std::cerr << "[relay] bad value for " << name << ": '" << s << "'\n";
    std::exit(64);
  }
}

void Usage() {
  std::cerr <<
      "a3_zmq_relay --aimrt-cfg PATH [options]\n"
      "  --aimrt-cfg PATH        AimRT config of the robot side (config/a3_aimrt_config.iceoryx.yaml on the MDU; the ros2 one for the simulator)\n"
      "  --bind IP               interface address for both ZMQ sockets (default 127.0.0.1; use the robot-network address, e.g. 10.42.10.12)\n"
      "  --state-port N          state PUB port (default 5570)       --command-port N   command SUB port (default 5571)\n"
      "  --allow-peer IP         accept command frames only from this sender address (default: any peer that can reach the port)\n"
      "  --listen-only           forward state, refuse every command, create no command publisher\n"
      "  --state-hz F            state frame rate (default 200)       --resend-hz F      HOLD / DAMP re-send rate (default 50)\n"
      "  --stale-ms F            silence before HOLD (default 60)     --hold-ms F        HOLD length before DAMP (default 150)\n"
      "  --ramp-ms F             gain ramp to damping (default 300)   --shutdown-damp-ms F  keep damping this long after SIGINT/SIGTERM (default 2000)\n"
      "  --arm-max-q-err F       first frame vs measured pose, rad (0.35; neck joints --arm-max-neck-err 1.0)\n"
      "  --max-q-err F           q_des vs measured while forwarding, rad (7.0)       --max-q-step F  q_des step between frames, rad (6.0)\n"
      "                          (absurdity bounds: the deploy itself sends errors of 4.8 rad / steps of 4.7 rad when the robot hangs in the band)\n"
      "  --max-dq-des F (5.0)    --max-tau-ff F (1.0)    --gain-max-scale F  limit = scale x largest policy / PD-stand gain (2.5)\n"
      "  --gain-profile P        policy gain table used for the DAMP kd: a3_024 (default; Litchi's real-robot profile) or a3\n"
      "  --damp-kd-scale F       DAMP kd = scale x that table (1.0)\n"
      "  --max-sample-age-ms F   per-channel freshness of the robot-side state (10)\n"
      "  --rt-priority N         SCHED_FIFO priority for the relay loop (default 0 = off; needs CAP_SYS_NICE)\n"
      "  --status-every-s F      status line period (default 2)\n";
}

// Maximum / damping gains in SDK (31-DOF) order, from the policy tables the deploy uses.
// The DAMP target kd is the policy kd table of the real-robot gain profile (Litchi's runtime config: policy_control.profile a3_024).
void BuildGainTables(double gain_max_scale, double damp_kd_scale, bool profile_a3_024, a3_relay::GuardConfig& cfg) {
  a3_relay::Vec kp_ref{}, kd_ref{}, kd_damp{};
  kp_ref.fill(a3_deploy::kA3HeadKp);
  kd_ref.fill(a3_deploy::kA3HeadKd);
  kd_damp.fill(a3_deploy::kA3HeadKd);
  for (int i = 0; i < robot_io::kA3PolicyDof; ++i) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[i];
    kp_ref[sdk] = std::max({a3_kps[i], a3_024_kps[i], a3_pd_stand_kps[i]});
    kd_ref[sdk] = std::max({a3_kds[i], a3_024_kds[i], a3_pd_stand_kds[i]});
    kd_damp[sdk] = profile_a3_024 ? a3_024_kds[i] : a3_kds[i];
  }
  for (int i = 0; i < a3_relay::kDof; ++i) {
    cfg.kp_max[i] = kp_ref[i] * gain_max_scale;
    cfg.kd_max[i] = kd_ref[i] * gain_max_scale;
    cfg.damp_kd[i] = std::min(kd_damp[i] * damp_kd_scale, cfg.kd_max[i]);
  }
}

bool FillStateFrame(const robot_io::RobotState& s, std::int64_t now_sys_ns, a3_relay::StateFrame& f) {
  if (s.q.size() != a3_relay::kDof || s.dq.size() != a3_relay::kDof || s.tau_est.size() != a3_relay::kDof) return false;
  for (int i = 0; i < a3_relay::kDof; ++i) {
    f.q[i] = s.q[i];
    f.dq[i] = s.dq[i];
    f.tau[i] = s.tau_est[i];
  }
  auto imu = [](const Eigen::Vector4d& q, const Eigen::Vector3d& g, const Eigen::Vector3d& a, a3_relay::ImuFrame& o) {
    for (int k = 0; k < 4; ++k) o.quat_wxyz[k] = q[k];
    for (int k = 0; k < 3; ++k) {
      o.gyro[k] = g[k];
      o.accel[k] = a[k];
    }
  };
  imu(s.imu_quat_wxyz, s.imu_gyro, s.imu_accel, f.pelvis);
  imu(s.sec_imu_quat_wxyz, s.sec_imu_gyro, s.sec_imu_accel, f.torso);
  if (s.has_secondary_imu) f.flags |= a3_relay::kFlagSecondaryImu;
  constexpr std::int64_t kMaxAge = 10'000'000'000LL;  // a nonsense clock must not read as "fresh"
  f.state_age_ns = std::clamp<std::int64_t>(now_sys_ns - s.timestamp_ns, 0, kMaxAge);
  f.data_age_ns = std::clamp<std::int64_t>(now_sys_ns - s.state_data_ready_ns, 0, kMaxAge);
  return true;
}

robot_io::RobotCommand ToRobotCommand(const a3_relay::RelayGuard::Output& o) {
  robot_io::RobotCommand c;
  c.q_des.resize(a3_relay::kDof);
  c.dq_des.resize(a3_relay::kDof);
  c.tau_ff.resize(a3_relay::kDof);
  c.kp.resize(a3_relay::kDof);
  c.kd.resize(a3_relay::kDof);
  for (int i = 0; i < a3_relay::kDof; ++i) {
    c.q_des[i] = o.q_des[i];
    c.dq_des[i] = o.dq_des[i];
    c.tau_ff[i] = o.tau_ff[i];
    c.kp[i] = o.kp[i];
    c.kd[i] = o.kd[i];
  }
  return c;
}

bool PeerAllowed(zmq::message_t& m, const std::string& allow) {
  if (allow.empty()) return true;
  try {
    const char* p = m.gets("Peer-Address");
    if (!p) return false;
    const std::string s(p);
    return s.substr(0, s.rfind(':')) == allow;
  } catch (const zmq::error_t&) {
    return false;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (HasFlag(argc, argv, "-h") || HasFlag(argc, argv, "--help")) {
    Usage();
    return 0;
  }
  const std::string aimrt_cfg = StrArg(argc, argv, "--aimrt-cfg", "");
  if (aimrt_cfg.empty()) {
    Usage();
    return 64;
  }
  const std::string bind_ip = StrArg(argc, argv, "--bind", "127.0.0.1");
  if (bind_ip == "0.0.0.0" || bind_ip == "*") {
    std::cerr << "[relay] refusing to bind every interface; pass the robot-network address (e.g. --bind 10.42.10.12)\n";
    return 64;
  }
  const int state_port = static_cast<int>(NumArg(argc, argv, "--state-port", 5570));
  const int command_port = static_cast<int>(NumArg(argc, argv, "--command-port", 5571));
  const std::string allow_peer = StrArg(argc, argv, "--allow-peer", "");
  const bool listen_only = HasFlag(argc, argv, "--listen-only");
  const double state_hz = NumArg(argc, argv, "--state-hz", 200.0);
  const double resend_hz = NumArg(argc, argv, "--resend-hz", 50.0);
  const double shutdown_damp_ms = NumArg(argc, argv, "--shutdown-damp-ms", 2000.0);
  const double status_every_s = NumArg(argc, argv, "--status-every-s", 2.0);
  const int rt_priority = static_cast<int>(NumArg(argc, argv, "--rt-priority", 0));
  if (state_hz <= 0 || resend_hz <= 0 || state_hz > 2000 || resend_hz > 1000) {
    std::cerr << "[relay] --state-hz / --resend-hz out of range\n";
    return 64;
  }

  a3_relay::GuardConfig gcfg;
  gcfg.stale_ms = NumArg(argc, argv, "--stale-ms", gcfg.stale_ms);
  gcfg.hold_ms = NumArg(argc, argv, "--hold-ms", gcfg.hold_ms);
  gcfg.ramp_ms = NumArg(argc, argv, "--ramp-ms", gcfg.ramp_ms);
  gcfg.arm_max_q_err_rad = NumArg(argc, argv, "--arm-max-q-err", gcfg.arm_max_q_err_rad);
  gcfg.arm_max_neck_err_rad = NumArg(argc, argv, "--arm-max-neck-err", gcfg.arm_max_neck_err_rad);
  gcfg.max_q_err_rad = NumArg(argc, argv, "--max-q-err", gcfg.max_q_err_rad);
  gcfg.max_q_step_rad = NumArg(argc, argv, "--max-q-step", gcfg.max_q_step_rad);
  gcfg.max_abs_dq_des = NumArg(argc, argv, "--max-dq-des", gcfg.max_abs_dq_des);
  gcfg.max_abs_tau_ff = NumArg(argc, argv, "--max-tau-ff", gcfg.max_abs_tau_ff);
  const std::string gain_profile = StrArg(argc, argv, "--gain-profile", "a3_024");
  if (gain_profile != "a3_024" && gain_profile != "a3") {
    std::cerr << "[relay] --gain-profile must be a3_024 (Litchi's real-robot profile, default) or a3\n";
    return 64;
  }
  BuildGainTables(NumArg(argc, argv, "--gain-max-scale", 2.5), NumArg(argc, argv, "--damp-kd-scale", 1.0), gain_profile == "a3_024", gcfg);
  const double max_sample_age_ms = NumArg(argc, argv, "--max-sample-age-ms", 10.0);
  if (gcfg.stale_ms <= 0 || gcfg.hold_ms < 0 || gcfg.ramp_ms < 0) {
    std::cerr << "[relay] --stale-ms must be > 0, --hold-ms / --ramp-ms >= 0\n";
    return 64;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  std::signal(SIGPIPE, SIG_IGN);

  // ---- robot side ----------------------------------------------------------------------------------------------------------
  std::ostringstream bcfg;
  bcfg << "cfg_file_path=" << aimrt_cfg << ",sync_mode=latest_cache,sync_hz=" << state_hz
       << ",max_sample_age_ms=" << max_sample_age_ms << ",publish_enabled=" << (listen_only ? "false" : "true")
       << ",hand_bridge_enabled=false";
  robot_io::A3AimrtBackend backend;
  if (!backend.Init(bcfg.str())) {
    std::cerr << "[relay] A3AimrtBackend::Init failed\n";
    return 1;
  }
  if (!backend.Start()) {
    std::cerr << "[relay] A3AimrtBackend::Start failed\n";
    return 1;
  }

  // ---- ZMQ side ------------------------------------------------------------------------------------------------------------
  zmq::context_t ctx(1);
  zmq::socket_t state_pub(ctx, zmq::socket_type::pub);
  zmq::socket_t cmd_sub(ctx, zmq::socket_type::sub);
  try {
    state_pub.set(zmq::sockopt::sndhwm, 2);
    state_pub.set(zmq::sockopt::linger, 0);
    cmd_sub.set(zmq::sockopt::rcvhwm, 8);
    cmd_sub.set(zmq::sockopt::linger, 0);
    cmd_sub.set(zmq::sockopt::subscribe, "");
    state_pub.bind("tcp://" + bind_ip + ":" + std::to_string(state_port));
    cmd_sub.bind("tcp://" + bind_ip + ":" + std::to_string(command_port));
  } catch (const zmq::error_t& e) {
    std::cerr << "[relay] ZMQ bind failed on " << bind_ip << " (" << e.what() << "); is another relay running or the address wrong?\n";
    backend.Stop();
    return 1;
  }

  if (rt_priority > 0) {
    sched_param sp{};
    sp.sched_priority = rt_priority;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    std::cerr << "[relay] SCHED_FIFO " << rt_priority << (rc == 0 ? " set" : " NOT set (needs CAP_SYS_NICE / root); running as SCHED_OTHER") << "\n";
  }

  std::mt19937_64 rng{std::random_device{}()};
  const std::uint64_t relay_id = rng() | 1ULL;
  a3_relay::RelayGuard guard(gcfg);

  std::cerr << "[relay] up: state tcp://" << bind_ip << ":" << state_port << " (PUB), commands tcp://" << bind_ip << ":" << command_port
            << " (SUB), relay_id=" << std::hex << relay_id << std::dec << (listen_only ? "  LISTEN-ONLY (commands refused)" : "")
            << (allow_peer.empty() ? "" : ("  allow-peer=" + allow_peer)) << "\n"
            << "[relay] failsafe: stale " << gcfg.stale_ms << " ms -> HOLD " << gcfg.hold_ms << " ms -> DAMP (ramp " << gcfg.ramp_ms
            << " ms, latched until a new deploy session arms near the measured pose); limits: arm " << gcfg.arm_max_q_err_rad
            << " rad (neck " << gcfg.arm_max_neck_err_rad << "), q_err " << gcfg.max_q_err_rad << " rad, step " << gcfg.max_q_step_rad << " rad, kp/kd max "
            << "scaled policy gains\n"
            << "[relay] NOTHING is published to the robot until the PC's first valid command frame"
            << (listen_only ? " (never, in listen-only mode)" : "") << "; stop with Ctrl-C / SIGTERM (damps for "
            << shutdown_damp_ms << " ms first; a second signal exits at once)\n";

  // ---- loop ----------------------------------------------------------------------------------------------------------------
  const std::int64_t state_period = static_cast<std::int64_t>(1e9 / state_hz);
  const std::int64_t resend_period = static_cast<std::int64_t>(1e9 / resend_hz);
  std::int64_t now = MonoNs();
  std::int64_t next_state = now, next_resend = now, next_status = now + static_cast<std::int64_t>(status_every_s * 1e9);
  std::int64_t shutdown_deadline = 0;
  bool shutting_down = false;
  a3_relay::Mode last_mode = a3_relay::Mode::kDisarmed;
  std::uint32_t state_seq = 0;
  std::uint64_t cmd_rx_window = 0, cmd_rx_total = 0, published_total = 0, bad_frames = 0;
  std::int64_t last_reject_log = 0;
  a3_relay::RelayGuard::Detail last_detail{};
  robot_io::RobotState rs;
  bool have_state_once = false;
  std::vector<std::uint8_t> out_buf(a3_relay::kStateBytes);

  auto publish = [&](const a3_relay::RelayGuard::Output& o) {
    if (listen_only) return;
    if (backend.SendCommand(ToRobotCommand(o))) ++published_total;
  };

  while (true) {
    now = MonoNs();
    if (g_signals >= 2) {
      std::cerr << "[relay] second signal: exiting now\n";
      break;
    }
    if (g_signals >= 1 && !shutting_down) {
      shutting_down = true;
      guard.BeginShutdownDamp(now);
      shutdown_deadline = now + static_cast<std::int64_t>(shutdown_damp_ms * 1e6);
      if (guard.mode() == a3_relay::Mode::kDamp) {
        std::cerr << "[relay] signal: damping for " << shutdown_damp_ms << " ms, then exiting\n";
      } else {
        std::cerr << "[relay] signal: nothing was ever commanded, nothing to damp; exiting\n";
      }
    }
    if (shutting_down && (now >= shutdown_deadline || guard.mode() == a3_relay::Mode::kDisarmed)) break;

    const std::int64_t due = std::min(next_state, next_resend);
    const int timeout_ms = static_cast<int>(std::max<std::int64_t>(0, (due - now) / 1'000'000));
    zmq::pollitem_t item{cmd_sub.handle(), 0, ZMQ_POLLIN, 0};
    try {
      zmq::poll(&item, 1, std::chrono::milliseconds(timeout_ms));
    } catch (const zmq::error_t& e) {
      if (e.num() != EINTR) throw;
    }

    now = MonoNs();
    if (item.revents & ZMQ_POLLIN) {
      // Drain everything that is queued; only the newest valid frame is published (a burst after a hiccup is not replayed).
      robot_io::RobotState probe;
      a3_relay::RelayGuard::Measured measured;
      measured.valid = backend.TryReadLatestState(probe) && probe.q.size() == a3_relay::kDof;
      if (measured.valid) {
        for (int i = 0; i < a3_relay::kDof; ++i) measured.q[i] = probe.q[i];
      }
      std::optional<a3_relay::RelayGuard::Output> to_publish;
      zmq::message_t msg;
      while (true) {
        zmq::recv_result_t r;
        try {
          r = cmd_sub.recv(msg, zmq::recv_flags::dontwait);
        } catch (const zmq::error_t&) {
          break;
        }
        if (!r) break;
        ++cmd_rx_window;
        ++cmd_rx_total;
        a3_relay::Reject reject = a3_relay::Reject::kNone;
        a3_relay::Command cmd;
        if (listen_only) {
          reject = a3_relay::Reject::kListenOnly;
          guard.NoteExternalReject(reject);
        } else if (!PeerAllowed(msg, allow_peer)) {
          reject = a3_relay::Reject::kPeer;
          guard.NoteExternalReject(reject);
        } else if (a3_relay::ParseCommand(msg.data(), msg.size(), cmd) != a3_relay::ParseResult::kOk) {
          reject = a3_relay::Reject::kBadFrame;
          ++bad_frames;
          guard.NoteExternalReject(reject);
        } else {
          auto d = guard.OnCommand(cmd, measured, MonoNs());
          reject = d.reject;
          if (d.publish) to_publish = d.out;
          if (d.reject != a3_relay::Reject::kNone) last_detail = d.detail;
        }
        if (reject != a3_relay::Reject::kNone && now - last_reject_log > 1'000'000'000LL) {
          last_reject_log = now;
          std::cerr << "[relay] command frame refused: " << a3_relay::RejectName(reject);
          if (last_detail.what[0] != '\0' && (reject == a3_relay::Reject::kLimit || reject == a3_relay::Reject::kArmFar)) {
            std::cerr << " (" << last_detail.what << " joint " << last_detail.joint << ": " << last_detail.value << " vs limit "
                      << last_detail.limit << "; q_des " << last_detail.q_des << ", previous " << last_detail.q_prev << ", measured "
                      << last_detail.q_meas << ")";
          }
          std::cerr << " (mode " << a3_relay::ModeName(guard.mode()) << ", " << guard.rejected_total() << " refused so far)\n";
        }
      }
      if (to_publish) publish(*to_publish);
      a3_relay::RelayGuard::Detail big;
      if (guard.TakeNewMaxStep(big) && big.value > 1.0) {
        std::cerr << "[relay] note: largest accepted step so far, joint " << big.joint << ": " << big.value << " rad (q_des " << big.q_des
                  << ", previous " << big.q_prev << ", measured " << big.q_meas << ")\n";
      }
    }

    now = MonoNs();
    if (now >= next_resend) {
      if (auto o = guard.Tick(now)) publish(*o);
      next_resend += resend_period;
      if (next_resend < now) next_resend = now + resend_period;
    }

    const a3_relay::Mode mode_now = guard.Update(now);
    if (mode_now != last_mode) {
      std::cerr << "[relay] mode " << a3_relay::ModeName(last_mode) << " -> " << a3_relay::ModeName(mode_now)
                << " (owner " << std::hex << guard.owner_session() << std::dec << ", accepted " << guard.accepted_total() << ")\n";
      last_mode = mode_now;
    }

    if (now >= next_state) {
      a3_relay::StateFrame f;
      f.relay_id = relay_id;
      f.seq = ++state_seq;
      f.mode = mode_now;
      f.last_reject = guard.last_reject();
      f.owner_session = guard.owner_session();
      f.last_accepted_seq = guard.last_accepted_seq();
      f.accepted_total = guard.accepted_total();
      f.rejected_total = guard.rejected_total();
      f.relay_ns = SystemNs();
      if (!listen_only) f.flags |= a3_relay::kFlagForwarding;
      if (backend.TryReadLatestState(rs) && FillStateFrame(rs, f.relay_ns, f)) {
        f.flags |= a3_relay::kFlagStateValid;
        have_state_once = true;
      } else if (have_state_once && FillStateFrame(rs, f.relay_ns, f)) {
        // channel went stale: keep the last numbers but flag them invalid (the PC refuses to act on them)
      }
      a3_relay::PackState(f, out_buf.data());
      try {
        state_pub.send(zmq::buffer(out_buf.data(), out_buf.size()), zmq::send_flags::dontwait);
      } catch (const zmq::error_t&) {
      }
      next_state += state_period;
      if (next_state < now) next_state = now + state_period;
    }

    if (status_every_s > 0 && now >= next_status) {
      const double dt = status_every_s;
      std::fprintf(stderr,
                   "[relay] %-8s owner=%llx acc=%u rej=%u(%s) cmd_rx=%.0f/s published=%llu state=%s q_err_max=%.3f(j%d) step_max=%.3f(j%d)\n",
                   a3_relay::ModeName(mode_now), static_cast<unsigned long long>(guard.owner_session()), guard.accepted_total(),
                   guard.rejected_total(), a3_relay::RejectName(guard.last_reject()), cmd_rx_window / dt,
                   static_cast<unsigned long long>(published_total), have_state_once ? "ok-once" : "none-yet", guard.max_q_err_seen(),
                   guard.max_q_err_joint(), guard.max_q_step_seen(), guard.max_q_step_joint());
      cmd_rx_window = 0;
      next_status = now + static_cast<std::int64_t>(status_every_s * 1e9);
    }
  }

  std::fprintf(stderr,
               "[relay] exiting: frames received %llu (bad %llu), accepted %u, refused %u, published to robot %llu; "
               "largest accepted q_des-vs-measured %.3f rad (joint %d), step %.3f rad (joint %d) (SDK slots; tighten --max-q-err / --max-q-step from these)\n",
               static_cast<unsigned long long>(cmd_rx_total), static_cast<unsigned long long>(bad_frames), guard.accepted_total(),
               guard.rejected_total(), static_cast<unsigned long long>(published_total), guard.max_q_err_seen(), guard.max_q_err_joint(),
               guard.max_q_step_seen(), guard.max_q_step_joint());
  backend.Stop();
  return 0;
}
