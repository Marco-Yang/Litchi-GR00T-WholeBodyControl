// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// RelayGuard: the safety logic of the A3 ZMQ relay, kept free of I/O and clocks so it can be unit-tested with a fake clock.
//
// The relay sits on the robot (MDU) between the PC's deploy process and body-drive.  The PC is not a real-time system and the
// network can drop, so the part that must keep working when the PC does not lives here, next to the actuators:
//
//   DISARMED --first valid frame, close to the measured pose--> ACTIVE
//   ACTIVE   --no valid frame for stale_ms-------------------> HOLD   (re-send the last command, velocity 0)
//   HOLD     --still nothing after hold_ms-------------------> DAMP   (latched; kp -> 0, kd -> damp_kd over ramp_ms)
//   HOLD     --valid frame of the owner session--------------> ACTIVE (a short blip heals itself)
//   DAMP     --frame of a NEW session close to the pose------> ACTIVE (restart the deploy to take over again)
//
// Frames are validated before they are forwarded; a rejected frame does not refresh the "last valid frame" time, so a PC that
// keeps sending garbage falls into HOLD / DAMP like a PC that is gone.  Nothing is clamped or rewritten: accept or reject.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include "a3_relay/a3_relay_wire.hpp"

namespace a3_relay {

struct GuardConfig {
  double stale_ms = 60.0;   // no valid frame for this long -> HOLD (3 frames of the 50 Hz policy)
  double hold_ms = 150.0;   // HOLD lasts this long, then DAMP
  double ramp_ms = 300.0;   // kp -> 0, kd -> damp_kd, tau_ff -> 0 over this long once DAMP starts
  // First frame of a session (arming): q_des must be this close to the measured pose.  The deploy's PD stand ramps from the
  // measured q, so a legitimate first frame is within a few degrees.
  double arm_max_q_err_rad = 0.35;
  // The deploy holds the head at 0 (kp 40) wherever it was left, so the two neck joints get their own, looser arming tolerance.
  double arm_max_neck_err_rad = 1.0;
  // While ACTIVE: |q_des - measured q| and |q_des - previous forwarded q_des| per joint.  ABSURDITY bounds, not safety limits (no joint
  // turns more than a revolution): they catch unit mistakes and garbage, not aggressive motion.  Measured on the deploy's own command
  // stream in MuJoCo with the robot hanging in the band, teleop on (plan 18.13): q_des vs measured up to 4.8 rad, single-frame steps
  // up to 4.7 rad (waist pitch -1.8 rad, hip pitch -1.78 rad: the policy flails off the ground); the direct deploy sends the very
  // same commands.  Tighten them from the maxima the relay prints once real-robot numbers exist.
  double max_q_err_rad = 7.0;
  double max_q_step_rad = 6.0;
  double max_abs_dq_des = 5.0;
  double max_abs_tau_ff = 1.0;
  Vec kp_max{}, kd_max{};   // per joint, SDK order
  Vec damp_kd{};            // per joint kd used by the DAMP state
};

class RelayGuard {
 public:
  struct Measured {
    bool valid = false;
    Vec q{};
  };
  struct Output {
    Vec q_des{}, dq_des{}, tau_ff{}, kp{}, kd{};
  };
  // Which limit a refused frame hit (for the log: "q_step joint 12 = 0.93 > 0.8").
  struct Detail {
    const char* what = "";
    int joint = -1;
    double value = 0.0;
    double limit = 0.0;
    double q_des = 0.0;       // this frame's target for `joint`
    double q_prev = 0.0;      // the previous forwarded target
    double q_meas = 0.0;      // measured position
  };
  struct Decision {
    Reject reject = Reject::kNone;
    bool publish = false;
    Output out{};
    Detail detail{};
  };

  explicit RelayGuard(GuardConfig cfg) : cfg_(cfg) {}

  Decision OnCommand(const Command& c, const Measured& m, std::int64_t now_ns) {
    Advance(now_ns);
    Decision d;
    auto fail = [&](Reject r) {
      last_reject_ = r;
      ++rejected_total_;
      d.reject = r;
      last_detail_ = d.detail;
      return d;
    };
    auto fail_limit = [&](const char* what, int joint, double value, double limit) {
      d.detail = Detail{what, joint, value, limit};
      if (joint >= 0 && joint < kDof) {
        d.detail.q_des = c.q_des[joint];
        d.detail.q_prev = last_out_.q_des[joint];
        d.detail.q_meas = m.valid ? m.q[joint] : 0.0;
      }
      return fail(Reject::kLimit);
    };

    if (c.session == 0) return fail(Reject::kBadValue);
    if (!AllFinite(c.q_des) || !AllFinite(c.dq_des) || !AllFinite(c.tau_ff) || !AllFinite(c.kp) || !AllFinite(c.kd)) {
      return fail(Reject::kBadValue);
    }
    for (int i = 0; i < kDof; ++i) {
      if (c.kp[i] < 0.0 || c.kp[i] > cfg_.kp_max[i]) return fail_limit("kp", i, c.kp[i], cfg_.kp_max[i]);
      if (c.kd[i] < 0.0 || c.kd[i] > cfg_.kd_max[i]) return fail_limit("kd", i, c.kd[i], cfg_.kd_max[i]);
      if (std::fabs(c.tau_ff[i]) > cfg_.max_abs_tau_ff) return fail_limit("tau_ff", i, c.tau_ff[i], cfg_.max_abs_tau_ff);
      if (std::fabs(c.dq_des[i]) > cfg_.max_abs_dq_des) return fail_limit("dq_des", i, c.dq_des[i], cfg_.max_abs_dq_des);
    }

    bool arming = false;
    if (mode_ == Mode::kDisarmed) {
      arming = true;
    } else if (mode_ == Mode::kDamp) {
      if (c.session == owner_) return fail(Reject::kLatched);
      arming = true;  // a restarted deploy (new session) may take over
    } else {
      if (c.session != owner_) return fail(Reject::kOtherSession);
      if (static_cast<std::int32_t>(c.seq - last_seq_) <= 0) return fail(Reject::kSeq);
    }
    if (!m.valid) return fail(Reject::kNoState);

    double err = 0.0;
    for (int i = 0; i < kDof; ++i) err = std::max(err, std::fabs(c.q_des[i] - m.q[i]));
    double step = 0.0;
    if (arming) {
      for (int i = 0; i < kDof; ++i) {
        const bool neck = i >= kNeckStart && i < kNeckStart + kNeckCount;
        const double tol = neck ? cfg_.arm_max_neck_err_rad : cfg_.arm_max_q_err_rad;
        if (std::fabs(c.q_des[i] - m.q[i]) > tol) {
          d.detail = Detail{"arm_q_err", i, std::fabs(c.q_des[i] - m.q[i]), tol};
          return fail(Reject::kArmFar);
        }
      }
    } else {
      int err_joint = 0, step_joint = 0;
      for (int i = 0; i < kDof; ++i) {
        const double e = std::fabs(c.q_des[i] - m.q[i]);
        const double s = std::fabs(c.q_des[i] - last_out_.q_des[i]);
        if (e >= err) { err = e; err_joint = i; }
        if (s >= step) { step = s; step_joint = i; }
      }
      if (err > cfg_.max_q_err_rad) return fail_limit("q_err", err_joint, err, cfg_.max_q_err_rad);
      if (step > cfg_.max_q_step_rad) return fail_limit("q_step", step_joint, step, cfg_.max_q_step_rad);
      if (err > max_q_err_seen_) { max_q_err_seen_ = err; max_q_err_joint_ = err_joint; }
      if (step > max_q_step_seen_) {
        max_q_step_seen_ = step;
        max_q_step_joint_ = step_joint;
        max_step_detail_ = Detail{"q_step", step_joint, step, cfg_.max_q_step_rad, c.q_des[step_joint], last_out_.q_des[step_joint],
                                  m.q[step_joint]};
        max_step_fresh_ = true;
      }
    }

    owner_ = c.session;
    last_seq_ = c.seq;
    last_valid_ns_ = now_ns;
    last_out_.q_des = c.q_des;
    last_out_.dq_des = c.dq_des;
    last_out_.tau_ff = c.tau_ff;
    last_out_.kp = c.kp;
    last_out_.kd = c.kd;
    mode_ = Mode::kActive;
    last_reject_ = Reject::kNone;
    ++accepted_total_;
    d.publish = true;
    d.out = last_out_;
    return d;
  }

  // Call periodically (the relay's resend rate).  ACTIVE / DISARMED -> nothing to send; HOLD / DAMP -> the command to publish.
  std::optional<Output> Tick(std::int64_t now_ns) {
    Advance(now_ns);
    switch (mode_) {
      case Mode::kDisarmed:
      case Mode::kActive:
        return std::nullopt;
      case Mode::kHold: {
        Output o = last_out_;
        o.dq_des.fill(0.0);
        return o;
      }
      case Mode::kDamp:
        return DampOutput(now_ns);
    }
    return std::nullopt;
  }

  // Applies the time-based transitions (ACTIVE -> HOLD -> DAMP) and returns the resulting mode.
  Mode Update(std::int64_t now_ns) {
    Advance(now_ns);
    return mode_;
  }

  // Relay is shutting down (SIGINT / SIGTERM): go straight to damping from the last command.
  void BeginShutdownDamp(std::int64_t now_ns) {
    Advance(now_ns);
    if (mode_ == Mode::kActive || mode_ == Mode::kHold) {
      mode_ = Mode::kDamp;
      damp_start_ns_ = now_ns;
    }
  }

  Mode mode() const { return mode_; }
  std::uint64_t owner_session() const { return owner_; }
  std::uint32_t last_accepted_seq() const { return last_seq_; }
  std::uint32_t accepted_total() const { return accepted_total_; }
  std::uint32_t rejected_total() const { return rejected_total_; }
  Reject last_reject() const { return last_reject_; }
  const Detail& last_detail() const { return last_detail_; }
  // The largest accepted step so far, once, when it has just grown (the relay logs it: "joint 25 went from -0.5 to 2.1").
  bool TakeNewMaxStep(Detail& out) {
    if (!max_step_fresh_) return false;
    max_step_fresh_ = false;
    out = max_step_detail_;
    return true;
  }
  double max_q_err_seen() const { return max_q_err_seen_; }
  double max_q_step_seen() const { return max_q_step_seen_; }
  int max_q_err_joint() const { return max_q_err_joint_; }    // SDK slot of the joint behind max_q_err_seen()
  int max_q_step_joint() const { return max_q_step_joint_; }
  const GuardConfig& config() const { return cfg_; }

  // Counts a frame the relay refused before the guard saw it (bad size / magic, wrong peer, listen-only).
  void NoteExternalReject(Reject r) {
    last_reject_ = r;
    ++rejected_total_;
  }

 private:
  static std::int64_t Ms(double ms) { return static_cast<std::int64_t>(ms * 1e6); }

  void Advance(std::int64_t now_ns) {
    if (mode_ == Mode::kActive && now_ns - last_valid_ns_ > Ms(cfg_.stale_ms)) mode_ = Mode::kHold;
    if (mode_ == Mode::kHold && now_ns - last_valid_ns_ > Ms(cfg_.stale_ms + cfg_.hold_ms)) {
      mode_ = Mode::kDamp;
      damp_start_ns_ = last_valid_ns_ + Ms(cfg_.stale_ms + cfg_.hold_ms);
    }
  }

  Output DampOutput(std::int64_t now_ns) const {
    const double ramp_ns = cfg_.ramp_ms * 1e6;
    const double s = ramp_ns <= 0.0 ? 1.0 : std::clamp(static_cast<double>(now_ns - damp_start_ns_) / ramp_ns, 0.0, 1.0);
    Output o;
    o.q_des = last_out_.q_des;
    o.dq_des.fill(0.0);
    for (int i = 0; i < kDof; ++i) {
      o.tau_ff[i] = last_out_.tau_ff[i] * (1.0 - s);
      o.kp[i] = last_out_.kp[i] * (1.0 - s);
      o.kd[i] = last_out_.kd[i] + (cfg_.damp_kd[i] - last_out_.kd[i]) * s;
    }
    return o;
  }

  GuardConfig cfg_;
  Mode mode_ = Mode::kDisarmed;
  std::uint64_t owner_ = 0;
  std::uint32_t last_seq_ = 0;
  std::int64_t last_valid_ns_ = 0;
  std::int64_t damp_start_ns_ = 0;
  Output last_out_{};
  Reject last_reject_ = Reject::kNone;
  Detail last_detail_{};
  Detail max_step_detail_{};
  bool max_step_fresh_ = false;
  std::uint32_t accepted_total_ = 0;
  std::uint32_t rejected_total_ = 0;
  double max_q_err_seen_ = 0.0;
  double max_q_step_seen_ = 0.0;
  int max_q_err_joint_ = -1;
  int max_q_step_joint_ = -1;
};

}  // namespace a3_relay
