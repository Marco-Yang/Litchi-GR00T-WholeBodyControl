// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// What a Pico "command" message does to the A3 deploy: the G1 operator flow mapped onto the A3 modes.  Pure function, unit-tested
// (unit_tests/test_a3_relay_pico_mapping.cpp).
//
// The Pico manager (gear_sonic/scripts/pico_manager_thread_server.py) sends ONE message per mode switch of its own state machine:
//   A+B+X+Y  OFF -> PLANNER   start=1 stop=0 planner=1
//   A+X      PLANNER -> POSE  start=1 stop=0 planner=0
//   A+X      POSE -> PLANNER  start=1 stop=0 planner=1
//   A+B+X+Y  any -> OFF       start=0 stop=1 planner=1   (and the manager script exits)
// A3 has no locomotion planner.  G1's PLANNER keeps the policy running on a stand reference; A3 mirrors that with
// "policy stand" (still TELEOP, default-stand tokenizer through a3_fast).  Open-loop PD stand is only for the first
// harden from IDLE / PASSIVE (and the keyboard S path):
//
//   A+B+X+Y   robot disabled -> PD stand: harden to the default standing pose (Litchi's PD_STAND gains)
//   A+X       PD stand -> teleop follow
//   A+X       teleop follow -> policy stand: keep a3_fast, reference = default stand (G1 POSE -> PLANNER)
//   A+X       policy stand -> teleop follow again
//   A+B+X+Y   robot enabled -> passive: kp = kd = tau_ff = 0
#pragma once

#include "a3_deploy/a3_manual_control.hpp"

namespace a3_deploy {

struct PicoCommand {
  bool start = false;
  bool stop = false;
  bool planner = true;  // true: PLANNER (stand); false: POSE (follow)
};

enum class PicoAction {
  kNone,
  kEnterPdStand,          // IDLE / PASSIVE -> open-loop PD stand (first harden only)
  kEnterPolicyStand,      // teleop / motion -> keep policy, track default stand (G1 PLANNER)
  kEnterTeleop,           // follow the operator now (also resumes from policy stand)
  kEnterTeleopWhenReady,  // PD stand ramp still running: follow as soon as it is ready
  kEnterPassive,          // disable
  kIgnoredNotEnabled,     // A+X while the robot is disabled
};

// teleop_policy_stand: true while TELEOP is holding the default-stand reference through the policy
// (G1 PLANNER equivalent).  DecidePicoAction needs it so A+X can toggle follow <-> policy stand without
// leaving DeployMode::kTeleop.
inline PicoAction DecidePicoAction(DeployMode mode, bool pd_stand_ready, bool teleop_policy_stand,
                                   const PicoCommand& c) {
  const bool enabled = mode == DeployMode::kPdStand || mode == DeployMode::kMotion || mode == DeployMode::kTeleop;
  if (c.stop) return enabled ? PicoAction::kEnterPassive : PicoAction::kNone;
  if (!c.start) return PicoAction::kNone;
  if (c.planner) {
    if (mode == DeployMode::kPdStand) return PicoAction::kNone;
    if (mode == DeployMode::kTeleop || mode == DeployMode::kMotion) {
      return teleop_policy_stand ? PicoAction::kNone : PicoAction::kEnterPolicyStand;
    }
    return PicoAction::kEnterPdStand;  // IDLE / PASSIVE: first harden
  }
  switch (mode) {
    case DeployMode::kPdStand:
      return pd_stand_ready ? PicoAction::kEnterTeleop : PicoAction::kEnterTeleopWhenReady;
    case DeployMode::kIdle:
    case DeployMode::kPassive:
      return PicoAction::kIgnoredNotEnabled;
    case DeployMode::kTeleop:
      // Already following: ignore.  Policy stand: A+X (planner=0) resumes follow, like G1 PLANNER -> POSE.
      return teleop_policy_stand ? PicoAction::kEnterTeleop : PicoAction::kNone;
    case DeployMode::kMotion:
      return PicoAction::kEnterTeleop;
  }
  return PicoAction::kNone;
}

// Overload kept for call sites that do not track policy-stand yet (defaults to "not in policy stand").
inline PicoAction DecidePicoAction(DeployMode mode, bool pd_stand_ready, const PicoCommand& c) {
  return DecidePicoAction(mode, pd_stand_ready, /*teleop_policy_stand=*/false, c);
}

}  // namespace a3_deploy
