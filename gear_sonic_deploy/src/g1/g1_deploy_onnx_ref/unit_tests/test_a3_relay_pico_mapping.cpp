// The G1 operator flow on the A3 modes (include/a3_deploy/a3_pico_command_logic.hpp):
//   A+B+X+Y: robot disabled -> PD stand (hardens); robot enabled -> passive
//   A+X:     PD stand -> teleop follow; teleop follow -> policy stand; policy stand -> follow
#include "a3_deploy/a3_pico_command_logic.hpp"

#include <gtest/gtest.h>

namespace {
using namespace a3_deploy;

PicoCommand Start(bool planner) { return PicoCommand{true, false, planner}; }
PicoCommand Stop() { return PicoCommand{false, true, true}; }  // Pico manager on A+B+X+Y -> OFF
PicoCommand Initial() { return PicoCommand{false, false, false}; }

TEST(A3PicoMapping, ABXYHardensTheRobotToTheDefaultStandingPose) {
  EXPECT_EQ(DecidePicoAction(DeployMode::kIdle, false, false, Start(true)),
            PicoAction::kEnterPdStand);
  EXPECT_EQ(DecidePicoAction(DeployMode::kPassive, false, false, Start(true)),
            PicoAction::kEnterPdStand);  // re-enable after a disable / link fault
}

TEST(A3PicoMapping, AXFollowsTheOperatorOnlyFromTheStandingPose) {
  EXPECT_EQ(DecidePicoAction(DeployMode::kPdStand, true, false, Start(false)),
            PicoAction::kEnterTeleop);
  EXPECT_EQ(DecidePicoAction(DeployMode::kPdStand, false, false, Start(false)),
            PicoAction::kEnterTeleopWhenReady);  // ramp still running
  EXPECT_EQ(DecidePicoAction(DeployMode::kIdle, false, false, Start(false)),
            PicoAction::kIgnoredNotEnabled);
  EXPECT_EQ(DecidePicoAction(DeployMode::kPassive, false, false, Start(false)),
            PicoAction::kIgnoredNotEnabled);
  EXPECT_EQ(DecidePicoAction(DeployMode::kTeleop, true, false, Start(false)),
            PicoAction::kNone);  // already following
}

TEST(A3PicoMapping, AXAgainEntersPolicyStandNotOpenLoopPdStand) {
  EXPECT_EQ(DecidePicoAction(DeployMode::kTeleop, true, false, Start(true)),
            PicoAction::kEnterPolicyStand);
  EXPECT_EQ(DecidePicoAction(DeployMode::kMotion, true, false, Start(true)),
            PicoAction::kEnterPolicyStand);
  EXPECT_EQ(DecidePicoAction(DeployMode::kTeleop, true, true, Start(true)),
            PicoAction::kNone);  // already in policy stand
  EXPECT_EQ(DecidePicoAction(DeployMode::kPdStand, true, false, Start(true)),
            PicoAction::kNone);  // already open-loop standing
}

TEST(A3PicoMapping, AXFromPolicyStandResumesFollow) {
  EXPECT_EQ(DecidePicoAction(DeployMode::kTeleop, true, true, Start(false)),
            PicoAction::kEnterTeleop);
}

TEST(A3PicoMapping, ABXYAgainDisablesTheRobotFromAnyEnabledMode) {
  for (DeployMode m : {DeployMode::kPdStand, DeployMode::kTeleop, DeployMode::kMotion}) {
    EXPECT_EQ(DecidePicoAction(m, true, false, Stop()), PicoAction::kEnterPassive);
  }
  EXPECT_EQ(DecidePicoAction(DeployMode::kIdle, false, false, Stop()), PicoAction::kNone);
  EXPECT_EQ(DecidePicoAction(DeployMode::kPassive, false, false, Stop()), PicoAction::kNone);
}

TEST(A3PicoMapping, TheManagersFirstMessageDoesNothing) {
  for (DeployMode m : {DeployMode::kIdle, DeployMode::kPassive, DeployMode::kPdStand, DeployMode::kTeleop}) {
    EXPECT_EQ(DecidePicoAction(m, true, false, Initial()), PicoAction::kNone);
  }
}

TEST(A3PicoMapping, TheFullG1SequenceWalksIdleStandTeleopPolicyStandFollowPassive) {
  DeployMode mode = DeployMode::kIdle;
  bool policy_stand = false;
  auto apply = [&](PicoAction a) {
    switch (a) {
      case PicoAction::kEnterPdStand:
        mode = DeployMode::kPdStand;
        policy_stand = false;
        break;
      case PicoAction::kEnterPolicyStand:
        mode = DeployMode::kTeleop;
        policy_stand = true;
        break;
      case PicoAction::kEnterTeleop:
        mode = DeployMode::kTeleop;
        policy_stand = false;
        break;
      case PicoAction::kEnterPassive:
        mode = DeployMode::kPassive;
        policy_stand = false;
        break;
      default:
        break;
    }
  };
  apply(DecidePicoAction(mode, false, policy_stand, Start(true)));   // A+B+X+Y
  EXPECT_EQ(mode, DeployMode::kPdStand);
  EXPECT_FALSE(policy_stand);
  apply(DecidePicoAction(mode, true, policy_stand, Start(false)));   // A+X follow
  EXPECT_EQ(mode, DeployMode::kTeleop);
  EXPECT_FALSE(policy_stand);
  apply(DecidePicoAction(mode, true, policy_stand, Start(true)));    // A+X policy stand
  EXPECT_EQ(mode, DeployMode::kTeleop);
  EXPECT_TRUE(policy_stand);
  apply(DecidePicoAction(mode, true, policy_stand, Start(false)));   // A+X follow again
  EXPECT_EQ(mode, DeployMode::kTeleop);
  EXPECT_FALSE(policy_stand);
  apply(DecidePicoAction(mode, true, policy_stand, Stop()));         // A+B+X+Y
  EXPECT_EQ(mode, DeployMode::kPassive);
}

}  // namespace
