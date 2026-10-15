// RelayGuard: arming, forwarding, validation, and the hold -> damp failsafe, driven by a fake clock.
#include "a3_relay/a3_relay_guard.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace {
using namespace a3_relay;
using Measured = RelayGuard::Measured;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::uint64_t kSessionA = 0xA11CE;
constexpr std::uint64_t kSessionB = 0xB0B;

GuardConfig Config() {
  GuardConfig cfg;  // stale 60 ms, hold 150 ms, ramp 300 ms
  cfg.max_q_err_rad = 2.0;   // the tests exercise the mechanism with tight limits; the shipped defaults are checked separately
  cfg.max_q_step_rad = 0.8;
  cfg.kp_max.fill(200.0);
  cfg.kd_max.fill(10.0);
  cfg.damp_kd.fill(2.0);
  return cfg;
}

Measured Pose(double q = 0.0) {
  Measured m;
  m.valid = true;
  m.q.fill(q);
  return m;
}

Command Cmd(std::uint64_t session, std::uint32_t seq, double q_des = 0.0) {
  Command c;
  c.session = session;
  c.seq = seq;
  c.q_des.fill(q_des);
  c.kp.fill(50.0);
  c.kd.fill(4.0);
  return c;
}

TEST(A3RelayGuard, ShippedPositionLimitsAreAbsurdityBoundsAboveWhatTheDeployMeasurablySends) {
  // MuJoCo, robot hanging in the band with teleop on (plan 18.13): q_des vs measured up to 4.8 rad, one-frame steps up to 4.7 rad
  const GuardConfig d;
  EXPECT_GT(d.max_q_err_rad, 4.9);
  EXPECT_GT(d.max_q_step_rad, 4.8);
  // ...but still refuse values no joint can have: about one full revolution (2 pi = 6.28 rad) or more
  EXPECT_LT(d.max_q_err_rad, 7.5);
  EXPECT_LT(d.max_q_step_rad, 6.5);
}

TEST(A3RelayGuard, StartsDisarmedAndPublishesNothing) {
  RelayGuard g(Config());
  EXPECT_EQ(g.mode(), Mode::kDisarmed);
  EXPECT_FALSE(g.Tick(1000 * kMs).has_value());
}

TEST(A3RelayGuard, FirstFrameNearThePoseArmsAndIsForwardedUnchanged) {
  RelayGuard g(Config());
  const auto d = g.OnCommand(Cmd(kSessionA, 1, 0.1), Pose(0.0), 0);
  EXPECT_EQ(d.reject, Reject::kNone);
  ASSERT_TRUE(d.publish);
  EXPECT_EQ(g.mode(), Mode::kActive);
  EXPECT_EQ(g.owner_session(), kSessionA);
  EXPECT_DOUBLE_EQ(d.out.q_des[7], 0.1);
  EXPECT_DOUBLE_EQ(d.out.kp[7], 50.0);
  EXPECT_DOUBLE_EQ(d.out.kd[7], 4.0);
}

TEST(A3RelayGuard, ArmingFarFromTheMeasuredPoseIsRefused) {
  RelayGuard g(Config());
  const auto d = g.OnCommand(Cmd(kSessionA, 1, 0.6), Pose(0.0), 0);
  EXPECT_EQ(d.reject, Reject::kArmFar);
  EXPECT_FALSE(d.publish);
  EXPECT_EQ(g.mode(), Mode::kDisarmed);
  EXPECT_EQ(g.rejected_total(), 1u);
  EXPECT_EQ(g.last_reject(), Reject::kArmFar);
}

TEST(A3RelayGuard, ArmingToleratesAHeadFarFromZeroButNotABodyJoint) {
  // the deploy holds the head at 0 wherever it was left (neck slots 3 and 4); 0.8 rad away is fine for them, not for an arm
  RelayGuard g(Config());
  auto c = Cmd(kSessionA, 1, 0.0);
  Measured m = Pose(0.0);
  m.q[3] = 0.8;
  m.q[4] = -0.8;
  EXPECT_TRUE(g.OnCommand(c, m, 0).publish);
  RelayGuard g2(Config());
  m.q[5] = 0.5;  // an arm joint 0.5 rad off
  EXPECT_EQ(g2.OnCommand(c, m, 0).reject, Reject::kArmFar);
  RelayGuard g3(Config());
  Measured far_head = Pose(0.0);
  far_head.q[3] = 1.2;  // beyond even the neck tolerance
  EXPECT_EQ(g3.OnCommand(c, far_head, 0).reject, Reject::kArmFar);
}

TEST(A3RelayGuard, NoMeasuredStateMeansNothingIsForwarded) {
  RelayGuard g(Config());
  Measured none;
  EXPECT_EQ(g.OnCommand(Cmd(kSessionA, 1), none, 0).reject, Reject::kNoState);
  EXPECT_EQ(g.mode(), Mode::kDisarmed);
}

TEST(A3RelayGuard, RejectsNonFiniteValuesAndOutOfLimitGains) {
  RelayGuard g(Config());
  auto c = Cmd(kSessionA, 1);
  c.q_des[3] = std::nan("");
  EXPECT_EQ(g.OnCommand(c, Pose(), 0).reject, Reject::kBadValue);  // the build uses -ffast-math: the check must not rely on std::isfinite
  c = Cmd(kSessionA, 1);
  c.kp[5] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(g.OnCommand(c, Pose(), 0).reject, Reject::kBadValue);
  c = Cmd(kSessionA, 1);
  c.tau_ff[30] = -std::numeric_limits<double>::infinity();
  EXPECT_EQ(g.OnCommand(c, Pose(), 0).reject, Reject::kBadValue);
  c = Cmd(kSessionA, 1);
  c.kp[0] = 201.0;
  EXPECT_EQ(g.OnCommand(c, Pose(), 0).reject, Reject::kLimit);
  c = Cmd(kSessionA, 1);
  c.kd[0] = -0.1;
  EXPECT_EQ(g.OnCommand(c, Pose(), 0).reject, Reject::kLimit);
  c = Cmd(kSessionA, 1);
  c.tau_ff[0] = 1.5;
  EXPECT_EQ(g.OnCommand(c, Pose(), 0).reject, Reject::kLimit);
  c = Cmd(kSessionA, 1);
  c.dq_des[0] = -6.0;
  EXPECT_EQ(g.OnCommand(c, Pose(), 0).reject, Reject::kLimit);
  EXPECT_EQ(g.OnCommand(Cmd(0, 1), Pose(), 0).reject, Reject::kBadValue);  // session 0 is "none"
  EXPECT_EQ(g.mode(), Mode::kDisarmed);
}

TEST(A3RelayGuard, OnlyTheOwnerSessionWithGrowingSeqIsForwarded) {
  RelayGuard g(Config());
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 10), Pose(), 0).publish);
  EXPECT_EQ(g.OnCommand(Cmd(kSessionB, 11), Pose(), 5 * kMs).reject, Reject::kOtherSession);
  EXPECT_EQ(g.OnCommand(Cmd(kSessionA, 10), Pose(), 6 * kMs).reject, Reject::kSeq);  // duplicate
  EXPECT_EQ(g.OnCommand(Cmd(kSessionA, 9), Pose(), 7 * kMs).reject, Reject::kSeq);   // old
  EXPECT_TRUE(g.OnCommand(Cmd(kSessionA, 11), Pose(), 8 * kMs).publish);
}

TEST(A3RelayGuard, SeqWrapAroundIsAccepted) {
  RelayGuard g(Config());
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 0xFFFFFFFEu), Pose(), 0).publish);
  EXPECT_TRUE(g.OnCommand(Cmd(kSessionA, 0xFFFFFFFFu), Pose(), 10 * kMs).publish);
  EXPECT_TRUE(g.OnCommand(Cmd(kSessionA, 0u), Pose(), 20 * kMs).publish);
}

TEST(A3RelayGuard, ActiveLimitsOnMeasuredErrorAndStep) {
  RelayGuard g(Config());
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 1, 0.0), Pose(0.0), 0).publish);
  // a step of 0.9 rad from the previous target is more than max_q_step (0.8)
  EXPECT_EQ(g.OnCommand(Cmd(kSessionA, 2, 0.9), Pose(0.9), 20 * kMs).reject, Reject::kLimit);
  // a target 2.5 rad away from the measured pose is more than max_q_err (2.0), even in small steps
  RelayGuard g2(Config());
  ASSERT_TRUE(g2.OnCommand(Cmd(kSessionA, 1, 0.0), Pose(0.0), 0).publish);
  double t = 0.0;
  std::uint32_t seq = 1;
  std::int64_t now = 0;
  Reject last = Reject::kNone;
  for (int i = 0; i < 10 && last == Reject::kNone; ++i) {
    t += 0.5;
    now += 20 * kMs;
    last = g2.OnCommand(Cmd(kSessionA, ++seq, t), Pose(0.0), now).reject;
  }
  EXPECT_EQ(last, Reject::kLimit);
  EXPECT_GT(g2.max_q_err_seen(), 1.0);
}

TEST(A3RelayGuard, ARefusedFrameSaysWhichLimitAndWhichJointItHit) {
  RelayGuard g(Config());
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 1, 0.0), Pose(0.0), 0).publish);
  auto c = Cmd(kSessionA, 2, 0.0);
  c.q_des[12] = 0.95;  // 0.95 rad away from the previous target and from the measured pose (limit 0.8 for the step)
  auto d = g.OnCommand(c, Pose(0.0), 20 * kMs);
  EXPECT_EQ(d.reject, Reject::kLimit);
  EXPECT_STREQ(d.detail.what, "q_step");
  EXPECT_EQ(d.detail.joint, 12);
  EXPECT_NEAR(d.detail.value, 0.95, 1e-9);
  EXPECT_NEAR(d.detail.limit, 0.8, 1e-9);
  c = Cmd(kSessionA, 3, 0.0);
  c.kd[7] = 99.0;
  d = g.OnCommand(c, Pose(0.0), 40 * kMs);
  EXPECT_STREQ(d.detail.what, "kd");
  EXPECT_EQ(d.detail.joint, 7);
  // the accepted maxima remember the joint that produced them
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 4, 0.3), Pose(0.0), 60 * kMs).publish);
  EXPECT_NEAR(g.max_q_step_seen(), 0.3, 1e-9);
  EXPECT_GE(g.max_q_step_joint(), 0);
}

TEST(A3RelayGuard, StaysQuietWhileFramesKeepComing) {
  RelayGuard g(Config());
  std::uint32_t seq = 0;
  for (std::int64_t t = 0; t <= 1000 * kMs; t += 20 * kMs) {
    ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, ++seq), Pose(), t).publish);
    EXPECT_FALSE(g.Tick(t + 10 * kMs).has_value());
  }
  EXPECT_EQ(g.mode(), Mode::kActive);
}

TEST(A3RelayGuard, SilenceHoldsThenDampsAndLatches) {
  RelayGuard g(Config());
  auto c = Cmd(kSessionA, 1, 0.2);
  c.dq_des.fill(1.5);
  c.tau_ff.fill(0.5);
  ASSERT_TRUE(g.OnCommand(c, Pose(0.2), 0).publish);

  // within stale_ms (60): nothing to re-send, still ACTIVE
  EXPECT_FALSE(g.Tick(50 * kMs).has_value());
  EXPECT_EQ(g.mode(), Mode::kActive);

  // stale: HOLD re-sends the last command with velocity 0 and the commanded gains
  auto h = g.Tick(70 * kMs);
  ASSERT_TRUE(h.has_value());
  EXPECT_EQ(g.mode(), Mode::kHold);
  EXPECT_DOUBLE_EQ(h->q_des[0], 0.2);
  EXPECT_DOUBLE_EQ(h->dq_des[0], 0.0);
  EXPECT_DOUBLE_EQ(h->tau_ff[0], 0.5);
  EXPECT_DOUBLE_EQ(h->kp[0], 50.0);
  EXPECT_DOUBLE_EQ(h->kd[0], 4.0);
  EXPECT_EQ(g.mode(), Mode::kHold);

  // end of HOLD at 60 + 150 = 210 ms: DAMP starts at s = 0 (gains unchanged), then ramps over 300 ms
  auto d0 = g.Tick(210 * kMs + 1);
  ASSERT_TRUE(d0.has_value());
  EXPECT_EQ(g.mode(), Mode::kDamp);
  EXPECT_NEAR(d0->kp[0], 50.0, 1e-3);
  EXPECT_NEAR(d0->kd[0], 4.0, 1e-3);
  auto dmid = g.Tick(210 * kMs + 150 * kMs);  // s = 0.5
  ASSERT_TRUE(dmid.has_value());
  EXPECT_NEAR(dmid->kp[0], 25.0, 1e-6);
  EXPECT_NEAR(dmid->kd[0], 3.0, 1e-6);  // 4 -> damp_kd 2
  EXPECT_NEAR(dmid->tau_ff[0], 0.25, 1e-6);
  EXPECT_DOUBLE_EQ(dmid->q_des[0], 0.2);  // the target does not move
  auto dend = g.Tick(210 * kMs + 400 * kMs);
  ASSERT_TRUE(dend.has_value());
  EXPECT_DOUBLE_EQ(dend->kp[0], 0.0);
  EXPECT_DOUBLE_EQ(dend->kd[0], 2.0);
  EXPECT_DOUBLE_EQ(dend->tau_ff[0], 0.0);
  EXPECT_DOUBLE_EQ(dend->dq_des[0], 0.0);

  // latched: the old session cannot resume, even with a perfect frame
  EXPECT_EQ(g.OnCommand(Cmd(kSessionA, 2, 0.2), Pose(0.2), 700 * kMs).reject, Reject::kLatched);
  EXPECT_EQ(g.mode(), Mode::kDamp);
}

TEST(A3RelayGuard, ABriefBlipHealsInsideTheHoldWindow) {
  RelayGuard g(Config());
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 1), Pose(), 0).publish);
  ASSERT_TRUE(g.Tick(100 * kMs).has_value());  // HOLD
  EXPECT_EQ(g.mode(), Mode::kHold);
  const auto d = g.OnCommand(Cmd(kSessionA, 2), Pose(), 150 * kMs);
  EXPECT_TRUE(d.publish);
  EXPECT_EQ(g.mode(), Mode::kActive);
  EXPECT_FALSE(g.Tick(160 * kMs).has_value());
}

TEST(A3RelayGuard, RejectedFramesDoNotKeepTheLinkAlive) {
  RelayGuard g(Config());
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 1), Pose(), 0).publish);
  std::uint32_t seq = 1;
  for (std::int64_t t = 20 * kMs; t < 400 * kMs; t += 20 * kMs) {
    auto bad = Cmd(kSessionA, ++seq);
    bad.kp[2] = 1e6;  // over the gain limit
    EXPECT_EQ(g.OnCommand(bad, Pose(), t).reject, Reject::kLimit);
  }
  EXPECT_TRUE(g.Tick(400 * kMs).has_value());
  EXPECT_EQ(g.mode(), Mode::kDamp);
}

TEST(A3RelayGuard, ANewSessionMayTakeOverAfterDampButOnlyNearThePose) {
  RelayGuard g(Config());
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 1, 0.0), Pose(0.0), 0).publish);
  ASSERT_TRUE(g.Tick(900 * kMs).has_value());
  ASSERT_EQ(g.mode(), Mode::kDamp);
  // the damped robot sagged to 0.5 rad: a restarted deploy that wants 0.0 is too far from the measured pose
  EXPECT_EQ(g.OnCommand(Cmd(kSessionB, 1, 0.0), Pose(0.5), 1000 * kMs).reject, Reject::kArmFar);
  EXPECT_EQ(g.mode(), Mode::kDamp);
  const auto d = g.OnCommand(Cmd(kSessionB, 1, 0.45), Pose(0.5), 1020 * kMs);
  EXPECT_TRUE(d.publish);
  EXPECT_EQ(g.mode(), Mode::kActive);
  EXPECT_EQ(g.owner_session(), kSessionB);
  EXPECT_EQ(g.OnCommand(Cmd(kSessionA, 50, 0.45), Pose(0.5), 1040 * kMs).reject, Reject::kOtherSession);
}

TEST(A3RelayGuard, ShutdownDampStartsFromTheLastCommandImmediately) {
  RelayGuard g(Config());
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 1, 0.1), Pose(0.1), 0).publish);
  g.BeginShutdownDamp(10 * kMs);
  EXPECT_EQ(g.mode(), Mode::kDamp);
  auto d = g.Tick(10 * kMs + 300 * kMs);
  ASSERT_TRUE(d.has_value());
  EXPECT_DOUBLE_EQ(d->kp[0], 0.0);
  EXPECT_DOUBLE_EQ(d->kd[0], 2.0);
  EXPECT_DOUBLE_EQ(d->q_des[0], 0.1);
}

TEST(A3RelayGuard, ShutdownWhileDisarmedStaysQuiet) {
  RelayGuard g(Config());
  g.BeginShutdownDamp(10 * kMs);
  EXPECT_EQ(g.mode(), Mode::kDisarmed);
  EXPECT_FALSE(g.Tick(20 * kMs).has_value());
}

TEST(A3RelayGuard, AZeroGainSafeHaltFromThePcIsForwardedAndThenDamped) {
  RelayGuard g(Config());
  ASSERT_TRUE(g.OnCommand(Cmd(kSessionA, 1), Pose(), 0).publish);
  auto halt = Cmd(kSessionA, 2);
  halt.kp.fill(0.0);
  halt.kd.fill(0.0);
  const auto d = g.OnCommand(halt, Pose(), 20 * kMs);
  ASSERT_TRUE(d.publish);  // the deploy's own passive command is not second-guessed
  EXPECT_DOUBLE_EQ(d.out.kp[0], 0.0);
  EXPECT_DOUBLE_EQ(d.out.kd[0], 0.0);
  auto end = g.Tick(20 * kMs + 600 * kMs);  // silence afterwards: the relay damps instead of leaving the robot limp
  ASSERT_TRUE(end.has_value());
  EXPECT_DOUBLE_EQ(end->kd[0], 2.0);
}

}  // namespace
