// Wire format of the A3 ZMQ relay: round trips, structural rejects, and byte-for-byte agreement with the Python tool
// (scripts/a3_relay_tool.py write-golden writes the golden frames this test reads).
#include "a3_relay/a3_relay_wire.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#ifndef A3_RELAY_GOLDEN_DIR
#error "A3_RELAY_GOLDEN_DIR must point at unit_tests/golden"
#endif

namespace {
using namespace a3_relay;

std::vector<std::uint8_t> ReadFile(const std::string& name) {
  std::ifstream f(std::string(A3_RELAY_GOLDEN_DIR) + "/" + name, std::ios::binary);
  EXPECT_TRUE(f.good()) << name;
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

Command GoldenCommand() {
  Command c;
  c.session = 0x1122334455667788ULL;
  c.seq = 4242;
  c.flags = 7;
  c.send_ns = 1'700'000'000'123'456'789LL;
  for (int i = 0; i < kDof; ++i) {
    c.q_des[i] = 0.01 * i;
    c.dq_des[i] = 0.1 * i;
    c.tau_ff[i] = -0.5 * i;
    c.kp[i] = 1.0 + i;
    c.kd[i] = 0.25 * (i + 1);
  }
  return c;
}

StateFrame GoldenState() {
  StateFrame s;
  s.relay_id = 0xAABBCCDDEEFF0011ULL;
  s.seq = 99;
  s.mode = Mode::kHold;
  s.last_reject = Reject::kLatched;
  s.flags = kFlagStateValid | kFlagSecondaryImu | kFlagForwarding;
  s.owner_session = 0x1122334455667788ULL;
  s.last_accepted_seq = 4242;
  s.accepted_total = 1000;
  s.rejected_total = 7;
  s.state_age_ns = 12'345'678;
  s.data_age_ns = 2'345'678;
  s.relay_ns = 1'700'000'000'987'654'321LL;
  for (int i = 0; i < kDof; ++i) {
    s.q[i] = 0.02 * i;
    s.dq[i] = 0.03 * i;
    s.tau[i] = 0.04 * i;
  }
  s.pelvis.quat_wxyz = {0.5, 0.5, 0.5, 0.5};
  s.pelvis.gyro = {0.1, 0.2, 0.3};
  s.pelvis.accel = {1.0, 2.0, 9.81};
  s.torso.quat_wxyz = {1.0, 0.0, 0.0, 0.0};
  s.torso.gyro = {-0.1, -0.2, -0.3};
  s.torso.accel = {-1.0, -2.0, 9.5};
  return s;
}

TEST(A3RelayWire, SizesMatchTheDocumentedLayout) {
  EXPECT_EQ(kCmdBytes, 1272u);
  EXPECT_EQ(kStateBytes, 976u);
}

TEST(A3RelayWire, CommandRoundTrip) {
  const Command c = GoldenCommand();
  std::vector<std::uint8_t> buf(kCmdBytes);
  PackCommand(c, buf.data());
  Command r;
  ASSERT_EQ(ParseCommand(buf.data(), buf.size(), r), ParseResult::kOk);
  EXPECT_EQ(r.session, c.session);
  EXPECT_EQ(r.seq, c.seq);
  EXPECT_EQ(r.flags, c.flags);
  EXPECT_EQ(r.send_ns, c.send_ns);
  EXPECT_EQ(r.q_des, c.q_des);
  EXPECT_EQ(r.dq_des, c.dq_des);
  EXPECT_EQ(r.tau_ff, c.tau_ff);
  EXPECT_EQ(r.kp, c.kp);
  EXPECT_EQ(r.kd, c.kd);
}

TEST(A3RelayWire, StateRoundTrip) {
  const StateFrame s = GoldenState();
  std::vector<std::uint8_t> buf(kStateBytes);
  PackState(s, buf.data());
  StateFrame r;
  ASSERT_EQ(ParseState(buf.data(), buf.size(), r), ParseResult::kOk);
  EXPECT_EQ(r.relay_id, s.relay_id);
  EXPECT_EQ(r.seq, s.seq);
  EXPECT_EQ(r.mode, Mode::kHold);
  EXPECT_EQ(r.last_reject, Reject::kLatched);
  EXPECT_TRUE(r.state_valid());
  EXPECT_TRUE(r.has_secondary_imu());
  EXPECT_TRUE(r.forwarding());
  EXPECT_EQ(r.owner_session, s.owner_session);
  EXPECT_EQ(r.last_accepted_seq, 4242u);
  EXPECT_EQ(r.accepted_total, 1000u);
  EXPECT_EQ(r.rejected_total, 7u);
  EXPECT_EQ(r.state_age_ns, s.state_age_ns);
  EXPECT_EQ(r.data_age_ns, s.data_age_ns);
  EXPECT_EQ(r.relay_ns, s.relay_ns);
  EXPECT_EQ(r.q, s.q);
  EXPECT_EQ(r.dq, s.dq);
  EXPECT_EQ(r.tau, s.tau);
  EXPECT_EQ(r.pelvis.quat_wxyz, s.pelvis.quat_wxyz);
  EXPECT_EQ(r.torso.accel, s.torso.accel);
}

TEST(A3RelayWire, MatchesPythonGoldenFramesByteForByte) {
  std::vector<std::uint8_t> cmd(kCmdBytes), st(kStateBytes);
  PackCommand(GoldenCommand(), cmd.data());
  PackState(GoldenState(), st.data());
  EXPECT_EQ(cmd, ReadFile("a3_relay_golden_cmd.bin"));
  EXPECT_EQ(st, ReadFile("a3_relay_golden_state.bin"));
}

TEST(A3RelayWire, RejectsStructuralDamage) {
  std::vector<std::uint8_t> buf(kCmdBytes);
  PackCommand(GoldenCommand(), buf.data());
  Command r;
  EXPECT_EQ(ParseCommand(buf.data(), buf.size() - 1, r), ParseResult::kSize);
  EXPECT_EQ(ParseCommand(buf.data(), buf.size() + 1, r), ParseResult::kSize);
  auto bad = buf;
  bad[0] = 'X';
  EXPECT_EQ(ParseCommand(bad.data(), bad.size(), r), ParseResult::kMagic);
  bad = buf;
  bad[4] = 2;  // version
  EXPECT_EQ(ParseCommand(bad.data(), bad.size(), r), ParseResult::kVersion);
  bad = buf;
  bad[6] = 29;  // dof
  EXPECT_EQ(ParseCommand(bad.data(), bad.size(), r), ParseResult::kDof);
  // a state frame is not a command frame, and vice versa
  std::vector<std::uint8_t> st(kStateBytes);
  PackState(GoldenState(), st.data());
  StateFrame sr;
  EXPECT_EQ(ParseCommand(st.data(), st.size(), r), ParseResult::kSize);
  EXPECT_EQ(ParseState(buf.data(), buf.size(), sr), ParseResult::kSize);
  st[1] = 'x';
  EXPECT_EQ(ParseState(st.data(), st.size(), sr), ParseResult::kMagic);
}

TEST(A3RelayWire, UnknownModeReadsAsDamp) {
  std::vector<std::uint8_t> st(kStateBytes);
  PackState(GoldenState(), st.data());
  st[20] = 200;  // mode byte
  StateFrame r;
  ASSERT_EQ(ParseState(st.data(), st.size(), r), ParseResult::kOk);
  EXPECT_EQ(r.mode, Mode::kDamp);
}

}  // namespace
