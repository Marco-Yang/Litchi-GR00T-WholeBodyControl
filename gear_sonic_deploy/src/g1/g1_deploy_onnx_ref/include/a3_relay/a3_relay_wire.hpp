// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Wire format between the A3 ZMQ relay (runs on the robot's MDU next to body-drive, iceoryx side) and the
// deploy process on the operator PC (A3ZmqBackend).  Header-only, no dependencies, little-endian only.
//
//   relay --state frame (PUB, bind)--> PC          A3ST, 31-DOF state + both IMUs + relay mode
//   PC    --command frame (PUB)------> relay (SUB, bind)   A3CM, 31-DOF PD command
//
// Both frames are fixed-size.  Anything that is not exactly the expected size / magic / version / dof is dropped by the receiver;
// value checks (finite, gain and step limits) are the relay guard's job (a3_relay_guard.hpp).  The same layout is mirrored by
// scripts/a3_relay_tool.py (pack / unpack + a fake PC), and unit_tests/test_a3_relay_wire.cpp checks both against a golden file.
#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace a3_relay {

static_assert(std::endian::native == std::endian::little, "the A3 relay wire format is little-endian only");

constexpr int kDof = 31;  // robot_io::kA3Dof (waist 3, neck 2, arms 14, legs 12)
constexpr std::uint16_t kVersion = 1;
constexpr char kCmdMagic[4] = {'A', '3', 'C', 'M'};
constexpr char kStateMagic[4] = {'A', '3', 'S', 'T'};

constexpr int kNeckStart = 3;  // SDK slots [3..4] are the head / neck (robot_io::kA3NeckStart); the policy does not drive them
constexpr int kNeckCount = 2;

using Vec = std::array<double, kDof>;

// What the relay is doing with the commands it gets.
enum class Mode : std::uint8_t {
  kDisarmed = 0,    // nothing forwarded yet (or listen-only): the relay publishes no command
  kActive = 1,      // forwarding the PC's commands
  kHold = 2,        // the PC went quiet: re-sending the last command (velocity 0) for a short while
  kDamp = 3,        // failsafe latched: gains ramped to pure damping (kp 0)
};

// Why the relay refused the last command frame.
enum class Reject : std::uint8_t {
  kNone = 0,
  kBadFrame = 1,       // size / magic / version / dof mismatch
  kBadValue = 2,       // NaN / inf
  kLimit = 3,          // gain, torque, velocity, or q_des-vs-measured / step limit
  kSeq = 4,            // sequence number not newer than the last accepted one
  kOtherSession = 5,   // another deploy session owns the relay
  kLatched = 6,        // damping failsafe is latched; restart the deploy (new session) to re-arm
  kArmFar = 7,         // arming frame is too far from the measured pose
  kPeer = 8,           // sender address is not the allowed peer
  kListenOnly = 9,     // relay was started without command forwarding (--listen-only)
  kNoState = 10,       // no valid measured state at the relay, cannot validate
};

// ---------------------------------------------------------------------------
// Command frame (PC -> relay), 32 + 5 * 31 * 8 = 1272 bytes
//   0  char[4]  "A3CM"
//   4  u16      version
//   6  u16      dof
//   8  u64      session id (random per deploy process; the relay locks to the first one)
//   16 u32      seq (strictly increasing within a session; wrap-around safe)
//   20 u32      flags (reserved, 0)
//   24 i64      sender wall clock, ns (informational only: the relay never trusts it)
//   32 f64[31]  q_des, then dq_des, tau_ff, kp, kd
// ---------------------------------------------------------------------------
struct Command {
  std::uint64_t session = 0;
  std::uint32_t seq = 0;
  std::uint32_t flags = 0;
  std::int64_t send_ns = 0;
  Vec q_des{}, dq_des{}, tau_ff{}, kp{}, kd{};
};
constexpr std::size_t kCmdHeaderBytes = 32;
constexpr std::size_t kCmdBytes = kCmdHeaderBytes + 5 * kDof * sizeof(double);

// ---------------------------------------------------------------------------
// State frame (relay -> PC), 72 + 3 * 31 * 8 + 20 * 8 = 976 bytes
//   0  char[4]  "A3ST"
//   4  u16      version
//   6  u16      dof
//   8  u64      relay id (random per relay process)
//   16 u32      seq
//   20 u8       mode (Mode)
//   21 u8       last reject (Reject)
//   22 u16      flags: bit0 state valid, bit1 has secondary IMU, bit2 command forwarding enabled (not listen-only)
//   24 u64      owner session (0 = none)
//   32 u32      last accepted command seq
//   36 u32      accepted commands total
//   40 u32      rejected commands total
//   44 u32      reserved
//   48 i64      state age at send, ns   (relay now - oldest header stamp of the six channels)
//   56 i64      data age at send, ns    (relay now - newest receive time of the six channels)
//   64 i64      relay wall clock, ns    (informational)
//   72 f64[31]  q, dq, tau_est
//   .. f64[10]  pelvis IMU: quat wxyz, gyro, accel
//   .. f64[10]  torso IMU:  quat wxyz, gyro, accel
// ---------------------------------------------------------------------------
constexpr std::uint16_t kFlagStateValid = 1u << 0;
constexpr std::uint16_t kFlagSecondaryImu = 1u << 1;
constexpr std::uint16_t kFlagForwarding = 1u << 2;

struct ImuFrame {
  std::array<double, 4> quat_wxyz{1, 0, 0, 0};
  std::array<double, 3> gyro{0, 0, 0};
  std::array<double, 3> accel{0, 0, 9.81};
};

struct StateFrame {
  std::uint64_t relay_id = 0;
  std::uint32_t seq = 0;
  Mode mode = Mode::kDisarmed;
  Reject last_reject = Reject::kNone;
  std::uint16_t flags = 0;
  std::uint64_t owner_session = 0;
  std::uint32_t last_accepted_seq = 0;
  std::uint32_t accepted_total = 0;
  std::uint32_t rejected_total = 0;
  std::int64_t state_age_ns = 0;
  std::int64_t data_age_ns = 0;
  std::int64_t relay_ns = 0;
  Vec q{}, dq{}, tau{};
  ImuFrame pelvis{}, torso{};

  bool state_valid() const { return (flags & kFlagStateValid) != 0; }
  bool has_secondary_imu() const { return (flags & kFlagSecondaryImu) != 0; }
  bool forwarding() const { return (flags & kFlagForwarding) != 0; }
};
constexpr std::size_t kStateHeaderBytes = 72;
constexpr std::size_t kImuDoubles = 10;
constexpr std::size_t kStateBytes = kStateHeaderBytes + 3 * kDof * sizeof(double) + 2 * kImuDoubles * sizeof(double);

enum class ParseResult { kOk, kSize, kMagic, kVersion, kDof };

namespace detail {
template <typename T>
inline void Put(std::uint8_t*& p, T v) {
  std::memcpy(p, &v, sizeof(T));
  p += sizeof(T);
}
template <typename T>
inline T Get(const std::uint8_t*& p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  p += sizeof(T);
  return v;
}
inline void PutVec(std::uint8_t*& p, const Vec& v) {
  std::memcpy(p, v.data(), kDof * sizeof(double));
  p += kDof * sizeof(double);
}
inline void GetVec(const std::uint8_t*& p, Vec& v) {
  std::memcpy(v.data(), p, kDof * sizeof(double));
  p += kDof * sizeof(double);
}
inline void PutImu(std::uint8_t*& p, const ImuFrame& f) {
  std::memcpy(p, f.quat_wxyz.data(), 4 * sizeof(double)); p += 4 * sizeof(double);
  std::memcpy(p, f.gyro.data(), 3 * sizeof(double)); p += 3 * sizeof(double);
  std::memcpy(p, f.accel.data(), 3 * sizeof(double)); p += 3 * sizeof(double);
}
inline void GetImu(const std::uint8_t*& p, ImuFrame& f) {
  std::memcpy(f.quat_wxyz.data(), p, 4 * sizeof(double)); p += 4 * sizeof(double);
  std::memcpy(f.gyro.data(), p, 3 * sizeof(double)); p += 3 * sizeof(double);
  std::memcpy(f.accel.data(), p, 3 * sizeof(double)); p += 3 * sizeof(double);
}
}  // namespace detail

// Writes exactly kCmdBytes into `out`.
inline void PackCommand(const Command& c, std::uint8_t* out) {
  std::uint8_t* p = out;
  std::memcpy(p, kCmdMagic, 4); p += 4;
  detail::Put<std::uint16_t>(p, kVersion);
  detail::Put<std::uint16_t>(p, kDof);
  detail::Put<std::uint64_t>(p, c.session);
  detail::Put<std::uint32_t>(p, c.seq);
  detail::Put<std::uint32_t>(p, c.flags);
  detail::Put<std::int64_t>(p, c.send_ns);
  detail::PutVec(p, c.q_des);
  detail::PutVec(p, c.dq_des);
  detail::PutVec(p, c.tau_ff);
  detail::PutVec(p, c.kp);
  detail::PutVec(p, c.kd);
}

inline ParseResult ParseCommand(const void* data, std::size_t size, Command& c) {
  if (size != kCmdBytes) return ParseResult::kSize;
  const auto* p = static_cast<const std::uint8_t*>(data);
  if (std::memcmp(p, kCmdMagic, 4) != 0) return ParseResult::kMagic;
  p += 4;
  if (detail::Get<std::uint16_t>(p) != kVersion) return ParseResult::kVersion;
  if (detail::Get<std::uint16_t>(p) != kDof) return ParseResult::kDof;
  c.session = detail::Get<std::uint64_t>(p);
  c.seq = detail::Get<std::uint32_t>(p);
  c.flags = detail::Get<std::uint32_t>(p);
  c.send_ns = detail::Get<std::int64_t>(p);
  detail::GetVec(p, c.q_des);
  detail::GetVec(p, c.dq_des);
  detail::GetVec(p, c.tau_ff);
  detail::GetVec(p, c.kp);
  detail::GetVec(p, c.kd);
  return ParseResult::kOk;
}

// Writes exactly kStateBytes into `out`.
inline void PackState(const StateFrame& s, std::uint8_t* out) {
  std::uint8_t* p = out;
  std::memcpy(p, kStateMagic, 4); p += 4;
  detail::Put<std::uint16_t>(p, kVersion);
  detail::Put<std::uint16_t>(p, kDof);
  detail::Put<std::uint64_t>(p, s.relay_id);
  detail::Put<std::uint32_t>(p, s.seq);
  detail::Put<std::uint8_t>(p, static_cast<std::uint8_t>(s.mode));
  detail::Put<std::uint8_t>(p, static_cast<std::uint8_t>(s.last_reject));
  detail::Put<std::uint16_t>(p, s.flags);
  detail::Put<std::uint64_t>(p, s.owner_session);
  detail::Put<std::uint32_t>(p, s.last_accepted_seq);
  detail::Put<std::uint32_t>(p, s.accepted_total);
  detail::Put<std::uint32_t>(p, s.rejected_total);
  detail::Put<std::uint32_t>(p, 0u);
  detail::Put<std::int64_t>(p, s.state_age_ns);
  detail::Put<std::int64_t>(p, s.data_age_ns);
  detail::Put<std::int64_t>(p, s.relay_ns);
  detail::PutVec(p, s.q);
  detail::PutVec(p, s.dq);
  detail::PutVec(p, s.tau);
  detail::PutImu(p, s.pelvis);
  detail::PutImu(p, s.torso);
}

inline ParseResult ParseState(const void* data, std::size_t size, StateFrame& s) {
  if (size != kStateBytes) return ParseResult::kSize;
  const auto* p = static_cast<const std::uint8_t*>(data);
  if (std::memcmp(p, kStateMagic, 4) != 0) return ParseResult::kMagic;
  p += 4;
  if (detail::Get<std::uint16_t>(p) != kVersion) return ParseResult::kVersion;
  if (detail::Get<std::uint16_t>(p) != kDof) return ParseResult::kDof;
  s.relay_id = detail::Get<std::uint64_t>(p);
  s.seq = detail::Get<std::uint32_t>(p);
  const auto mode = detail::Get<std::uint8_t>(p);
  const auto reject = detail::Get<std::uint8_t>(p);
  s.mode = mode <= static_cast<std::uint8_t>(Mode::kDamp) ? static_cast<Mode>(mode) : Mode::kDamp;  // unknown -> the safe reading
  s.last_reject = static_cast<Reject>(reject);
  s.flags = detail::Get<std::uint16_t>(p);
  s.owner_session = detail::Get<std::uint64_t>(p);
  s.last_accepted_seq = detail::Get<std::uint32_t>(p);
  s.accepted_total = detail::Get<std::uint32_t>(p);
  s.rejected_total = detail::Get<std::uint32_t>(p);
  (void)detail::Get<std::uint32_t>(p);
  s.state_age_ns = detail::Get<std::int64_t>(p);
  s.data_age_ns = detail::Get<std::int64_t>(p);
  s.relay_ns = detail::Get<std::int64_t>(p);
  detail::GetVec(p, s.q);
  detail::GetVec(p, s.dq);
  detail::GetVec(p, s.tau);
  detail::GetImu(p, s.pelvis);
  detail::GetImu(p, s.torso);
  return ParseResult::kOk;
}

// Finite check on the IEEE-754 bit pattern.  The deploy is built with -ffast-math (-ffinite-math-only), under which the optimizer is
// allowed to turn std::isfinite / std::isnan into constants; the relay's NaN / inf guard must not depend on the compile flags.
inline bool IsFiniteBits(double x) {
  std::uint64_t u;
  std::memcpy(&u, &x, sizeof(u));
  return ((u >> 52) & 0x7FFu) != 0x7FFu;  // exponent all ones = inf or NaN
}

inline bool AllFinite(const Vec& v) {
  for (double x : v) {
    if (!IsFiniteBits(x)) return false;
  }
  return true;
}

inline const char* ModeName(Mode m) {
  switch (m) {
    case Mode::kDisarmed: return "DISARMED";
    case Mode::kActive: return "ACTIVE";
    case Mode::kHold: return "HOLD";
    case Mode::kDamp: return "DAMP";
  }
  return "?";
}

inline const char* RejectName(Reject r) {
  switch (r) {
    case Reject::kNone: return "none";
    case Reject::kBadFrame: return "bad_frame";
    case Reject::kBadValue: return "bad_value";
    case Reject::kLimit: return "limit";
    case Reject::kSeq: return "seq";
    case Reject::kOtherSession: return "other_session";
    case Reject::kLatched: return "latched";
    case Reject::kArmFar: return "arm_far";
    case Reject::kPeer: return "peer";
    case Reject::kListenOnly: return "listen_only";
    case Reject::kNoState: return "no_state";
  }
  return "?";
}

}  // namespace a3_relay
