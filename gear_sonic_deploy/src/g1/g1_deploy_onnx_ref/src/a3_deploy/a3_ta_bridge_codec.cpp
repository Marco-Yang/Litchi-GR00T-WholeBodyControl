// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "a3_deploy/a3_ta_bridge_codec.hpp"

#include <cctype>
#include <cmath>
#include <cstring>

namespace a3_deploy {
namespace {

// Packed little-endian reader/writer (x86-64 and aarch64 targets are little-endian).
class Reader {
 public:
  Reader(const unsigned char* p, std::size_t n) : p_(p), n_(n) {}
  template <typename T>
  T Get() {
    T v{};
    std::memcpy(&v, p_ + off_, sizeof(T));
    off_ += sizeof(T);
    return v;
  }
  std::size_t offset() const { return off_; }
  std::size_t size() const { return n_; }

 private:
  const unsigned char* p_;
  std::size_t n_;
  std::size_t off_ = 0;
};

template <typename T>
void Put(std::string& s, const T& v) {
  s.append(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <std::size_t N>
bool Finite(const std::array<double, N>& a) {
  for (double v : a)
    if (!std::isfinite(v)) return false;
  return true;
}

constexpr char kHex[] = "0123456789ABCDEF";

}  // namespace

bool ParseA3RefMessage(const void* data, std::size_t size, A3TeleopFrame& out,
                       std::string* error) {
  auto fail = [error](const char* what) {
    if (error) *error = what;
    return false;
  };
  if (size != kA3RefTopicSize + kA3RefBodySize) return fail("unexpected message size");
  const auto* bytes = static_cast<const unsigned char*>(data);
  if (std::memcmp(bytes, kA3RefTopic, kA3RefTopicSize) != 0) return fail("unexpected topic prefix");

  Reader r(bytes + kA3RefTopicSize, kA3RefBodySize);
  if (r.Get<std::uint32_t>() != kA3RefMagic) return fail("bad magic");
  if (r.Get<std::uint32_t>() != kA3RefVersion) return fail("bad version");
  A3TeleopFrame f{};
  f.stamp_ns = r.Get<std::int64_t>();
  for (auto& v : f.q_mujoco) v = r.Get<double>();
  for (auto& v : f.dq_mujoco) v = r.Get<double>();
  for (auto& v : f.pelvis_quat_wxyz) v = r.Get<double>();
  f.has_head_command = r.Get<std::uint8_t>() != 0;
  for (auto& v : f.head_q_rad) v = r.Get<double>();
  for (auto& v : f.head_dq_rad_s) v = r.Get<double>();
  if (r.offset() != kA3RefBodySize) return fail("internal layout error");

  if (!Finite(f.q_mujoco) || !Finite(f.dq_mujoco) || !Finite(f.pelvis_quat_wxyz) ||
      !Finite(f.head_q_rad) || !Finite(f.head_dq_rad_s)) {
    return fail("non-finite value");
  }
  double n2 = 0.0;
  for (double v : f.pelvis_quat_wxyz) n2 += v * v;
  if (std::abs(n2 - 1.0) > 1e-3) return fail("pelvis quaternion is not unit length");
  out = f;
  return true;
}

std::string PackA3RefMessage(const A3TeleopFrame& f) {
  std::string s(kA3RefTopic, kA3RefTopicSize);
  Put(s, kA3RefMagic);
  Put(s, kA3RefVersion);
  Put(s, static_cast<std::int64_t>(f.stamp_ns));
  for (double v : f.q_mujoco) Put(s, v);
  for (double v : f.dq_mujoco) Put(s, v);
  for (double v : f.pelvis_quat_wxyz) Put(s, v);
  Put(s, static_cast<std::uint8_t>(f.has_head_command ? 1 : 0));
  for (double v : f.head_q_rad) Put(s, v);
  for (double v : f.head_dq_rad_s) Put(s, v);
  return s;
}

#ifdef HAS_A3_TA_PROTO
void BuildTaWholeBodyCommand(const A3TeleopFrame& f, std::uint32_t seq,
                             std::int64_t stamp_ns,
                             aimdk::protocol::TaWholeBodyCommandChannel& msg) {
  msg.Clear();
  auto* header = msg.mutable_header();
  header->set_seq(seq);
  header->mutable_timestamp()->set_seconds(stamp_ns / 1'000'000'000LL);
  header->mutable_timestamp()->set_nanos(static_cast<int>(stamp_ns % 1'000'000'000LL));
  header->mutable_timestamp()->set_ms_since_epoch(stamp_ns / 1'000'000LL);
  header->set_frame_id("a3_ref_bridge");

  auto* d = msg.mutable_data();
  d->set_joint_layout(aimdk::protocol::TaJointLayout_BODY_31);

  auto* pelvis = d->mutable_pelvis_pose();
  for (double v : f.pelvis_quat_wxyz) pelvis->add_quat_wxyz(v);
  for (int i = 0; i < 3; ++i) pelvis->add_position_xyz(0.0);  // unused by the A3 policy

  // Policy-view order: waist[0..2], left arm[3..9], right arm[10..16],
  // left leg[17..22], right leg[23..28]  ->  protocol groups.
  auto* waist = d->mutable_waist_command();
  for (int i = 0; i < 3; ++i) waist->add_angles_rad(f.q_mujoco[i]);
  auto* arm = d->mutable_arm_command();
  for (int i = 0; i < 14; ++i) arm->add_angles_rad(f.q_mujoco[3 + i]);
  auto* leg = d->mutable_leg_command();
  for (int i = 0; i < 12; ++i) leg->add_angles_rad(f.q_mujoco[17 + i]);
  if (f.has_head_command) {
    auto* head = d->mutable_head_command();
    head->add_angles_rad(f.head_q_rad[0]);
    head->add_angles_rad(f.head_q_rad[1]);
  }

  // BODY_31 velocities: leg(12) + waist(3) + head(2) + arm(14).
  auto* vel = d->mutable_joint_velocities();
  for (int i = 0; i < 12; ++i) vel->add_velocities_rad_s(f.dq_mujoco[17 + i]);
  for (int i = 0; i < 3; ++i) vel->add_velocities_rad_s(f.dq_mujoco[i]);
  vel->add_velocities_rad_s(f.has_head_command ? f.head_dq_rad_s[0] : 0.0);
  vel->add_velocities_rad_s(f.has_head_command ? f.head_dq_rad_s[1] : 0.0);
  for (int i = 0; i < 14; ++i) vel->add_velocities_rad_s(f.dq_mujoco[3 + i]);
}
#endif

std::string Ros2EncodeName(const std::string& s) {
  std::string out;
  out.reserve(s.size() * 3);
  for (unsigned char c : s) {
    if (std::isalnum(c) || c == '/') {
      out += static_cast<char>(c);
    } else {
      out += '_';
      out += kHex[c >> 4];
      out += kHex[c & 0xF];
    }
  }
  return out;
}

std::string TaRos2TopicName(const std::string& channel_topic) {
  return channel_topic + "/" + Ros2EncodeName("pb:aimdk.protocol.TaWholeBodyCommandChannel");
}

}  // namespace a3_deploy
