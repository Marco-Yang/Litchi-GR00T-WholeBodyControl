// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// a3_ta_bridge: ZMQ "a3_ref" (real-time Pico -> A3 retarget node)  ->  AimRT ROS2 channel
// /ta/whole_body_command (aimdk.protocol.TaWholeBodyCommandChannel, protobuf).
//
// The deploy process (a3_deploy_onnx_ref, config/a3_aimrt_config.ros2.yaml) subscribes to that
// channel through AimRT's ros2 backend, which carries protobuf channels as
// ros2_plugin_proto/msg/RosMsgWrapper{serialization_type="pb", context=[], data=<pb bytes>} on
// the topic "<channel>/<Ros2NameEncode(msg_type)>". This tool publishes exactly that, so no
// AimRT runtime needs to run on the sender side.
//
//   a3_ta_bridge --zmq tcp://127.0.0.1:5560 [--topic /ta/whole_body_command]
//                [--max-age-ms 200] [--print-every 100] [--dry-run]
//
// Safety: nothing is published when no valid a3_ref frame arrived within --max-age-ms; the
// receiver then falls back to its stale-hold / stand logic. Frames are re-stamped with the
// local system clock (the deploy re-stamps on arrival as well).
#include <zmq.hpp>

#include <atomic>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "a3_deploy/a3_ta_bridge_codec.hpp"

#ifdef A3_TA_BRIDGE_WITH_ROS2
#include <rclcpp/rclcpp.hpp>
#include <ros2_plugin_proto/msg/ros_msg_wrapper.hpp>
#endif

namespace {

std::atomic<bool> g_stop{false};
void OnSignal(int) { g_stop.store(true); }

std::int64_t NowSystemNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

struct Args {
  std::string zmq_url = "tcp://127.0.0.1:5560";
  std::string topic = "/ta/whole_body_command";
  double max_age_ms = 200.0;
  int print_every = 100;
  bool dry_run = false;
};

bool ParseArgs(int argc, char** argv, Args& a) {
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](std::string& dst) {
      if (i + 1 >= argc) return false;
      dst = argv[++i];
      return true;
    };
    std::string v;
    if (k == "--zmq" && next(v)) a.zmq_url = v;
    else if (k == "--topic" && next(v)) a.topic = v;
    else if (k == "--max-age-ms" && next(v)) a.max_age_ms = std::atof(v.c_str());
    else if (k == "--print-every" && next(v)) a.print_every = std::atoi(v.c_str());
    else if (k == "--dry-run") a.dry_run = true;
    else {
      std::cerr << "usage: a3_ta_bridge [--zmq URL] [--topic T] [--max-age-ms N] "
                   "[--print-every N] [--dry-run]\n";
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!ParseArgs(argc, argv, args)) return 2;
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

#ifdef A3_TA_BRIDGE_WITH_ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("a3_ta_bridge");
  const std::string ros_topic = a3_deploy::TaRos2TopicName(args.topic);
  // Same QoS as the AimRT config (keep_last 10, best_effort, volatile).
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  auto pub = node->create_publisher<ros2_plugin_proto::msg::RosMsgWrapper>(ros_topic, qos);
  const char* domain = std::getenv("ROS_DOMAIN_ID");
  const char* lo = std::getenv("ROS_LOCALHOST_ONLY");
  std::cout << "[a3_ta_bridge] ROS2 topic: " << ros_topic << "  (ROS_DOMAIN_ID="
            << (domain ? domain : "0 [default]") << ", ROS_LOCALHOST_ONLY=" << (lo ? lo : "unset")
            << ")\n";
  if (domain == nullptr || std::string(domain) == "0") {
    std::cerr << "[a3_ta_bridge] WARNING: ROS_DOMAIN_ID is unset/0. Domain 0 is shared with every\n"
                 "  other ROS2 participant on this host and LAN. Use a dedicated ROS_DOMAIN_ID\n"
                 "  (same value as a3_deploy_onnx_ref), and ROS_LOCALHOST_ONLY=1 when both run on\n"
                 "  one machine.\n";
  }
#else
  if (!args.dry_run) {
    std::cerr << "[a3_ta_bridge] built without ROS2; only --dry-run is available\n";
    return 3;
  }
#endif

  zmq::context_t ctx(1);
  zmq::socket_t sub(ctx, zmq::socket_type::sub);
  sub.set(zmq::sockopt::subscribe, a3_deploy::kA3RefTopic);
  sub.set(zmq::sockopt::rcvtimeo, 50);
  sub.set(zmq::sockopt::conflate, 1);  // always the newest frame; the buffer re-times anyway
  sub.connect(args.zmq_url);
  std::cout << "[a3_ta_bridge] ZMQ SUB " << args.zmq_url << (args.dry_run ? "  (dry run)" : "")
            << "\n";

  std::uint32_t seq = 0;
  std::uint64_t forwarded = 0, rejected = 0;
  aimdk::protocol::TaWholeBodyCommandChannel msg;
  std::string serialized;
#ifdef A3_TA_BRIDGE_WITH_ROS2
  ros2_plugin_proto::msg::RosMsgWrapper wrapper;
  wrapper.serialization_type = "pb";
#endif

  while (!g_stop.load()) {
    zmq::message_t raw;
    zmq::recv_result_t res;
    try {
      res = sub.recv(raw, zmq::recv_flags::none);
    } catch (const zmq::error_t& e) {
      if (e.num() == EINTR) continue;  // SIGINT/SIGTERM: g_stop is checked by the loop
      std::cerr << "[a3_ta_bridge] zmq error: " << e.what() << "\n";
      break;
    }
    const std::int64_t now_ns = NowSystemNs();
    if (!res) continue;  // source quiet: publish nothing, the receiver falls back on its own
    a3_deploy::A3TeleopFrame frame{};
    std::string error;
    if (!a3_deploy::ParseA3RefMessage(raw.data(), raw.size(), frame, &error)) {
      if ((rejected++ % 100) == 0) std::cerr << "[a3_ta_bridge] reject frame: " << error << "\n";
      continue;
    }
    // Drop frames that were stamped long ago by the sender (e.g. a stalled/buffered source).
    const double age_ms = (now_ns - frame.stamp_ns) * 1e-6;
    if (frame.stamp_ns > 0 && (age_ms > args.max_age_ms * 5.0 || age_ms < -1000.0)) {
      if ((rejected++ % 100) == 0)
        std::cerr << "[a3_ta_bridge] reject frame: sender stamp age " << age_ms << " ms\n";
      continue;
    }

    a3_deploy::BuildTaWholeBodyCommand(frame, seq++, now_ns, msg);
    if (!args.dry_run) {
#ifdef A3_TA_BRIDGE_WITH_ROS2
      serialized.clear();
      msg.SerializeToString(&serialized);
      wrapper.data.assign(serialized.begin(), serialized.end());
      pub->publish(wrapper);
#endif
    }
    ++forwarded;
    if (args.print_every > 0 && (forwarded % static_cast<std::uint64_t>(args.print_every)) == 0) {
      std::cout << "[a3_ta_bridge] forwarded=" << forwarded << " rejected=" << rejected
                << " last_age_ms=" << age_ms << " pelvis_wxyz=[" << frame.pelvis_quat_wxyz[0] << ","
                << frame.pelvis_quat_wxyz[1] << "," << frame.pelvis_quat_wxyz[2] << ","
                << frame.pelvis_quat_wxyz[3] << "]\n";
    }
  }
#ifdef A3_TA_BRIDGE_WITH_ROS2
  rclcpp::shutdown();
#endif
  std::cout << "[a3_ta_bridge] stopped, forwarded=" << forwarded << " rejected=" << rejected << "\n";
  return 0;
}
