#!/usr/bin/env python3
# ruff: noqa: T201
"""READ-ONLY ROS 2 graph probe (plan 18.8).

Joins a domain as a bare node (no publishers, subscriptions or services of ours; only the standard ROS 2 node infrastructure, rosout
and parameter services switched off), waits for discovery, then prints the topics and, for the interesting ones, who publishes /
subscribes with which QoS.

Always run it under a memory cap, on the Cyclone RMW and pinned to the robot NIC (conda Humble's Fast DDS 2.6.4 once allocated tens
of GB in a foreign domain, plan 9.7 / 18.7); the RMW prints a lot of rcutils noise on stderr, hide it with 2>/dev/null:

    systemd-run --user --scope -p MemoryMax=16G -p MemorySwapMax=0 env ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=0 \\
        RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\
        CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="10.42.10.99" multicast="true"/></Interfaces></General></Domain></CycloneDDS>' \\
        timeout 90 python a3_ros_graph_probe.py 15 2>/dev/null

    python a3_ros_graph_probe.py [WAIT_SECONDS] [TOPIC_REGEX] [--sonic-gate]

``--sonic-gate`` turns it into a pass / fail check for running the deploy off the robot (Litchi's "ADU-style" ros2 config, used by
``deploy.sh --remote``): exit 0 only if the robot publishes the 6 state topics and subscribes the 4 command topics of
``a3_aimrt_config.ros2.yaml`` with matching message types and compatible QoS; exit 3 otherwise, with the reasons.
"""

from __future__ import annotations

import os
import re
import sys
import time

# What a3_deploy_onnx_ref needs on the wire in the ros2 (cross-board) configuration: topic -> (message type, who must be on the robot:
# 'pub' = the robot publishes it, 'sub' = the robot subscribes to it).  Our own QoS is best_effort / volatile / keep_last 10 on
# every one of them (config/a3_aimrt_config.ros2.yaml).
SONIC_TOPICS = {
    "/body_drive/waist_joint_state": ("joint_msgs/msg/JointState", "pub"),
    "/body_drive/leg_joint_state": ("joint_msgs/msg/JointState", "pub"),
    "/body_drive/arm_joint_state": ("joint_msgs/msg/JointState", "pub"),
    "/body_drive/neck_joint_state": ("joint_msgs/msg/JointState", "pub"),
    "/body_drive/pelvis_imu/data": ("sensor_msgs/msg/Imu", "pub"),
    "/body_drive/torso_imu/data": ("sensor_msgs/msg/Imu", "pub"),
    "/body_drive/waist_joint_command": ("joint_msgs/msg/JointCommand", "sub"),
    "/body_drive/leg_joint_command": ("joint_msgs/msg/JointCommand", "sub"),
    "/body_drive/arm_joint_command": ("joint_msgs/msg/JointCommand", "sub"),
    "/body_drive/neck_joint_command": ("joint_msgs/msg/JointCommand", "sub"),
}


def sonic_gate(types_by_topic, pubs_by_topic, subs_by_topic):
    """Problems of the robot's ROS 2 graph for the ros2 deploy config (empty list = it offers everything).

    ``types_by_topic``: {topic: [message types]}; ``pubs_by_topic`` / ``subs_by_topic``: {topic: [reliability name of each endpoint]}.

    Our state subscriptions are best_effort, which accepts a best_effort or a reliable robot publisher.  Our command publishers are
    best_effort, which only reaches best_effort robot subscribers: one reliable subscriber on a command topic is a mismatch.
    """
    problems = []
    for topic, (want_type, role) in SONIC_TOPICS.items():
        if topic not in types_by_topic:
            problems.append(f"{topic}: not on the ROS 2 network")
            continue
        if want_type not in types_by_topic[topic]:
            problems.append(f"{topic}: type {list(types_by_topic[topic])} instead of {want_type}")
        if role == "pub" and not pubs_by_topic.get(topic):
            problems.append(f"{topic}: nobody publishes it")
        if role == "sub":
            subs = subs_by_topic.get(topic, [])
            if not subs:
                problems.append(f"{topic}: nobody subscribes to it (our commands would go nowhere)")
            elif any(r != "best_effort" for r in subs):
                problems.append(
                    f"{topic}: a robot subscriber is {sorted(set(subs))}; our best_effort publisher cannot reach a reliable one")
    return problems


def _rss_mb() -> float:
    for line in open("/proc/self/status"):
        if line.startswith("VmRSS"):
            return int(line.split()[1]) / 1024
    return 0.0


def main() -> int:
    import rclpy
    from rclpy.node import Node

    gate = "--sonic-gate" in sys.argv
    argv = [a for a in sys.argv[1:] if a != "--sonic-gate"]
    wait_s = float(argv[0]) if argv else 12.0
    pattern = re.compile(argv[1] if len(argv) > 1 else r"^/(ros2/)?(body_drive|ta|motion|hand)/")

    rclpy.init()
    node = Node("a3_graph_probe", enable_rosout=False, start_parameter_services=False)
    t0 = time.time()
    peak = 0.0
    while time.time() - t0 < wait_s:
        rclpy.spin_once(node, timeout_sec=0.2)
        peak = max(peak, _rss_mb())
    print(f"rmw={os.environ.get('RMW_IMPLEMENTATION')} domain={os.environ.get('ROS_DOMAIN_ID')} waited {wait_s:.0f} s, peak RSS {peak:.0f} MB")

    topics = sorted(node.get_topic_names_and_types())
    print(f"topics visible: {len(topics)}")
    pol = lambda q: f"{q.reliability.name.lower()}/{q.durability.name.lower()}/{q.history.name.lower()}{q.depth}"  # noqa: E731
    if not gate:
        try:
            nodes = sorted(f"{ns.rstrip('/')}/{n}" for n, ns in node.get_node_names_and_namespaces())
            print(f"nodes visible: {len(nodes)}")
            for n in nodes[:60]:
                print("  ", n)
        except Exception as exc:  # noqa: BLE001 -- the Cyclone RMW cannot list nodes of non-ROS (AimRT) participants
            print("node list failed:", type(exc).__name__, str(exc)[:120])
        for name, types in topics:
            if not pattern.search(name):
                continue
            print(f"\n{name}   [{', '.join(types)}]")
            for label, infos in (("pub", node.get_publishers_info_by_topic(name)), ("sub", node.get_subscriptions_info_by_topic(name))):
                for i in infos:
                    print(f"   {label}  {i.node_namespace.rstrip('/')}/{i.node_name or '?':30s} qos={pol(i.qos_profile)}")
        others = [n for n, _ in topics if not pattern.search(n)]
        print(f"\nother topics ({len(others)}), complete list (an earlier version cut this at 1500 characters, which hid the /ros2/* mirrors):")
        for n in others:
            print("  ", n)
        code = 0
    else:
        pubs = {t: [i.qos_profile.reliability.name.lower() for i in node.get_publishers_info_by_topic(t)] for t in SONIC_TOPICS}
        subs = {t: [i.qos_profile.reliability.name.lower() for i in node.get_subscriptions_info_by_topic(t)] for t in SONIC_TOPICS}
        problems = sonic_gate({n: list(ty) for n, ty in topics}, pubs, subs)
        print("\nSONIC ros2 gate:", "PASS" if not problems else f"FAIL ({len(problems)} problems)")
        for p in problems:
            print("  -", p)
        code = 0 if not problems else 3
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
