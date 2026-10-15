#!/usr/bin/env bash
# ============================================================================
# READ-ONLY survey of the robot's MDU for the SONIC deployment (plan 18.9).  Writes nothing, changes nothing, uses no sudo, sends
# nothing to the robot's topics: it only runs ls / cat / grep / ps / ss / systemctl is-active on the machine it runs on.
#
#   ssh <user>@10.42.10.12 'bash -s' < gear_sonic_deploy/scripts/a3_mdu_readonly_check.sh        # from the operator PC (stdin, nothing copied)
#
# What it answers: which platform / OS / ROS 2 the MDU runs, whether motion_control and the other vendor services are up, what
# `sm_config.yaml` says for the Motion group, which ROS_DOMAIN_ID / RMW the vendor environment sets, which vendor AimRT configs name
# the /body_drive/* channels and whether any of them has a ros2 backend (Litchi's "ADU-style" transport needs it), which vendor module
# subscribes /ta/whole_body_command, and whether a SONIC package is already on the machine.
# Lines that look like secrets are masked.  Each command is limited to a few seconds and a few dozen lines.
# ============================================================================
set +e
export LC_ALL=C
MAXL="${MAXL:-40}"

sec() { printf '\n===== %s =====\n' "$*"; }
mask() { sed -E 's/((pass(word|wd)?|token|secret|api[_-]?key|private[_-]?key)[A-Za-z_]*[=:][[:space:]]*)[^[:space:]]+/\1***/Ig'; }
run() { printf '$ %s\n' "$1"; timeout "${2:-10}" bash -c "$1" 2>&1 | mask | head -n "${3:-$MAXL}"; }

sec "who / where"
run 'hostname; whoami; id; date -Is; uptime'
run 'uname -a; cat /proc/device-tree/model 2>/dev/null | tr -d "\0"; echo; cat /etc/os-release 2>/dev/null | head -4'
run 'lscpu 2>/dev/null | grep -E "Architecture|Model name|^CPU\(s\)|Max MHz|Thread|Vendor"'
run 'free -m; df -h / /agibot /home 2>/dev/null'

sec "network and time"
run 'ip -br addr 2>/dev/null'
run 'for u in chrony chronyd ptp4l phc2sys systemd-timesyncd; do st=$(systemctl is-active "$u" 2>/dev/null); echo "$u: ${st:-n/a}"; done'

sec "vendor services and processes (is the stock motion_control running?)"
run 'systemctl is-active agibot_pm 2>/dev/null || echo "agibot_pm: n/a"'
run 'systemctl list-units --type=service --no-pager 2>/dev/null | grep -i -E "agibot|aimrt|motion|body|iox|roudi|teleop|ta[._-]" | head -30'
run 'ps -eo pid,etime,user,args --sort=start_time 2>/dev/null | grep -i -E "motion_control|body_drive|aimrt|roudi|agent|teleop|[/ ]ta[_ -]|sonic|a3_deploy|ros2|rclcpp" | grep -v grep | cut -c1-220' 10 50

sec "vendor tree"
run 'ls -la /agibot 2>/dev/null; ls -la /agibot/software/v0 2>/dev/null; ls -la /agibot/software/v0/config 2>/dev/null; ls -la /agibot/software/v0/config/sm 2>/dev/null'

sec "sm_config.yaml: the Motion function group (read only; a SONIC bring-up wants it to start only the agent, Litchi 6.1)"
run 'grep -n -i -B3 -A14 "\"Motion\"" /agibot/software/v0/config/sm/sm_config.yaml 2>/dev/null' 10 60
run 'ls -la /agibot/software/v0/config/sm/*.yaml* 2>/dev/null'

sec "vendor environment script: ROS / RMW / DDS / iceoryx variables only"
run 'ls -la /agibot/software/v0/entry/env 2>/dev/null; grep -n -E "^[[:space:]]*(export[[:space:]]+)?(ROS_|RMW_|FASTRTPS|FASTDDS|CYCLONE|AMENT_|AIMRT|ICEORYX|IOX|A3_|LD_LIBRARY_PATH)" /agibot/software/v0/entry/env/env.sh 2>/dev/null' 10 60
run 'printenv 2>/dev/null | grep -E "^(ROS|RMW|FASTRTPS|FASTDDS|CYCLONE|AMENT_PREFIX|AIMRT|A3_)" | cut -c1-200'
run 'ls /opt/ros 2>/dev/null; command -v ros2 || echo "ros2 CLI: not on PATH"'

sec "which vendor AimRT configs mention the body-drive channels, and do any of them enable a ros2 backend?"
run 'grep -rIl --include="*.yaml" --include="*.yml" -e "body_drive" /agibot/software/v0 2>/dev/null | head -30' 30 40
run 'grep -rIl --include="*.yaml" --include="*.yml" -e "ros2_plugin" /agibot/software/v0 2>/dev/null | head -30' 30 40
run 'for f in $(grep -rIl --include="*.yaml" --include="*.yml" -e "body_drive" /agibot/software/v0 2>/dev/null | head -8); do echo "--- $f"; grep -n -E "type: (ros2|iceoryx|net|mqtt)|enable_backends|ros2_plugin|iceoryx_plugin|/body_drive/[a-z_/]*(state|command|data)|reliability|qos" "$f" | head -16; done' 40 160

sec "which vendor module touches /ta/whole_body_command (the 2 unknown subscribers seen on ROS 2)"
run 'grep -rIn --include="*.yaml" --include="*.yml" --include="*.json" -e "whole_body_command" /agibot/software/v0 2>/dev/null | cut -c1-220 | head -20' 30 25

sec "is a SONIC / A3 deploy package already here?"
run 'find /agibot /home /opt /root -maxdepth 4 \( -name run_a3.sh -o -name a3_deploy_onnx_ref -o -iname "*sonic*" \) 2>/dev/null | head -20' 20 25

sec "NPU runtime (RKNN) and listening sockets"
run 'ls -la /usr/lib/librknnrt.so /usr/lib/aarch64-linux-gnu/librknnrt.so 2>/dev/null; strings -a /usr/lib/librknnrt.so 2>/dev/null | grep -m2 -i "librknnrt version"'
run 'ss -ltnup 2>/dev/null | head -40'

printf '\n===== done: nothing was written or changed =====\n'
