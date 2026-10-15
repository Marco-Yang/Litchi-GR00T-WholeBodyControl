#!/usr/bin/env bash
# ============================================================================
# READ-ONLY survey, part 2 (plan 18.11): how the MDU's body-drive (hal_ethercat) routes its /body_drive/* channels between iceoryx
# and ROS 2, who publishes the IMU topics, how motion_control (mc) is started, and the vendor Fast DDS profile.  Same rules as part 1:
# writes nothing, changes nothing, no sudo, publishes nothing; it only prints (parts of) configuration files and process facts.
#
#   ssh agi@10.42.10.12 'bash -s' < gear_sonic_deploy/scripts/a3_mdu_readonly_check2.sh
# ============================================================================
set +e
export LC_ALL=C
CFG=/agibot/software/v0/config
mask() { sed -E 's/((pass(word|wd)?|token|secret|api[_-]?key|private[_-]?key)[A-Za-z_]*[=:][[:space:]]*)[^[:space:]]+/\1***/Ig'; }
sec() { printf '\n===== %s =====\n' "$*"; }
run() { printf '$ %s\n' "$1"; timeout "${2:-10}" bash -c "$1" 2>&1 | mask | head -n "${3:-60}"; }

sec "which hal_ethercat config is running, and its channel section (the body-drive provider: where /body_drive/* go)"
run 'ps -eo pid,args | grep -E "aimrt_main_hal|aimrt_main .*hal_" | grep -v grep | cut -c1-200'
F=$CFG/hal_ethercat/hal_ethercat_a3_ultra_t3d0_cfg.yaml
run "ls -la $F; grep -n -E '^[[:space:]]*(channel|rpc|executor|log|plugin|module):' $F"
run "awk '/^[[:space:]]*channel:/{p=1} p{print NR\": \"\$0} /^[[:space:]]*rpc:/{if(p) exit}' $F" 10 170

sec "hal_elink (hand / other body-drive channels): the channel routing, for comparison"
run "awk '/^[[:space:]]*channel:/{p=1} p{print NR\": \"\$0} /^[[:space:]]*rpc:/{if(p) exit}' $CFG/hal_elink/hal_elink_a3_cfg.yaml" 10 90

sec "motion_control (mc) aimrt config: how it consumes /body_drive/* and /ta/whole_body_command (backends, topics)"
M=$CFG/motion_control/configuration/robot/a3_ultra_t3d0/aimrt/default.yaml
run "awk '/^[[:space:]]*channel:/{p=1} p{print NR\": \"\$0} /^[[:space:]]*rpc:/{if(p) exit}' $M" 10 120
run "sed -n 195,215p $M; sed -n 455,475p $M"

sec "the /ros2/body_drive/* names (seen in the UWB fusion config): who creates this ROS 2 mirror of the body-drive topics?"
run "grep -rIn --include='*.yaml' --include='*.yml' --include='*.json' --include='*.xml' --include='*.launch*' -e 'ros2/body_drive' -e '_ros2' /agibot/software/v0 2>/dev/null | grep -v '/tmp/' | cut -c1-220 | head -30" 20 40
run "awk '/^[[:space:]]*channel:/{p=1} p{print NR\": \"\$0} /^[[:space:]]*rpc:/{if(p) exit}' /agibot/software/v0/bin/aimrt_uwb_fusion_config.yaml" 10 60

sec "who publishes the IMU topics (backend lines around them)"
run "grep -rIn --include='*.yaml' -E 'imu/data' $CFG 2>/dev/null | grep -v '/tmp/' | cut -c1-200 | head -20"

sec "vendor Fast DDS profile used by the ROS 2 side (FASTRTPS_DEFAULT_PROFILES_FILE)"
run 'cat /agibot/software/v0/entry/cfg/ros_dds_configuration.xml 2>/dev/null' 10 80

sec "how motion_control is started / stopped (scripts and the state-machine CLI)"
run 'ls -la /agibot/software/v0/scripts /agibot/software/v0/scripts/motion_control /agibot/software/v0/bin 2>/dev/null | head -60'
run 'sed -n 1,50p /agibot/software/v0/scripts/motion_control/start_motion_control.sh 2>/dev/null' 10 50
run 'ls /agibot/software/v0/bin /agibot/software/v0/entry 2>/dev/null | grep -i -E "sm|ctl|cli|mode|switch" | head'

sec "ROS environment of the RUNNING mc process (same user, readable; ROS / DDS / AimRT variables only)"
run 'p=$(pgrep -x mc | head -1); echo "mc pid: ${p:-none}"; [ -n "$p" ] && tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -E "^(ROS_|RMW_|FASTRTPS|FASTDDS|CYCLONE|AIMRT|LD_LIBRARY_PATH|ADU_IP)" | cut -c1-220'

sec "the recorder: does it subscribe /ta/whole_body_command? (one of the 2 unknown ROS 2 subscribers)"
run "grep -n -B1 -A3 'whole_body_command' $CFG/record/topic_meta_list.yaml 2>/dev/null | head -12"
run "ls $CFG/record 2>/dev/null"

sec "writable? (so we know what a config edit would touch; nothing is written here)"
run 'findmnt -no TARGET,FSTYPE,OPTIONS / /opt /agibot 2>/dev/null | head -5; stat -c "%U %a %n" /opt/agibot /agibot/software/v0/config/hal_ethercat 2>/dev/null'

printf '\n===== done: nothing was written or changed =====\n'
