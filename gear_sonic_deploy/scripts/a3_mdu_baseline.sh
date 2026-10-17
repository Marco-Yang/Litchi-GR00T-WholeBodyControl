#!/usr/bin/env bash
# ============================================================================
# READ-ONLY fingerprint of the MDU, to be run BEFORE and AFTER a vendor software upgrade (e.g. 3.2.17 -> 3.3) and compared with diff.
# Writes nothing (print to stdout; redirect it yourself), changes nothing, uses no sudo, sends nothing to any robot topic.
#
#   on the MDU:   bash a3_mdu_baseline.sh | tee ~/mdu_baseline_before_upgrade.txt
#   after the upgrade:   bash a3_mdu_baseline.sh > ~/mdu_baseline_after_upgrade.txt
#   on the PC (copy both files over first):   diff -u mdu_baseline_before_upgrade.txt mdu_baseline_after_upgrade.txt
#
# What changes between two software versions and would break the relay (docs/workflow/a3_integration_plan.md 18.22 / 18.23): the vendor
# version strings, the ROS 2 distribution and the libraries the relay is linked against (rclcpp, fastcdr, libzmq), the /body_drive/* channel
# definitions of hal_ethercat / hal_elink (names, message types, iceoryx / ros2 backends), the Motion group of sm_config.yaml, the vendor
# environment script, what the stock motion control subscribes, which ports listen, and whether our files under ~ survive.
# Time-dependent lines (date, uptime, pids, load) are left out so that an unchanged system gives an empty diff.
# ============================================================================
set +e
export LC_ALL=C
export V0="${V0:-/agibot/software/v0}"            # exported: every command below runs in a child bash -c
export RELAY_DIR="${RELAY_DIR:-$HOME/a3_relay_rockchip}"

sec() { printf '\n===== %s =====\n' "$*"; }
run() { printf '$ %s\n' "$1"; timeout "${2:-10}" bash -c "$1" 2>&1 | head -n "${3:-60}"; }

sec "versions"
run 'uname -sr; uname -m; grep -E "^(PRETTY_NAME|VERSION)=" /etc/os-release'
run 'grep -h -E "系统版本|full_version|bsp_version|git_commit|build_time|target" /etc/motd /etc/update-motd.d/* "$HOME/.login_info" 2>/dev/null | sed "s/\x1b\[[0-9;]*m//g" | sort -u'
run 'ls /agibot/software 2>/dev/null; ls "$V0" 2>/dev/null | head -30'
run 'ls "$V0"/*version* "$V0"/../*version* /agibot/*version* 2>/dev/null; cat "$V0"/version* 2>/dev/null | head -5'
run 'head -30 "$V0/metadata.yaml" 2>/dev/null' 10 30

sec "ROS 2 and the libraries the relay bundle is linked against"
run 'ls /opt/ros 2>/dev/null; ls /opt/ros/*/lib/librclcpp.so* /opt/ros/*/lib/libfastcdr.so* 2>/dev/null'
run 'dpkg -l 2>/dev/null | grep -E "libzmq|libsodium|libpgm|libnorm|libbsd0|libgssapi-krb5|libtbb|libstdc\+\+6|libc6 " | awk "{print \$2, \$3}"'
run 'ldconfig -p 2>/dev/null | grep -E "libzmq|librknnrt" | sed "s/ (.*) =>/ =>/"'
run 'strings -a /usr/lib/librknnrt.so 2>/dev/null | grep -m1 -i "librknnrt version"; strings -a /usr/lib/aarch64-linux-gnu/librknnrt.so 2>/dev/null | grep -m1 -i "librknnrt version"'
if [[ -x "${RELAY_DIR}/a3_zmq_relay" ]]; then
  # LD_LIBRARY_PATH as run_a3.sh gets it after it sources /opt/ros/<distro>/setup.bash (a non-interactive ssh shell has no ROS environment)
  run "cd '${RELAY_DIR}' && LD_LIBRARY_PATH=.:/opt/ros/jazzy/lib:/opt/ros/humble/lib ldd ./a3_zmq_relay | sed 's/ (0x[0-9a-f]*)//' | sort" 20 60
  run "cd '${RELAY_DIR}' && sha256sum a3_zmq_relay run_a3_relay.sh a3_real_preflight.sh 2>/dev/null"
else
  echo "(no relay bundle at ${RELAY_DIR}: set RELAY_DIR to check that it survives the upgrade)"
fi

sec "vendor config fingerprints (sha256 of the files that define body-drive, the Motion group and the environment)"
run 'cd "$V0" 2>/dev/null && sha256sum config/sm/sm_config.yaml entry/env/env.sh 2>/dev/null'
run 'cd "$V0" 2>/dev/null && find config -type f \( -name "hal_ethercat*t3d0*" -o -name "hal_ethercat*a3_ultra*" -o -name "hal_elink*" -o -path "*t3d0*" \) 2>/dev/null | sort | head -40 | xargs -r sha256sum' 20 50

sec "body-drive channels as the vendor config defines them (names, types, backends)"
run 'f=$(ls "$V0"/config/hal_ethercat/hal_ethercat_a3_ultra_t3d0_cfg.yaml 2>/dev/null | head -1); [ -n "$f" ] && echo "$f" && grep -n -E "topic_name|enable_backends|msg_type|body_drive" "$f" | head -80' 10 90
sec "what the vendor config says about timeouts, heartbeats, emergency and safe states (the unanswered hal_ethercat question)"
run 'for f in "$V0/config/hal_ethercat/hal_ethercat_a3_ultra_t3d0_cfg.yaml" "$V0/config/hal_elink/hal_elink_a3_cfg.yaml"; do echo "--- $f"; grep -n -i -E "timeout|watchdog|heartbeat|stale|lost|emergency|estop|safe|damp|hold|disable|motion_info|fault|protect" "$f" 2>/dev/null | cut -c1-200; done' 10 90
run 'grep -rIn --include="*.yaml" -e "whole_body_command" -e "pnc/arm_joint_command" "$V0/config" 2>/dev/null | sed "s/:[0-9]*:/: /" | sort -u | cut -c1-200' 20 30
run 'grep -n -i -B3 -A14 "\"Motion\"" "$V0/config/sm/sm_config.yaml" 2>/dev/null' 10 40

sec "services, applications, listening ports (no pids / times)"
run 'systemctl list-units --type=service --no-pager --plain 2>/dev/null | awk "{print \$1, \$3, \$4}" | grep -i -E "agibot|aimrt|motion|body|iox|roudi|ta[._-]" | sort' 15 40
run 'ps -eo comm,args 2>/dev/null | grep -i -E "motion_control|aimrt_main|roudi|agent|hal_|colistener|motion_player" | grep -v grep | sed -E "s#^[^ ]+ ##" | cut -c1-200 | sort -u' 10 40
run 'ss -ltn 2>/dev/null | awk "NR>1{print \$4}" | sort -u' 10 60
run 'ls -la "$HOME" 2>/dev/null | awk "{print \$1, \$3, \$9}" | tail -n +2'
echo
echo "# end of baseline"
