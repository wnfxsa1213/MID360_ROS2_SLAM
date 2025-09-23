#!/usr/bin/env bash
set -euo pipefail

# Dynamic filter runtime verifier
# Verifies topics, logs, and end-to-end data flow for bag → SLAM → dynamic filter

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)/ws_livox/install/setup.bash"

info() { echo -e "\033[0;34m[INFO]\033[0m $*"; }
ok()   { echo -e "\033[0;32m[PASS]\033[0m $*"; }
warn() { echo -e "\033[1;33m[WARN]\033[0m $*"; }
fail() { echo -e "\033[0;31m[FAIL]\033[0m $*"; }

timeout_run() {
  local secs="$1"; shift
  timeout "${secs}s" bash -lc "$*"
}

wait_for_topic() {
  local topic="$1"; local secs="${2:-15}"
  for ((i=0;i<secs*2;i++)); do
    if ros2 topic list | grep -qx "$topic"; then return 0; fi
    sleep 0.5
  done
  return 1
}

main() {
  info "Sourcing ROS and workspace"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if [[ -f "$WS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$WS_SETUP"
  else
    fail "Workspace not built. Build with: cd ws_livox && colcon build"
    exit 1
  fi

  local errors=0

  info "Checking upstream topics from bag"
  if wait_for_topic "/livox/lidar" 15; then ok "/livox/lidar present"; else fail "Missing /livox/lidar"; ((errors++)); fi
  if wait_for_topic "/livox/imu" 15;   then ok "/livox/imu present";   else fail "Missing /livox/imu";   ((errors++)); fi

  info "Checking SLAM outputs (/slam/...)"
  if wait_for_topic "/slam/body_cloud" 20; then
    if timeout_run 6 "ros2 topic echo -n 1 /slam/body_cloud >/dev/null"; then ok "/slam/body_cloud has messages"; else fail "/slam/body_cloud no messages"; ((errors++)); fi
  else
    fail "Missing /slam/body_cloud"
    ((errors++))
  fi
  if wait_for_topic "/slam/lio_odom" 20; then ok "/slam/lio_odom present"; else fail "Missing /slam/lio_odom"; ((errors++)); fi

  info "Checking Localizer dynamic filter outputs"
  if wait_for_topic "/localizer/filter_stats" 20; then
    if timeout_run 8 "ros2 topic echo -n 1 /localizer/filter_stats >/dev/null"; then ok "/localizer/filter_stats published"; else fail "/localizer/filter_stats no data"; ((errors++)); fi
  else
    fail "Missing /localizer/filter_stats"
    ((errors++))
  fi

  # filtered_cloud only publishes when there is a subscriber; echo provides one
  if wait_for_topic "/localizer/filtered_cloud" 20; then
    if timeout_run 8 "ros2 topic echo -n 1 /localizer/filtered_cloud >/dev/null"; then ok "/localizer/filtered_cloud published"; else fail "/localizer/filtered_cloud no data"; ((errors++)); fi
  else
    fail "Missing /localizer/filtered_cloud"
    ((errors++))
  fi

  info "Checking Localizer logs from /rosout"
  # Look for key log lines from localizer_node
  if timeout_run 6 "ros2 topic echo -n 50 /rosout | grep -E 'name:.*localizer_node|msg:.*(LOAD FROM YAML CONFIG PATH|Dynamic filter enabled: YES)' >/dev/null"; then
    ok "Localizer loaded config and enabled dynamic filter"
  else
    warn "Could not confirm logs; check localizer_node terminal output"
  fi

  if [[ $errors -eq 0 ]]; then
    ok "Dynamic filter runtime checks passed"
    exit 0
  else
    fail "Dynamic filter runtime checks found $errors issue(s)"
    exit 2
  fi
}

main "$@"

