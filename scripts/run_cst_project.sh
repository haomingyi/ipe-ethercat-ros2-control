#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${project_dir}/ros2_ws"
log_dir="${project_dir}/artifacts/test-logs"

if pgrep -af 'single_joint_lab|ipe_three_mode_lab|ipe_joint_state_publisher|ipe_ros2_control_node' >/dev/null; then
  echo "Another EtherCAT master is already running." >&2
  pgrep -af 'single_joint_lab|ipe_three_mode_lab|ipe_joint_state_publisher|ipe_ros2_control_node' >&2 || true
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
set -u

mkdir -p "${log_dir}"
stdbuf -oL -eL ros2 launch ipe_bringup ipe_cst_project.launch.py \
  2>&1 | tee "${log_dir}/cst_project_latest.log"
