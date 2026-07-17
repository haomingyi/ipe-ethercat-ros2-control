#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
log_dir="${project_dir}/../artifacts/test-logs"
log_file="${log_dir}/ros2_three_mode_control_latest.log"

if pgrep -af 'single_joint_lab|ipe_three_mode_lab|ipe_joint_state_publisher|ipe_ros2_control_node' \
    >/dev/null; then
    echo "Another EtherCAT master appears to be running." >&2
    echo "Stop it with quit or Ctrl+C before starting three-mode control." >&2
    pgrep -af 'single_joint_lab|ipe_three_mode_lab|ipe_joint_state_publisher|ipe_ros2_control_node' >&2 || true
    exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${project_dir}/install-ros2/setup.bash"
set -u

mkdir -p "${log_dir}"
cd "${project_dir}"
stdbuf -oL -eL ros2 launch ethercat_master ipe_three_mode_control.launch.py \
    2>&1 | tee "${log_file}"
