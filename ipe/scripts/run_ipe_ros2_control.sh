#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if pgrep -af 'single_joint_lab|ipe_joint_state_publisher|ipe_ros2_control_node' \
    >/dev/null; then
    echo "Another EtherCAT master appears to be running." >&2
    echo "Stop it with Ctrl+C before starting ros2_control." >&2
    pgrep -af 'single_joint_lab|ipe_joint_state_publisher|ipe_ros2_control_node' >&2 || true
    exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${project_dir}/install-ros2/setup.bash"
set -u
exec ros2 launch ethercat_master ipe_ros2_control.launch.py
