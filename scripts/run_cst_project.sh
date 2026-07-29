#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=project_env.sh
source "${script_dir}/project_env.sh"
log_dir="${IPE_PROJECT_DIR}/artifacts/test-logs"

if pgrep -af 'single_joint_lab|ipe_three_mode_lab|ipe_ros2_control_node' >/dev/null; then
  echo "Another EtherCAT master is already running." >&2
  pgrep -af 'single_joint_lab|ipe_three_mode_lab|ipe_ros2_control_node' >&2 || true
  exit 1
fi

ipe_source_ros
ipe_source_workspace

mkdir -p "${log_dir}"
stdbuf -oL -eL ros2 launch ipe_bringup ipe_cst_project.launch.py \
  interface:="${IPE_INTERFACE}" \
  zero_count:="${IPE_ZERO_COUNT}" \
  direction:="${IPE_DIRECTION}" \
  velocity_raw_to_rad_s:="${IPE_VELOCITY_RAW_TO_RAD_S}" \
  2>&1 | tee "${log_dir}/cst_project_latest.log"
