#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=project_env.sh
source "${script_dir}/project_env.sh"

ipe_source_ros
ipe_source_workspace

exec ros2 launch ipe_bringup ipe_cst_project.launch.py \
    use_mock_hardware:=true \
    interface:="${IPE_INTERFACE}" \
    zero_count:="${IPE_ZERO_COUNT}" \
    direction:="${IPE_DIRECTION}" \
    velocity_raw_to_rad_s:="${IPE_VELOCITY_RAW_TO_RAD_S}"
