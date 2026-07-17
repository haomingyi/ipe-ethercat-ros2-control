#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
installed_node="${project_dir}/ros2_ws/install/ethercat_master/lib/ethercat_master/ipe_ros2_control_node"

if [[ ! -e "${installed_node}" ]]; then
  echo "Project is not built. Run scripts/build_ros2_project.sh first." >&2
  exit 1
fi

node_binary="$(readlink -f "${installed_node}")"
sudo setcap cap_net_admin,cap_net_raw,cap_sys_nice=ep "${node_binary}"
getcap "${node_binary}"
