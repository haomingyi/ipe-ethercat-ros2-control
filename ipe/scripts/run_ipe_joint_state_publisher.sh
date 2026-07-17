#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
installed_node="${project_dir}/install-ros2/ethercat_master/lib/ethercat_master/ipe_joint_state_publisher"
log_dir="${project_dir}/../artifacts/test-logs"

set +u
source /opt/ros/jazzy/setup.bash
source "${project_dir}/install-ros2/setup.bash"
set -u
set -u

node_binary="$(readlink -f "${installed_node}")"
capability="$(getcap "${node_binary}")"
if [[ "${capability}" != *"cap_net_admin,cap_net_raw"* &&
      "${capability}" != *"cap_net_raw,cap_net_admin"* ]]; then
    echo "EtherCAT capability is missing." >&2
    echo "Run: ${project_dir}/scripts/setup_ethercat_capability.sh" >&2
    exit 1
fi

cd "${project_dir}"
mkdir -p "${log_dir}"
stdbuf -oL -eL ros2 run ethercat_master ipe_joint_state_publisher \
    --ros-args --params-file "${project_dir}/config/ipe_joint_state.yaml" "$@" \
    2>&1 | tee "${log_dir}/ros2_joint_state_node.txt"
