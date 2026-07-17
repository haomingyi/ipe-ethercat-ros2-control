#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
installed_node="${project_dir}/install-ros2/ethercat_master/lib/ethercat_master/ipe_joint_state_publisher"
installed_control_node="${project_dir}/install-ros2/ethercat_master/lib/ethercat_master/ipe_ros2_control_node"

if [[ ! -e "${installed_node}" || ! -e "${installed_control_node}" ]]; then
    echo "ROS 2 EtherCAT executables are not built." >&2
    echo "Build it with the colcon command in README.md first." >&2
    exit 1
fi

node_binary="$(readlink -f "${installed_node}")"
control_node_binary="$(readlink -f "${installed_control_node}")"
echo "Granting EtherCAT network capabilities to:"
echo "  ${node_binary}"
echo "  ${control_node_binary}"
capabilities="cap_net_raw,cap_net_admin,cap_sys_nice=ep"
sudo setcap "${capabilities}" "${node_binary}"
sudo setcap "${capabilities}" "${control_node_binary}"

capability="$(getcap "${node_binary}")"
control_node_capability="$(getcap "${control_node_binary}")"
if [[ "${capability}" != *"cap_net_admin"* ||
      "${capability}" != *"cap_net_raw"* ||
      "${capability}" != *"cap_sys_nice"* ||
      "${control_node_capability}" != *"cap_net_admin"* ||
      "${control_node_capability}" != *"cap_net_raw"* ||
      "${control_node_capability}" != *"cap_sys_nice"* ]]; then
    echo "Capability verification failed: ${capability}" >&2
    exit 1
fi

echo "Configured successfully: ${capability}"
echo "Configured successfully: ${control_node_capability}"
echo "You can now run scripts/run_ipe_joint_state_publisher.sh without sudo."
echo "You can now launch ipe_ros2_control.launch.py without sudo."
