#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=project_env.sh
source "${script_dir}/project_env.sh"

check_hardware=false
if [[ "${1:-}" == "--hardware" ]]; then
    check_hardware=true
elif [[ $# -ne 0 ]]; then
    echo "Usage: $0 [--hardware]" >&2
    exit 2
fi

failures=0
warnings=0
ok() { printf '[OK]   %s\n' "$1"; }
warn() { printf '[WARN] %s\n' "$1"; warnings=$((warnings + 1)); }
fail() { printf '[FAIL] %s\n' "$1"; failures=$((failures + 1)); }

printf 'Project: %s\n' "${IPE_PROJECT_DIR}"
printf 'ROS setup: %s\n' "${IPE_ROS_SETUP}"
printf 'EtherCAT interface: %s\n\n' "${IPE_INTERFACE}"

for command in bash cmake c++ python3 colcon git ip; do
    if command -v "${command}" >/dev/null 2>&1; then
        ok "command available: ${command}"
    else
        fail "missing command: ${command}"
    fi
done

for command in getcap setcap stdbuf script; do
    if command -v "${command}" >/dev/null 2>&1; then
        ok "runtime utility available: ${command}"
    else
        fail "missing runtime utility: ${command}"
    fi
done

if [[ -r "${IPE_ROS_SETUP}" ]]; then
    ok "ROS setup file is readable"
    ipe_source_ros
    for package in controller_manager hardware_interface rclcpp xacro; do
        if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
            ok "ROS package available: ${package}"
        else
            fail "missing ROS package: ${package}"
        fi
    done
else
    fail "ROS setup file not found: ${IPE_ROS_SETUP}"
fi

if [[ -r "${IPE_PROJECT_DIR}/ipe/SOEM/LICENSE" ]]; then
    ok "vendored SOEM dependency is present"
else
    fail "SOEM source tree is incomplete"
fi

if [[ -r "${IPE_WORKSPACE}/install/setup.bash" ]]; then
    ok "ROS workspace has been built"
else
    warn "ROS workspace is not built yet"
fi

if ${check_hardware}; then
    if [[ -d "/sys/class/net/${IPE_INTERFACE}" ]]; then
        ok "network interface exists: ${IPE_INTERFACE}"
    else
        fail "network interface not found: ${IPE_INTERFACE}"
    fi

    if [[ -r "/sys/class/net/${IPE_INTERFACE}/carrier" ]] &&
       [[ "$(<"/sys/class/net/${IPE_INTERFACE}/carrier")" == "1" ]]; then
        ok "physical Ethernet link is up"
    else
        fail "physical Ethernet link is down"
    fi

    mapfile -t masters < <(
        pgrep -af \
          'single_joint_lab|ipe_three_mode_lab|ipe_ros2_control_node' \
          | grep -v 'check_system.sh' || true
    )
    if (( ${#masters[@]} == 0 )); then
        ok "no EtherCAT master is running"
    elif (( ${#masters[@]} == 1 )); then
        warn "one EtherCAT master is already running: ${masters[0]}"
    else
        fail "multiple possible EtherCAT masters are running"
        printf '       %s\n' "${masters[@]}"
    fi
fi

printf '\nResult: %d failure(s), %d warning(s).\n' "${failures}" "${warnings}"
(( failures == 0 ))
