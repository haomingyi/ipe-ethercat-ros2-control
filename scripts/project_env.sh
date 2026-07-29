#!/usr/bin/env bash

# Shared workstation configuration for project scripts.
# Source this file; do not execute it as a standalone command.

IPE_PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IPE_WORKSPACE="${IPE_PROJECT_DIR}/ros2_ws"

if [[ -f "${IPE_PROJECT_DIR}/.env" ]]; then
    set -a
    # shellcheck source=/dev/null
    source "${IPE_PROJECT_DIR}/.env"
    set +a
fi

: "${IPE_ROS_DISTRO:=jazzy}"
: "${IPE_ROS_SETUP:=/opt/ros/${IPE_ROS_DISTRO}/setup.bash}"
: "${IPE_INTERFACE:=enp130s0}"
: "${IPE_ZERO_COUNT:=130336}"
: "${IPE_DIRECTION:=-1}"
: "${IPE_VELOCITY_RAW_TO_RAD_S:=2.3731138426448658e-7}"

export IPE_PROJECT_DIR IPE_WORKSPACE
export IPE_ROS_DISTRO IPE_ROS_SETUP IPE_INTERFACE
export IPE_ZERO_COUNT IPE_DIRECTION IPE_VELOCITY_RAW_TO_RAD_S

ipe_source_ros() {
    if [[ ! -r "${IPE_ROS_SETUP}" ]]; then
        echo "ROS setup file not found: ${IPE_ROS_SETUP}" >&2
        echo "Install ROS 2 or set IPE_ROS_SETUP in ${IPE_PROJECT_DIR}/.env." >&2
        return 1
    fi
    set +u
    # shellcheck source=/dev/null
    source "${IPE_ROS_SETUP}"
    set -u
}

ipe_source_workspace() {
    local setup_file="${IPE_WORKSPACE}/install/setup.bash"
    if [[ ! -r "${setup_file}" ]]; then
        echo "Workspace is not built: ${setup_file}" >&2
        echo "Run ${IPE_PROJECT_DIR}/scripts/build_ros2_project.sh first." >&2
        return 1
    fi
    set +u
    # shellcheck source=/dev/null
    source "${setup_file}"
    set -u
}
