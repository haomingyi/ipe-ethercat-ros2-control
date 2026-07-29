#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=project_env.sh
source "${script_dir}/project_env.sh"

if ! command -v apt-get >/dev/null 2>&1; then
    echo "This installer currently supports Debian/Ubuntu systems." >&2
    exit 1
fi

ipe_source_ros

sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    ethtool \
    libcap2-bin \
    python3-colcon-common-extensions \
    python3-rosdep \
    util-linux

if ! rosdep db >/dev/null 2>&1; then
    if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init
    fi
    rosdep update
fi

rosdep install \
    --from-paths "${IPE_PROJECT_DIR}/ipe" "${IPE_WORKSPACE}/src" \
    --ignore-src \
    --rosdistro "${IPE_ROS_DISTRO}" \
    --skip-keys ament_python \
    -r \
    -y

echo "Dependencies installed. Run scripts/check_system.sh next."
