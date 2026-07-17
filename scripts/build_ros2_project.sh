#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${project_dir}/ros2_ws"

set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "${workspace}"
colcon --log-base log build \
  --base-paths "${project_dir}/ipe" "${workspace}/src" \
  --build-base build \
  --install-base install \
  --symlink-install \
  --packages-up-to ipe_bringup

echo "Build complete. Run scripts/setup_project_capability.sh after a relink."
