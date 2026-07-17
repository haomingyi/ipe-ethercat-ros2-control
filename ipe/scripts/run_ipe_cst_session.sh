#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "${project_dir}/install-ros2/setup.bash"
set -u

exec ros2 run ethercat_master ipe_cst_session
