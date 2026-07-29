#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=project_env.sh
source "${script_dir}/project_env.sh"

ipe_source_ros

cd "${IPE_WORKSPACE}"
colcon --log-base log build \
  --base-paths "${IPE_PROJECT_DIR}/ipe" "${IPE_WORKSPACE}/src" \
  --build-base build \
  --install-base install \
  --symlink-install \
  --packages-up-to ipe_bringup

echo "Build complete. Run scripts/setup_project_capability.sh after a relink."
