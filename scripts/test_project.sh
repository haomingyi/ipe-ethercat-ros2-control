#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=project_env.sh
source "${script_dir}/project_env.sh"

ipe_source_ros
ipe_source_workspace

colcon test \
    --base-paths "${IPE_WORKSPACE}/src" "${IPE_PROJECT_DIR}/ipe" \
    --build-base "${IPE_WORKSPACE}/build" \
    --install-base "${IPE_WORKSPACE}/install" \
    --packages-select ipe_control ipe_controllers
colcon test-result \
    --test-result-base "${IPE_WORKSPACE}/build" \
    --verbose

cmake \
    -S "${IPE_PROJECT_DIR}/ipe" \
    -B "${IPE_PROJECT_DIR}/ipe/build-safe" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "${IPE_PROJECT_DIR}/ipe/build-safe" \
    --target single_joint_lab ipe_joint_units_test \
    -j
ctest \
    --test-dir "${IPE_PROJECT_DIR}/ipe/build-safe" \
    --output-on-failure

cmake \
    -S "${IPE_PROJECT_DIR}/adapter" \
    -B "${IPE_PROJECT_DIR}/adapter/build" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "${IPE_PROJECT_DIR}/adapter/build" -j

"${IPE_PROJECT_DIR}/scripts/check_repository.sh"
