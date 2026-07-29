#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=project_env.sh
source "${script_dir}/project_env.sh"

if [[ "${1:-}" != "--yes" || $# -ne 1 ]]; then
    echo "This removes generated build/install/log output only." >&2
    echo "Usage: $0 --yes" >&2
    exit 2
fi

targets=(
    "${IPE_PROJECT_DIR}/adapter/build"
    "${IPE_PROJECT_DIR}/ipe/build-safe"
    "${IPE_PROJECT_DIR}/ipe/build-verify"
    "${IPE_WORKSPACE}/build"
    "${IPE_WORKSPACE}/build-verify"
    "${IPE_WORKSPACE}/install"
    "${IPE_WORKSPACE}/install-verify"
    "${IPE_WORKSPACE}/log"
    "${IPE_WORKSPACE}/log-verify"
)

printf 'Removing generated directories:\n'
printf '  %s\n' "${targets[@]}"
rm -rf -- "${targets[@]}"
echo "Generated workspace output removed."
