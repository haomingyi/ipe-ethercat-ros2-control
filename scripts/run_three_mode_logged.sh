#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="${project_dir}/adapter/build/ipe_three_mode_lab"
interface="${1:-enp130s0}"
log_dir="${project_dir}/artifacts/test-logs"
timestamp="$(date +%Y%m%d-%H%M%S)"
session_log="${log_dir}/three_mode_${timestamp}.log"
latest_log="${log_dir}/three_mode_latest.log"

if [[ ! "${interface}" =~ ^[[:alnum:]_.:-]+$ ]]; then
    echo "Invalid network interface name: ${interface}" >&2
    exit 2
fi

if [[ ! -x "${binary}" ]]; then
    echo "Three-mode program is not built: ${binary}" >&2
    echo "Build it from the repository root with:" >&2
    echo "  cmake -S adapter -B adapter/build -DCMAKE_BUILD_TYPE=Release" >&2
    echo "  cmake --build adapter/build -j" >&2
    exit 1
fi

mkdir -p "${log_dir}"
ln -sfn "$(basename "${session_log}")" "${latest_log}"

echo "Terminal cache: ${latest_log}"
echo "Timestamped copy: ${session_log}"
echo "The cache is local and excluded from Git."

# util-linux script records prompts, terminal input and output while preserving
# the interactive console. --flush makes the latest log readable during a run.
exec script --quiet --flush \
    --command "sudo stdbuf -oL -eL '${binary}' '${interface}'" \
    "${session_log}"
