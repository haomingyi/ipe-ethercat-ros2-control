#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
interface="${1:-enp130s0}"
control_node="${project_dir}/install-ros2/ethercat_master/lib/ethercat_master/ipe_ros2_control_node"
publisher="${project_dir}/install-ros2/ethercat_master/lib/ethercat_master/ipe_joint_state_publisher"

failures=0
ok() { printf '[OK] %s\n' "$1"; }
warn() { printf '[WARN] %s\n' "$1"; }
fail() { printf '[FAIL] %s\n' "$1"; failures=$((failures + 1)); }

if [[ -d "/sys/class/net/${interface}" ]]; then
    ok "network interface exists: ${interface}"
else
    fail "network interface not found: ${interface}"
fi

if [[ -r "/sys/class/net/${interface}/carrier" ]] &&
   [[ "$(<"/sys/class/net/${interface}/carrier")" == "1" ]]; then
    ok "physical Ethernet link is up"
else
    fail "physical Ethernet link is down"
fi

for executable in "${control_node}" "${publisher}"; do
    if [[ ! -e "${executable}" ]]; then
        fail "ROS executable is not built: ${executable}"
        continue
    fi
    binary="$(readlink -f "${executable}")"
    capability="$(getcap "${binary}" || true)"
    if [[ "${capability}" == *cap_net_admin* &&
          "${capability}" == *cap_net_raw* &&
          "${capability}" == *cap_sys_nice* ]]; then
        ok "EtherCAT capabilities: $(basename "${binary}")"
    else
        fail "missing EtherCAT capabilities: $(basename "${binary}")"
    fi
done

mapfile -t masters < <(
    pgrep -af 'single_joint_lab|ipe_three_mode_lab|ipe_joint_state_publisher|ipe_ros2_control_node' |
        grep -v 'check_ipe_environment.sh' || true
)
if (( ${#masters[@]} == 0 )); then
    ok "no EtherCAT master is currently running"
elif (( ${#masters[@]} == 1 )); then
    ok "one EtherCAT master is running"
    printf '     %s\n' "${masters[0]}"
else
    fail "more than one possible EtherCAT master is running"
    printf '     %s\n' "${masters[@]}"
fi

driver="$(basename "$(readlink -f "/sys/class/net/${interface}/device/driver" 2>/dev/null)" 2>/dev/null || true)"
[[ -n "${driver}" ]] && printf '[INFO] network driver: %s\n' "${driver}"
if command -v ethtool >/dev/null 2>&1; then
    rx_errors="$(ethtool -S "${interface}" 2>/dev/null |
        awk '/^[[:space:]]*rx_errors:/ {print $2; exit}')"
    align_errors="$(ethtool -S "${interface}" 2>/dev/null |
        awk '/^[[:space:]]*align_errors:/ {print $2; exit}')"
    printf '[INFO] NIC counters: rx_errors=%s align_errors=%s\n' \
        "${rx_errors:-unknown}" "${align_errors:-unknown}"
fi

if (( failures > 0 )); then
    printf 'Environment check failed: %d problem(s).\n' "${failures}"
    exit 1
fi
printf 'Environment check passed.\n'
