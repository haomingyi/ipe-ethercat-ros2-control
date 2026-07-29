# EtherCAT Hardware Package

This package is the hardware boundary of the project. It contains the SOEM
transport, IPE-specific PDO and CiA 402 logic, the three-mode `ros2_control`
plugin, and the conservative standalone diagnostic tool.

Normal robot applications should use the repository-level launch and publish
`JointTrajectory`; they should not call this package directly.

## Responsibilities

```text
ros2_control command interfaces
  -> IpeThreeModeSystem
  -> ecat_motor_master
  -> ethercat_common
  -> SOEM
  -> IPE EtherCAT joint
```

| Component | Responsibility |
| --- | --- |
| `src/ipe_three_mode_ros2_control_hardware.cpp` | ROS units, command ownership, mode switching, limits |
| `src/ipe_ros2_control_node.cpp` | Capability-compatible controller-manager process |
| `includes/ecat_motor_master.c/.h` | PDO map, CiA 402 state, enable/disable, cyclic targets |
| `includes/ethercat_common.c/.h` | SOEM lifecycle, 1 ms PDO loop, WKC/DC, recovery |
| `src/single_joint_lab.cpp` | Passive monitoring and bounded direct diagnostics |
| `includes/ipe_joint_units.hpp` | Encoder and SI conversion |
| `tests/ipe_joint_units_test.cpp` | Hardware-free conversion tests |
| `SOEM/` | Vendored upstream EtherCAT master source |

The single `IpeThreeModeSystem` plugin exports position, raw velocity, and raw
torque interfaces. Controller activation determines which command interface is
owned; only one motion controller may be active.

## Standalone build

The diagnostic tool can be built without the complete ROS workspace:

```bash
cmake -S ipe -B ipe/build-safe -DCMAKE_BUILD_TYPE=Release
cmake --build ipe/build-safe \
  --target single_joint_lab ipe_joint_units_test \
  -j
ctest --test-dir ipe/build-safe --output-on-failure
```

Passive monitoring:

```bash
source scripts/project_env.sh
sudo ./ipe/build-safe/single_joint_lab \
  --interface "${IPE_INTERFACE}" \
  monitor
```

Other actions include `info`, `mode_probe`, `reset`, `hold`, bounded `move`, and
interactive `session`. Read [IPE_LEARNING_GUIDE.md](IPE_LEARNING_GUIDE.md)
before issuing a motion command.

## Tested device profile

| Property | Value |
| --- | --- |
| Slave name | `CoE Drives` |
| Vendor ID | `0x00041101` |
| Product ID | `0x00009253` |
| Expected one-slave WKC | `3` |
| Distributed clocks | Supported |
| Encoder | 18-bit absolute |
| Encoder resolution | 262,144 count/revolution |
| Measured reduction ratio | Approximately 101:1 |

The motor layer verifies the configured slave count and identity before
enabling. Raw torque is deliberately not labeled Nm because its physical scale
has not been confirmed.

## Firmware-specific CSP behavior

The tested firmware requires a fresh control-word bit-4 edge for a new CSP
motion segment: hold `0x000F`, then write `0x001F`. Updating the target while
leaving the control word at `0x001F` can change PDO target/demand/actual values
without visible flange motion. Both the standalone tool and the hardware plugin
implement the required arm/commit transition.

## Watchdog behavior

When cyclic communication stops, the drive may latch error `0x1002`. The
software treats fault reset as an explicit state transition and does not loop
automatic reset attempts. Long-running tools keep one EtherCAT session open to
avoid repeatedly tripping the watchdog.

## Safety boundary

- Exactly one SOEM master may own the selected network interface.
- Secure the joint and clear the flange workspace before enabling.
- Prime CSP with measured position before applying torque.
- Zero CSV/CST before switching modes or disabling.
- Treat raw velocity and torque as drive-native quantities.
- Do not infer loaded safe torque from an unloaded test.
- Keep an independent emergency power cut within reach.
