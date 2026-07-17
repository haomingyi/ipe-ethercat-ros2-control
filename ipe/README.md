# EtherCAT Master Hardware Package

This directory contains the IPE EtherCAT core, standalone commissioning tools,
ros2_control plugins, and focused commissioning utilities. The production
robot application is launched from the repository root and documented in
[the unified operations guide](../docs/OPERATIONS_GUIDE.md).

## Recommended single-joint entry points

Build conservative tools:

```bash
cd ~/IPE-EtherCAT-ROS2-Control/ipe
cmake -S . -B build-safe -DCMAKE_BUILD_TYPE=Release
cmake --build build-safe --target single_joint_lab ipe_joint_units_test -j
ctest --test-dir build-safe --output-on-failure
```

Monitor without enabling:

```bash
sudo ./build-safe/single_joint_lab monitor
```

Other `single_joint_lab` actions are `reset`, `hold`, bounded `move`, and
interactive `session`. Read [IPE_LEARNING_GUIDE.md](IPE_LEARNING_GUIDE.md) before
using them.

## EtherCAT observations for the tested joint

- Product: IPE IRGML-14-I integrated rotary joint
- Slave name: `CoE Drives`
- Vendor ID: `0x00041101`
- Product ID: `0x00009253`
- One-slave expected WKC: `3`
- Distributed clocks: supported
- Absolute encoder: 18 bits, 262,144 counts per encoder revolution
- Measured reduction ratio: approximately 101:1

The program refuses motion if identity or PDO health checks fail. Raw torque is
not labeled Nm because its physical scale has not been confirmed.

## Production ROS 2 application

The production workspace is `../ros2_ws`, not this directory's standalone install
tree. Build and start from the repository root:

```bash
cd ~/IPE-EtherCAT-ROS2-Control
./scripts/build_ros2_project.sh
./scripts/setup_project_capability.sh
./scripts/run_cst_project.sh
```

It provides a standard trajectory input, reference interpolation, a real-time CST
impedance controller, and the `IpeThreeModeSystem` hardware plugin. Do not source
`ipe/install-ros2` when operating the production application.

## Legacy ROS 2 read-only state publisher

Build this package independently for low-level diagnostics:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log-ros2 build --packages-select ethercat_master \
  --build-base build-ros2 --install-base install-ros2 --symlink-install
./scripts/setup_ethercat_capability.sh
./scripts/run_ipe_joint_state_publisher.sh
```

It publishes `/joint_states` and does not provide a motion command interface.

## Read-only ros2_control

```bash
./scripts/run_ipe_ros2_control.sh
```

This loads `IpeReadOnlySystem`, `joint_state_broadcaster`, robot state publisher,
and TF. It exports state only. Its empty `write()` path is intentional.

## Standalone CSP

```bash
./scripts/run_ipe_csp_control.sh
```

The position controller starts inactive. Explicit activation selects CSP and
enables the drive; deactivation zeros and disables it. The old delta utility is:

```bash
ros2 run ethercat_master send_ipe_csp_delta --degrees 1
```

This path is retained for compatibility and conservative commissioning, not
normal production motion.

## Three-mode ros2_control

```bash
./scripts/run_ipe_three_mode_control.sh
```

The launch loads one state broadcaster and three mutually exclusive motion
controllers:

- `ipe_position_controller` for CSP position in radians
- `ipe_velocity_raw_controller` for CSV drive-native raw velocity
- `ipe_torque_raw_controller` for CST drive-native raw torque

The state broadcaster starts active; all motion controllers start inactive.
Controller switching disables the old mode, zeros safe targets, writes 0x6060,
verifies 0x6061, and only then enables the new mode.

Command utilities:

```bash
ros2 run ethercat_master send_ipe_csp_trajectory --degrees 90 --rpm 5
ros2 run ethercat_master send_ipe_raw_pulse --mode csv --value 200000 --seconds 0.8
ros2 run ethercat_master send_ipe_raw_pulse --mode cst --value 50 --seconds 0.2
./scripts/run_ipe_cst_session.sh
```

`ipe_cst_session` provides `set`, `add`, `status`, `zero`, and `stop` for manual
raw-torque commissioning. It is not the production CST controller and must not
run while the production impedance controller is active.

## CSP firmware timing

The tested IRGML firmware requires a fresh control-word bit-4 edge for a new CSP
motion: hold `0x000F`, then write `0x001F`. Merely changing the target while the
control word remains `0x001F` can update PDO target/demand/actual values without
starting visible flange motion. Current single-joint and hardware-plugin paths
implement the required arm/commit sequence.

## Watchdog fault behavior

After communication stops, the drive may latch error `0x1002`. Long-running
sessions avoid repeated watchdog trips. A later session may perform an explicit
fault reset after identity and PDO checks. Automatic repeated reset is disabled.

## Source structure

### Common EtherCAT core

| File | Purpose |
| --- | --- |
| `includes/ethercat_common.c/.h` | SOEM init, 1 ms PDO thread, WKC/DC, recovery |
| `includes/ecat_motor_master.c/.h` | PDO map, CiA 402, modes, enable/disable, targets |
| `includes/cia402_def.h` | Object and state definitions |
| `includes/ipe_joint_units.hpp` | Count and SI conversion |

### Current single-joint and ROS code

| File | Purpose |
| --- | --- |
| `src/single_joint_lab.cpp` | Conservative direct utility |
| `src/ipe_joint_state_publisher.cpp` | Read-only ROS state node |
| `src/ipe_ros2_control_hardware.cpp` | Read-only hardware plugin |
| `src/ipe_csp_ros2_control_hardware.cpp` | Standalone CSP plugin |
| `src/ipe_three_mode_ros2_control_hardware.cpp` | CSP/CSV/CST hardware plugin |
| `src/ipe_ros2_control_node.cpp` | Project controller-manager executable |
| `ipe_ros2_control_plugin.xml` | pluginlib registration |

## Core C API summary

`ecat_motor_master.h` exposes passive/active initialization, explicit enable,
mode switching, cyclic-motion arm/commit, disable, fault reset, command update,
state access, and shutdown. The core verifies the configured joint count and
slave identity before enabling.

## Safety notes

- Use one master per interface.
- Fix the body and clear the flange workspace.
- Prime CSP with actual position before enable.
- Zero CSV/CST before switching or stopping.
- Treat raw velocity and torque as drive-native quantities unless explicitly
  converted by the production hardware description.
- Do not infer safe torque from an unloaded test.
- Keep a physical emergency power cut within reach.
