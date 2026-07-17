# Unified IPE Joint Operations Guide

This guide explains which program to start, what every controller does, and
which terminal should run each command. Use the production CST application for
normal robot work. Every other entry point is a commissioning or diagnostic tool.

## Choose one entry point

| Goal | Entry point | Physical joint | Intended use |
| --- | --- | --- | --- |
| Develop ROS nodes and controller wiring | Production launch with mock hardware | No | Daily software development |
| Execute joint trajectories | Production CST application | Yes | Normal robot application |
| Verify CSP, CSV, or CST independently | `adapter/ipe_three_mode_lab` | Yes | Low-level commissioning |
| Diagnose legacy ros2_control interfaces | Legacy three-mode launch | Yes | Compatibility and diagnosis |
| Read joint feedback only | Read-only node or plugin | Yes | Diagnosis |
| Reset, hold, or move a few counts | `single_joint_lab` | Yes | Initial learning and repair |

Never start two entries simultaneously. SOEM owns `enp130s0` directly, and only
one EtherCAT master may use that interface.

## Do not mix the two ROS environments

Production commands use:

```bash
source /opt/ros/jazzy/setup.bash
source ~/IPE-EtherCAT-ROS2-Control/ros2_ws/install/setup.bash
```

Legacy `ipe/scripts/run_ipe_*.sh` commands use:

```bash
source /opt/ros/jazzy/setup.bash
source ~/IPE-EtherCAT-ROS2-Control/ipe/install-ros2/setup.bash
```

Paths containing `ros2_ws/install` are production. Paths containing
`ipe/install-ros2` are legacy commissioning.

## Production controllers

The launch file loads every controller, but only the state broadcaster starts
active. A loaded controller does not mean that the drive is enabled.

| Controller | Interface | Role | Normal use |
| --- | --- | --- | --- |
| `joint_state_broadcaster` | State only | Publish position, velocity, and raw feedback | Always active |
| `ipe_cst_impedance_controller` | `torque_raw` | Compute CST effort from position and velocity errors | Production motion |
| `ipe_position_controller` | `position` | Pass a CSP position target directly | Commissioning fallback |
| `ipe_velocity_raw_controller` | `velocity_raw` | Pass a CSV raw target directly | Commissioning fallback |
| `ipe_torque_raw_controller` | `torque_raw` | Pass a CST raw target directly | Commissioning fallback |

Activate only one motion controller. Normal production operation activates only
`ipe_cst_impedance_controller`.

## Standard ROS commands versus project code

| Name | Origin | Purpose |
| --- | --- | --- |
| `ros2 launch`, `ros2 run`, `ros2 topic` | ROS 2 CLI | Start nodes, run executables, inspect topics |
| `ros2 control list_controllers` | ros2_control CLI | Query controller-manager state |
| `ros2 control switch_controllers` | ros2_control CLI | Activate or deactivate controllers |
| `joint_state_broadcaster` | Standard ros2_controllers plugin | Publish hardware state |
| `JointGroupPositionController` | Standard ros2_controllers plugin | Forward a position command |
| `ForwardCommandController` | Standard ros2_controllers plugin | Forward a raw velocity or torque command |
| `IpeThreeModeSystem` | Project C++ plugin | Connect SOEM, CiA 402, and the joint |
| `CstImpedanceController` | Project C++ plugin | Calculate real-time CST commands |
| `reference_manager` | Project Python node | Validate and interpolate trajectories |
| `send_joint_goal` | Project Python utility | Send one relative or absolute target |

The `switch_controllers` syntax below is standard ROS, while the controller name
and algorithm are project-defined:

```bash
ros2 control switch_controllers \
  --activate ipe_cst_impedance_controller \
  --strict
```

## Production data flow

```text
send_joint_goal, MoveIt, or a task node
  -> /ipe/command (JointTrajectory)
  -> reference_manager (started by launch)
  -> /ipe_cst_impedance_controller/reference
  -> ipe_cst_impedance_controller
  -> ipe_joint/torque_raw
  -> IpeThreeModeSystem
  -> CiA 402 CST / EtherCAT / IPE joint
```

Do not start `reference_manager` manually. `send_joint_goal` exits after
publishing one command; that does not stop the launch or controller.

## Build and capability setup

Run after the first checkout or after changing code, Xacro, or parameters:

```bash
cd ~/IPE-EtherCAT-ROS2-Control
./scripts/build_ros2_project.sh
./scripts/setup_project_capability.sh
```

The capability script grants raw-network and scheduling capabilities only to the
project node. Re-run it if that executable is relinked and `getcap` becomes empty.

## Mock operation

Terminal 1:

```bash
cd ~/IPE-EtherCAT-ROS2-Control
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch ipe_bringup ipe_cst_project.launch.py use_mock_hardware:=true
```

Terminal 2:

```bash
cd ~/IPE-EtherCAT-ROS2-Control
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 control list_controllers
ros2 control switch_controllers \
  --activate ipe_cst_impedance_controller \
  --strict
ros2 run ipe_control send_joint_goal \
  --relative-degrees 5 \
  --seconds 5
ros2 topic echo /ipe_cst_impedance_controller/status --once
ros2 control switch_controllers \
  --deactivate ipe_cst_impedance_controller \
  --strict
```

Press `Ctrl+C` in terminal 1 afterward. Mock hardware validates software wiring
but does not simulate the motor, reducer, friction, or physical torque response.

## Physical-joint daily operation

### Before power-on

Secure the body, clear the flange workspace, prepare an emergency power cut, and
verify that no center-bore cable can be wound. Keep hands outside the motion path.

Check for an existing master:

```bash
pgrep -af 'single_joint_lab|ipe_three_mode_lab|ipe_joint_state_publisher|ipe_ros2_control_node'
```

No output means no master is running. If a process appears, exit it normally in
its original terminal with `quit` or `Ctrl+C`.

### Terminal 1: start and keep running

```bash
cd ~/IPE-EtherCAT-ROS2-Control
./scripts/run_cst_project.sh
```

Output is cached in `artifacts/test-logs/cst_project_latest.log`. The line
`Operational state reached for all slaves` confirms EtherCAT OP. The drive remains
disabled until terminal 2 activates a motion controller.

### Terminal 2: inspect, enable, move, disable

Prepare every new terminal:

```bash
cd ~/IPE-EtherCAT-ROS2-Control
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

Inspect the system:

```bash
ros2 control list_hardware_components -v
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Expected state: hardware `active`, state broadcaster `active`, every motion
controller `inactive`.

Enable production CST control:

```bash
ros2 control switch_controllers \
  --activate ipe_cst_impedance_controller \
  --strict
```

Start with a small relative target:

```bash
ros2 run ipe_control send_joint_goal \
  --relative-degrees 5 \
  --seconds 5
```

Reverse direction:

```bash
ros2 run ipe_control send_joint_goal \
  --relative-degrees -5 \
  --seconds 5
```

Absolute ROS position in radians:

```bash
ros2 run ipe_control send_joint_goal \
  --position-rad 1.0 \
  --seconds 5
```

Disable motion control:

```bash
ros2 control switch_controllers \
  --deactivate ipe_cst_impedance_controller \
  --strict
```

Finally press `Ctrl+C` in terminal 1 to return EtherCAT to a safe state.

### Terminal 3: optional monitoring

Source the production workspace, then use:

```bash
ros2 topic echo /joint_states
ros2 topic echo /dynamic_joint_states
ros2 topic echo /ipe_cst_impedance_controller/status
```

Use `--once` to print one message. Press `Ctrl+C` to stop continuous output.

| CST status field | Meaning |
| --- | --- |
| `position` | Actual ROS joint position in rad |
| `velocity` | Actual ROS joint velocity in rad/s |
| `position_reference` | Current interpolated position reference in rad |
| `velocity_reference` | Current interpolated velocity reference in rad/s |
| `torque_command_raw` | Controller command in ROS joint direction |
| `torque_actual_raw` | Drive feedback in ROS joint direction |
| `reference_valid` | `1` for a fresh reference; `0` after timeout or deactivation |

## Current CST parameters

Parameters are in `ros2_ws/src/ipe_bringup/config/controllers.yaml`:

```text
torque_raw = Kp * position_error + Kd * velocity_error + breakaway compensation
```

- `kp_raw_per_rad=120`: position stiffness.
- `kd_raw_per_rad_s=40`: velocity damping.
- `breakaway_raw=60`: measured no-load breakaway compensation.
- `position_deadband_rad=0.01`: do not apply breakaway compensation inside about 0.57 degrees.
- `breakaway_velocity_threshold_rad_s=0.02`: apply breakaway only near standstill.
- `max_command_raw=100`: production controller hard limit.
- `max_position_error_rad=0.35`: stop above about 20.1 degrees of error.
- `reference_timeout_sec=0.20`: stop if the reference stream is stale.

These are starting values for one unloaded joint, not final loaded-robot tuning.
Do not begin with 360 degrees in 5 seconds. Validate 5, 10, and 20 degrees while
watching tracking error, current, and temperature.

## Commissioning tools

Interactive direct console:

```bash
cd ~/IPE-EtherCAT-ROS2-Control
./scripts/run_three_mode_logged.sh enp130s0
```

It runs `adapter/build/ipe_three_mode_lab` and switches CSP/CSV/CST within one
EtherCAT connection. Do not run a ROS EtherCAT launch at the same time.

Conservative single-joint monitor:

```bash
cd ~/IPE-EtherCAT-ROS2-Control/ipe
sudo ./build-safe/single_joint_lab monitor
```

The legacy ros2_control three-mode launch is:

```bash
cd ~/IPE-EtherCAT-ROS2-Control/ipe
./scripts/run_ipe_three_mode_control.sh
```

Its utilities are `send_ipe_csp_trajectory`, `send_ipe_raw_pulse`, and
`ipe_cst_session`. They use `ipe/install-ros2/setup.bash` and are not production
commands. Never run `ipe_cst_session` while the production impedance controller
is active.

Read-only legacy entry points are `run_ipe_joint_state_publisher.sh` and
`run_ipe_ros2_control.sh`; neither is a motion interface.

## Root script index

| Script | Purpose | Motion behavior |
| --- | --- | --- |
| `build_ros2_project.sh` | Build production workspace | No motion |
| `setup_project_capability.sh` | Set executable capabilities | No motion |
| `run_cst_project.sh` | Start physical production launch | No motion until activation |
| `run_three_mode_logged.sh` | Start direct commissioning console | Can move after commands |
| `check_repository.sh` | Check publication boundary | No motion |

## Common failures

- **Another EtherCAT master is already running:** exit the existing master; do
  not start a second one.
- **Controller is inactive:** it is loaded but does not own hardware. This is the
  correct stopped state.
- **Trajectory is accepted but the flange does not move:** inspect command,
  velocity, reference validity, and the cached log. `Accepted` only confirms input.
- **Tracking error exceeds the limit:** the drive was intentionally zeroed and
  disabled. Reduce the target/rate and inspect load, friction, direction, and tuning.
- **Reference timeout:** reference publication or ROS communication stopped; inspect
  `reference_manager` and its reference topic.
- **30 ms write overrun:** the first IPE motion edge currently uses a blocking
  trigger. It is not a PDO loss, but production real-time work should replace it
  with a nonblocking state machine.
- **A message was lost:** usually a temporary `ros2 topic echo` subscription issue,
  not an EtherCAT failure. Judge EtherCAT from OP, WKC, PDO, and drive errors.

## Safe shutdown

1. Stop sending trajectories.
2. Deactivate `ipe_cst_impedance_controller`.
3. Confirm it is `inactive`.
4. Press `Ctrl+C` in the launch terminal.
5. Turn off DC power last.

In an emergency, cut drive power immediately instead of waiting for software.
