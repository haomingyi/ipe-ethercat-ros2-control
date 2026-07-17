# Production CST Project Workflow

Use the [operations guide](OPERATIONS_GUIDE.md) for commands. This document
describes architecture and extension points.

## Objective

The production application does not stream a fixed raw torque. An application
provides a joint trajectory, a reference node interpolates it, and a real-time
controller calculates effort from position and velocity error. The hardware layer
owns CiA 402, PDO exchange, direction, limiting, and drive disable.

```text
Planner or task
  -> JointTrajectory on /ipe/command
  -> reference interpolation at 50 Hz
  -> CST impedance controller at 100 Hz
  -> ipe_joint/torque_raw
  -> EtherCAT hardware plugin
  -> 1 ms PDO loop and IPE drive
```

## Package responsibilities

| Package | Responsibility |
| --- | --- |
| `ethercat_master` | SOEM, PDO, CiA 402, ros2_control hardware |
| `ipe_description` | Xacro, joint direction, zero, interfaces, safety parameters |
| `ipe_controllers` | Real-time CST impedance controller |
| `ipe_control` | Trajectory validation, interpolation, command utility |
| `ipe_bringup` | Unified launch and deployment parameters |

`adapter`, `single_joint_lab`, and `ipe_cst_session` are commissioning tools, not
production application entry points.

## Build and launch

```bash
cd ~/ipe-ethercat-ros2-control
./scripts/build_ros2_project.sh
./scripts/setup_project_capability.sh
```

Mock launch:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ipe-ethercat-ros2-control/ros2_ws/install/setup.bash
ros2 launch ipe_bringup ipe_cst_project.launch.py use_mock_hardware:=true
```

Physical launch:

```bash
./scripts/run_cst_project.sh
```

Both launch paths start hardware, the state broadcaster, and reference manager.
Motion controllers start inactive.

## Production CST operation

Activation captures the measured position as a hold reference, so stale targets
are not applied:

```bash
ros2 control switch_controllers \
  --activate ipe_cst_impedance_controller \
  --strict
```

Relative target:

```bash
ros2 run ipe_control send_joint_goal \
  --relative-degrees 5 \
  --seconds 5
```

Absolute target:

```bash
ros2 run ipe_control send_joint_goal \
  --position-rad 1.0 \
  --seconds 5
```

Monitor:

```bash
ros2 topic echo /ipe_cst_impedance_controller/status
```

Stop and disable:

```bash
ros2 control switch_controllers \
  --deactivate ipe_cst_impedance_controller \
  --strict
```

## Control law

```text
torque_raw = Kp * wrap(q_ref - q)
           + Kd * (dq_ref - dq)
           + near-standstill breakaway compensation
```

Parameters in `ipe_bringup/config/controllers.yaml` define stiffness, damping,
breakaway magnitude, deadband, breakaway velocity threshold, command limit,
tracking-error limit, and reference timeout. Values are measured starting points
for one unloaded joint, not final loaded-robot tuning. Torque remains raw until
the manufacturer scale is verified.

## Application integration

MoveIt, a behavior tree, teleoperation, or an autonomous task only needs to
publish `trajectory_msgs/JointTrajectory` on `/ipe/command`. It does not need to
know about EtherCAT. For multiple joints, extend the URDF, hardware interfaces,
gain sets, and reference arrays while preserving the same layer boundaries.

## Entry-point boundary

- `ipe_cst_session`: manual fixed-raw commissioning.
- `send_ipe_raw_pulse`: short auto-zeroing validation pulse.
- `ipe_cst_impedance_controller`: production real-time closed loop.
- `/ipe/command`: production application API.

Only one hardware launch may run. Exit one backend completely before switching
between mock, production physical hardware, and legacy commissioning.
