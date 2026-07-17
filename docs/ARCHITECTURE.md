# Architecture and Data Flow

## Production path

```text
JointTrajectory task target
  -> ipe_control reference management
  -> ipe_controllers real-time CST impedance control
  -> ethercat_master ros2_control hardware
  -> CiA 402 / SOEM / IPE joint
```

The production application uses the four-layer `ros2_ws` structure. Interactive
programs remain available for commissioning but are not production task control.

## Direct commissioning path

```text
Terminal command
  -> adapter/ipe_three_mode_lab or ipe/single_joint_lab
  -> ecat_motor_master (CiA 402, PDO map, mode, enable)
  -> ethercat_common (SOEM, cyclic exchange, WKC/DC, recovery)
  -> enp130s0 raw Ethernet frames
  -> IPE IRGML-14-I EtherCAT slave
```

`adapter` and `ipe` share the same EtherCAT core. The adapter provides the
interactive three-mode interface; it is not a second independent driver.

## Synchronous modes

- **CSP:** the master sends a target position cyclically. This firmware requires
  a fresh control-word bit-4 edge for each new motion segment.
- **CSV:** the master sends target velocity cyclically. A nonzero target continues
  rotation until explicitly zeroed.
- **CST:** the master sends target torque cyclically. Because the raw-to-Nm scale
  is not confirmed, project interfaces retain the `raw` suffix.

Commands and feedback use cyclic PDO exchange. Configuration and object lookup
normally use CoE SDO mailbox transactions.

## Production ROS 2 path

```text
MoveIt, task node, or send_joint_goal
  -> /ipe/command
  -> reference_manager at 50 Hz
  -> CstImpedanceController at 100 Hz
  -> ipe_joint/torque_raw
  -> IpeThreeModeSystem
  -> CiA 402 CST and 1 ms EtherCAT PDO loop
```

Feedback exports `position` in rad, `velocity` in rad/s, `velocity_raw`, and
`torque_raw`. The hardware layer owns identity checks, direction conversion,
mode switching, CiA 402 enable/disable, limits, PDO health, and safe zeroing.

The production Xacro maps raw velocity and torque into the ROS joint direction.
Legacy commissioning URDFs omit that mapping and preserve drive-native raw signs.

With `use_mock_hardware:=true`, `mock_components/GenericSystem` replaces the real
hardware plugin. Interfaces stay identical, but mock mode neither opens EtherCAT
nor simulates physical motor dynamics.

## Legacy ROS 2 plugins

- `IpeReadOnlySystem`: state only, no motion interface.
- `IpeCspSystem`: bounded position command for conservative CSP tests.
- `IpeThreeModeSystem`: mutually exclusive position, velocity_raw, and torque_raw
  command interfaces for commissioning.

Legacy and production launch files cannot run together because both directly own
the same network interface.

## Safety limits

The three-mode hardware validates identity and PDO health before enabling. It
enforces CSP travel/following error, CSV raw speed, CST raw effort, rated velocity,
and optional travel boundaries. The production CST controller adds reference
timeout, tracking error, output clamp, damping, and near-standstill breakaway logic.

## Concurrency rule

SOEM directly owns the selected interface. Multiple master processes on
`enp130s0` can cause configuration failure, invalid WKC, watchdog faults, or a
drive fault. Exactly one master process is allowed.
