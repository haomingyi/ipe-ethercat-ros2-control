# System Architecture

## 1. Architectural goals

The system is designed around five constraints:

1. Application code must use standard ROS messages instead of drive-native PDOs.
2. EtherCAT ownership, CiA 402 transitions, and hardware limits must remain in
   one hardware boundary.
3. Motion controllers must start inactive and acquire command interfaces
   explicitly.
4. Commissioning utilities must remain separate from the normal application
   path.
5. The complete ROS graph must run without physical hardware for integration
   development.

The result is a layered single-joint reference architecture, not a monolithic
motor test program.

## 2. System context

```text
┌───────────────────────────────────────────────────────────────┐
│ Application domain                                            │
│ MoveIt / task node / teleoperation / send_joint_goal          │
└──────────────────────────────┬────────────────────────────────┘
                               │ JointTrajectory
┌──────────────────────────────▼────────────────────────────────┐
│ Reference domain                                              │
│ Validation, time ordering, interpolation, stale-input policy  │
└──────────────────────────────┬────────────────────────────────┘
                               │ q_ref, dq_ref
┌──────────────────────────────▼────────────────────────────────┐
│ Control domain                                                │
│ ros2_control controller manager + CST impedance controller    │
└──────────────────────────────┬────────────────────────────────┘
                               │ torque_raw
┌──────────────────────────────▼────────────────────────────────┐
│ Hardware abstraction domain                                  │
│ IpeThreeModeSystem, units, limits, mode ownership, CiA 402    │
└──────────────────────────────┬────────────────────────────────┘
                               │ cyclic process data
┌──────────────────────────────▼────────────────────────────────┐
│ Transport and device domain                                  │
│ SOEM master -> raw Ethernet -> IPE IRGML-14-I EtherCAT drive │
└───────────────────────────────────────────────────────────────┘
```

Each downward interface reduces abstraction. Each upward interface converts
drive feedback into stable ROS-facing state.

## 3. Runtime components

### 3.1 Application layer

`ipe_control/send_joint_goal.py` is a small command-line client. A real robot
application can replace it with MoveIt, a behavior tree, a teleoperation node,
or another planner without changing the hardware layer.

The application contract is:

```text
topic: /ipe/command
type:  trajectory_msgs/JointTrajectory
unit:  rad and seconds
```

### 3.2 Reference layer

`ipe_control/reference_manager.py`:

- rejects trajectories without the configured joint;
- requires finite positions and strictly increasing timestamps;
- starts from measured state rather than an assumed position;
- interpolates references at 50 Hz;
- holds the final accepted position;
- publishes a continuous reference stream for timeout supervision.

This layer owns trajectory semantics. It does not own EtherCAT or drive state.

### 3.3 Controller layer

`ipe_controllers/CstImpedanceController` runs inside the controller manager at
100 Hz. Its primary law is:

```text
position_error = wrap(q_ref - q)
velocity_error = dq_ref - dq

torque_raw = Kp * position_error
           + Kd * velocity_error
           + breakaway_compensation
```

The controller additionally owns:

- reference freshness validation;
- position-error shutdown;
- output clamping;
- activation-time capture of the measured position;
- near-standstill breakaway compensation;
- controller-status telemetry;
- zero output on deactivation or invalid input.

It intentionally does not issue control words or EtherCAT frames.

### 3.4 Hardware abstraction layer

`ethercat_master/IpeThreeModeSystem` exports:

| Interface | Direction | ROS unit |
| --- | --- | --- |
| `position` | state and command | rad |
| `velocity` | state | rad/s |
| `velocity_raw` | state and command | drive-native |
| `torque_raw` | state and command | drive-native |

The plugin owns:

- hardware identity validation;
- encoder zero and direction conversion;
- mode selection and mutual exclusion;
- safe target initialization;
- CiA 402 enable/disable sequencing;
- CSP motion-edge handling;
- command, step, travel, and following-error limits;
- zeroing before mode changes and shutdown;
- PDO/WKC/drive-state supervision.

The plugin is the only component allowed to translate between ROS interfaces
and the low-level motor API.

### 3.5 EtherCAT core

`ecat_motor_master.c` owns the device-specific PDO layout and CiA 402 behavior.
`ethercat_common.c` owns SOEM initialization, the 1 ms process-data thread,
distributed-clock handling, WKC checks, and slave recovery.

```text
IpeThreeModeSystem
  -> ecat_motor_master
       -> target mapping and CiA 402 state machine
       -> ethercat_common
            -> SOEM
            -> network interface
```

## 4. Timing model

The stack uses nested rates:

| Loop | Nominal rate | Responsibility |
| --- | ---: | --- |
| EtherCAT process-data loop | 1 kHz | Keep PDO exchange alive and collect feedback |
| Controller manager | 100 Hz | Read state, update controller, write command |
| Reference manager | 50 Hz | Validate and interpolate application trajectories |

The rates are configurable only where exposed as parameters. Raising a ROS loop
rate does not change the drive's physical limits and must not be used to bypass
step or slew protection.

## 5. Lifecycle and motion authorization

The expected startup sequence is:

```text
process starts
  -> EtherCAT interface opens
  -> slave identity and PDO map are checked
  -> slave reaches OP
  -> ros2_control hardware becomes active
  -> state broadcaster starts
  -> motion controllers remain inactive
  -> operator activates exactly one motion controller
  -> hardware selects and verifies the requested CiA 402 mode
  -> safe measured-state target is primed
  -> drive enters Operation enabled
```

Loading a controller does not enable motion. Activation grants it command
interface ownership and may enable the drive after all hardware checks pass.

Deactivation reverses that authority:

```text
controller output zeroed
  -> drive command zeroed
  -> CiA 402 operation disabled
  -> command interfaces released
```

## 6. Fault-containment boundaries

| Failure | Detection owner | Response |
| --- | --- | --- |
| Missing/wrong slave | Hardware plugin | Refuse activation |
| EtherCAT not OP | EtherCAT core/plugin | Mark unhealthy; stop command |
| WKC below expected | EtherCAT core | Recovery attempt and unhealthy state |
| Drive fault/status change | Motor layer/plugin | Zero and disable |
| Reference timeout | CST controller | Zero output and report invalid reference |
| Excessive position error | CST controller | Zero output and deactivate/error |
| CSP following error | Hardware layer | Abort command and disable |
| Process shutdown | Hardware/plugin lifecycle | Zero, disable, close socket |

Software fault handling supplements but does not replace an independent
emergency power cut.

## 7. Operating backends

### Physical backend

`ethercat_master/IpeThreeModeSystem` opens the configured Linux network
interface and controls the real drive.

### Mock backend

`mock_components/GenericSystem` exposes the same ROS-facing interfaces without
opening EtherCAT. It is intended for launch, API, and controller integration.
It is not a motor or gearbox simulator.

Backend selection is a launch argument:

```bash
use_mock_hardware:=true   # software integration
use_mock_hardware:=false  # physical joint
```

## 8. Commissioning architecture

The direct tools deliberately bypass the application and controller layers:

```text
operator terminal
  -> ipe_three_mode_lab or single_joint_lab
  -> ecat_motor_master
  -> ethercat_common / SOEM
  -> drive
```

They share the production EtherCAT core, so their observations are relevant to
the hardware implementation. They are not a second driver and cannot run
concurrently with the ROS hardware plugin.

## 9. Concurrency and resource ownership

SOEM sends raw Ethernet frames through one network interface. The interface is
therefore an exclusive process resource:

```text
one interface -> one SOEM master -> one drive-control owner
```

Starting multiple masters can produce configuration failure, invalid WKC,
watchdog faults, or unintended mode transitions. Process exclusivity is an
operational requirement, not merely a recommendation.

## 10. Extension strategy

To integrate a higher-level application, publish `JointTrajectory`; do not
couple the application to raw EtherCAT data.

To add another joint:

1. Add its model and state/command interfaces to Xacro.
2. Extend the hardware plugin from scalar state to per-joint state.
3. Validate each slave identity and PDO map independently.
4. Add per-joint direction, zero, units, limits, and gain parameters.
5. Update reference and controller arrays.
6. Define a deterministic partial-failure policy before enabling coupled motion.

To harden the stack for stricter real-time use:

1. Replace the firmware-specific blocking CSP edge with a nonblocking state
   machine.
2. Remove dynamic work from controller update paths.
3. Add latency and deadline instrumentation.
4. Validate scheduling, CPU affinity, memory locking, and worst-case timing.
5. Add hardware-in-the-loop fault-injection tests.

## 11. Explicit non-goals

This repository does not provide:

- a safety PLC or certified safety function;
- a verified `torque_raw` to Nm conversion;
- a general EtherCAT configuration generator;
- a validated multi-axis synchronization policy;
- a realistic physics simulation;
- autonomous recovery from every mechanical or electrical fault.
