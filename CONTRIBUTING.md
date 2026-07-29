# Contributing

## Development principles

Changes should preserve the project boundaries:

- application code publishes standard ROS messages;
- controllers compute commands but do not manage EtherCAT;
- the hardware plugin owns units, limits, modes, and CiA 402 transitions;
- the EtherCAT core owns process-data exchange and slave recovery;
- commissioning tools remain separate from the application launch.

Do not duplicate PDO structures, conversion constants, or enable sequences in a
new application node.

## Naming

- C++ types: `PascalCase`
- C/C++ functions and variables: `snake_case`
- Python modules, functions, and ROS packages: `snake_case`
- ROS topics and interfaces: lowercase names with `/` or `_`
- Units in identifiers when ambiguity is possible, for example
  `velocity_raw`, `position_rad`, or `timeout_sec`

Names such as `value`, `data`, or `command` must be qualified when they cross a
unit or ownership boundary.

## Local validation

Build the complete workspace:

```bash
./scripts/build_ros2_project.sh
```

Run all hardware-free tests:

```bash
./scripts/test_project.sh
```

## Hardware-affecting changes

A pull request that changes any item below must state the tested hardware,
load condition, command range, observed fault behavior, and shutdown result:

- PDO layout or CiA 402 state transitions
- encoder scale, zero, direction, or gear ratio
- command, travel, following-error, slew, or timeout limits
- controller gains or breakaway compensation
- controller activation/deactivation behavior
- EtherCAT recovery or shutdown paths

Start with mock validation, then passive hardware observation, then bounded
motion. Never make unattended hardware movement part of CI.

## Pull requests

Keep each change focused. A pull request should explain:

1. the problem and affected layer;
2. why the change belongs in that layer;
3. user-visible behavior;
4. safety impact;
5. validation performed;
6. known limitations that remain.

Generated build output, runtime logs, device captures, credentials, and private
manufacturer files must not be committed.
