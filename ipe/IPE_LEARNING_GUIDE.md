# IPE IRGML-14-I Single-Joint Learning Guide

This guide covers the conservative `single_joint_lab` utility. It is a
commissioning tool, not the production ROS 2 application.

## Safety assumptions

- The joint body is fixed and the flange workspace is clear.
- A 48 V-class supply is configured with an appropriate current limit.
- An emergency power disconnect is reachable.
- The interface configured as `IPE_INTERFACE` is connected directly to one IPE
  EtherCAT joint.
- No other EtherCAT master is running.

## Build

```bash
cd ~/ipe-ethercat-ros2-control
cmake -S ipe -B ipe/build-safe -DCMAKE_BUILD_TYPE=Release
cmake --build ipe/build-safe --target single_joint_lab ipe_joint_units_test -j
ctest --test-dir ipe/build-safe --output-on-failure
```

`cmake -S . -B build-safe` configures a separate build directory.
`cmake --build` compiles only the requested safe targets. Rebuild after source
changes; normal repeated operation does not require rebuilding.

## Command overview

```text
monitor       Read feedback without enabling motion
reset         Perform an explicit fault-reset sequence only
hold          Reset if needed, enable CSP, and hold the current position
move <count>  Move by a bounded relative count and return
session       Keep one EtherCAT connection open for interactive learning
mode_probe    Check whether a cyclic mode can be selected while disabled
```

Run the monitor first after every power cycle:

```bash
source scripts/project_env.sh
sudo ./ipe/build-safe/single_joint_lab \
  --interface "${IPE_INTERFACE}" \
  monitor
```

Expected healthy indicators include one slave, EtherCAT OP, WKC `3/3`, and
`pdo=OK`. Press `Ctrl+C` to stop monitoring.

## Understanding the feedback line

Example fields:

```text
position=131020 velocity=0 torque_raw=-18 status=0x1637 error=0 pdo=OK
```

- `position`: signed absolute-encoder count from the drive.
- `velocity`: drive-native raw velocity.
- `torque_raw`: drive-native raw torque feedback, not confirmed Nm.
- `status`: CiA 402 status word and decoded state.
- `error`: drive error object.
- `pdo=OK`: cyclic process data and expected WKC are healthy.

## Encoder counts

The drive reports an 18-bit absolute encoder:

```text
2^18 = 262144 counts per encoder revolution
```

One count is one encoder subdivision, not one revolution. The value 262,144 is
not universal; it comes from this encoder resolution. This project measured an
approximately 101:1 reduction ratio, so motor/encoder motion and flange motion
must not be confused.

Useful conversions:

```text
encoder degrees = count * 360 / 262144
count/s = encoder deg/s * 262144 / 360
```

Production ROS conversion also accounts for reduction ratio, zero, and direction.

## Fault reset and watchdog latch

Error `0x1002` has repeatedly appeared after communication stops. It is treated
as a communication-watchdog latch. A reset clears the stored fault condition; it
does not solve a broken cable, duplicate master, or unhealthy PDO exchange.

```bash
sudo ./ipe/build-safe/single_joint_lab --interface "${IPE_INTERFACE}" reset
```

Only reset after checking the status and physical connection. Repeated automatic
fault reset is intentionally disabled.

## Hold test

```bash
sudo ./ipe/build-safe/single_joint_lab --interface "${IPE_INTERFACE}" hold
```

Follow the terminal confirmations exactly. The utility primes the target with the
measured position before enabling CSP, preventing a jump to a stale target. A
healthy result reaches `Operation enabled` and holds the current position.

## Bounded movement

```bash
sudo ./ipe/build-safe/single_joint_lab --interface "${IPE_INTERFACE}" move 500
sudo ./ipe/build-safe/single_joint_lab --interface "${IPE_INTERFACE}" move -500
```

Do not type angle brackets. Documentation notation such as `move <count>` means
replace `<count>` with a number; the literal characters `<` and `>` are not part
of the command.

The utility limits one request to a small relative count, observes following
error, and returns safely. The displayed encoder-derived angle is not necessarily
the output-flange angle.

## Interactive session

```bash
sudo ./ipe/build-safe/single_joint_lab --interface "${IPE_INTERFACE}" session
```

The session keeps one connection alive and avoids watchdog faults caused by
repeated process startup and shutdown. Use `status`, reset/enable commands shown
by the prompt, small `move` values, `return`, and `quit`. Always use `quit` for a
normal disable and safe EtherCAT shutdown.

## CSP, CSV, and CST

- CSP controls cyclic target position.
- CSV controls cyclic target velocity.
- CST controls cyclic target torque.

The direct adapter console is better for comparing all three modes:

```bash
cd ~/ipe-ethercat-ros2-control
./scripts/run_three_mode_logged.sh
```

Use the production ROS application only after low-level identity, PDO, direction,
and basic motion have been established.

## Power-cycle procedure

1. Turn on the supply and wait for the drive to boot.
2. Check link LEDs and `ip link show "${IPE_INTERFACE}"`.
3. Confirm that no other master process is running.
4. Run `monitor` and verify slave identity, OP, WKC, PDO, status, and error.
5. Reset only if the drive is in Fault.
6. Continue with hold or a small motion.

The network interface name does not change merely because drive power was cycled.
Rebuilding is unnecessary unless software changed.

## Stop conditions

Cut power or stop immediately on unexpected motion, rising speed, repeated WKC
errors, PDO unhealthy state, a new drive fault, excessive noise, temperature, or
supply current. Software output is not a substitute for physical observation.
