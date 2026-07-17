# IPE single-joint three-mode adapter

This directory is an independent adapter for the directly connected
IPE IRGML-14-I. It does not replace or edit files under `../ipe`; the original
archive is retained locally under `../archives/` and excluded from Git.

Build and start the long-running console from the repository root:

```bash
cmake -S adapter -B adapter/build -DCMAKE_BUILD_TYPE=Release
cmake --build adapter/build -j
./scripts/run_three_mode_logged.sh enp130s0
```

The wrapper uses `script` to keep an unbuffered local terminal transcript at
`../artifacts/test-logs/three_mode_latest.log`. The log is excluded from Git.
Run `sudo ./adapter/build/ipe_three_mode_lab enp130s0` directly only when a
transcript is not wanted.

The console keeps one EtherCAT connection open and switches the mapped
CiA 402 mode only while the drive is disabled. Commands are deliberately
limited because CSV and CST physical-unit scaling is not established by
the project files.

In CSP mode, `live` provides a multi-turn position-trajectory console. It
requires `NO_CABLE`, `CSP_LIVE`, and `ENABLE`, then accepts:

- `speed <rpm>` from 0.1 to the calibrated rated flange speed of about 29.7 rpm;
- `move_deg <relative-degrees>` or `turns <relative-turns>` (up to 100 turns
  per command);
- `hold`, `status`, and `stop`.

It preserves the IPE CSP bit-4 trigger sequence and stops on a following error
over 16384 count, PDO/WKC loss, a drive fault, or a position overflow.

In CSV mode, `live` enters an interactive continuous-speed session. It
requires the operator to confirm `NO_CABLE`, then `ENABLE`. While enabled:

- `set <raw>` sets a target in the range `-13107200` to `13107200`. A timed
  `200000 raw` test produced about 163 output degrees/minute, identifying an
  approximately 101:1 relationship between velocity raw and flange position.
  The hard limit therefore maps the documented 3000 motor rpm rating to about
  29.7 flange rpm. This is an in-system calibration, not a certified load-test
  rating.
- `add <raw>` adjusts the current target by an increment.
- `status` prints the live EtherCAT and drive state.
- `stop` ramps to zero, disables the drive, and returns to the main console.

The live command has no cumulative position limit because it is only for a
joint confirmed to have no center-axis internal cable. PDO health, WKC,
CiA 402 fault state, signal handling, speed limiting, and slew limiting stay
active throughout the session.

CSV changes by `1000 raw` per millisecond, so a zero-to-rated transition takes
about 13.1 seconds. This slew remains active for both `set` and `stop`.

In CST mode, `live` provides the same `set` / `add` / `status` / `stop`
interaction with an unloaded-test hard limit of `+/-300 raw`. Commands above
the previously tested `+/-100 raw` boundary print an additional warning. It requires
`NO_CABLE`, `CST_LIVE`, and `ENABLE` confirmations, changes torque by only one
raw unit every 20 ms, and automatically zeros/disables if actual velocity
exceeds the calibrated `+/-13107200 raw` motor rated-speed boundary. Torque
scaling to Nm is still unverified, so this is
not a certified rated-torque or thermal test.
