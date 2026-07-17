# IPE EtherCAT Master

An EtherCAT and ROS 2 control project for one IPE IRGML-14-I integrated rotary
joint. The repository contains a production-oriented CST application, SOEM/CiA
402 commissioning tools, and conservative single-joint utilities. Production
and commissioning entry points are separate and must
never use the EtherCAT interface at the same time.

## Start here

- [Operations guide](docs/OPERATIONS_GUIDE.md): exact terminal layout, commands,
  controller roles, and shutdown procedure.
- [Project workflow](docs/PROJECT_WORKFLOW.md): production CST architecture and
  extension points.
- [Architecture](docs/ARCHITECTURE.md): EtherCAT, ROS 2, and control data flows.
- [Project layout](docs/PROJECT_LAYOUT.md): ownership of every major directory.
- [Safety checklist](docs/SAFETY.md): required checks before powered motion.

## Production entry point

The production application lives in `ros2_ws/`. It uses a 100 Hz CST impedance
controller. Applications publish standard joint trajectories instead of fixed
raw torque values.

```bash
cd ~/IPE-EtherCAT-ROS2-Control
./scripts/build_ros2_project.sh
./scripts/setup_project_capability.sh
./scripts/run_cst_project.sh
```

The same launch file supports two hardware backends:

- `use_mock_hardware:=true`: does not open an Ethernet interface; use it for ROS
  node, controller, and task-interface development.
- `use_mock_hardware:=false`: connects to `enp130s0`; the physical flange can
  move after a motion controller is activated.

## Directory map

| Path | Purpose |
| --- | --- |
| `ros2_ws/` | Production ROS 2 packages: description, controller, application, bringup |
| `ipe/` | EtherCAT core, ros2_control hardware, and single-joint tools |
| `adapter/` | Interactive CSP/CSV/CST commissioning console |
| `ipe/SOEM/` | Vendored upstream SOEM source tree |
| `docs/` | Operations, architecture, hardware, safety, and provenance documentation |
| `artifacts/` | Local logs and device captures; ignored by Git |
| `archives/` | Local source snapshots; ignored by Git |

## Requirements

- Ubuntu/Linux and ROS 2 Jazzy for ROS functionality
- CMake 3.16 or newer, a C/C++ compiler, pthread, and colcon
- One dedicated wired network interface connected directly to the EtherCAT joint
- Expected slave identity: vendor `0x00041101`, product `0x00009253`

Only one SOEM master may own an interface. Exit every `single_joint_lab`,
`ipe_three_mode_lab`, state publisher, or ros2_control node before starting a
different EtherCAT entry point.

## Quick production check with mock hardware

```bash
cd ~/IPE-EtherCAT-ROS2-Control
./scripts/build_ros2_project.sh
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch ipe_bringup ipe_cst_project.launch.py use_mock_hardware:=true
```

Mock hardware validates software wiring only. It does not model inertia,
friction, gearing, or physical CST response.

## Commissioning tools

Interactive three-mode console:

```bash
cd ~/IPE-EtherCAT-ROS2-Control
cmake -S adapter -B adapter/build -DCMAKE_BUILD_TYPE=Release
cmake --build adapter/build -j
./scripts/run_three_mode_logged.sh enp130s0
```

Conservative single-joint monitor:

```bash
cd ~/IPE-EtherCAT-ROS2-Control/ipe
cmake -S . -B build-safe -DCMAKE_BUILD_TYPE=Release
cmake --build build-safe --target single_joint_lab ipe_joint_units_test -j
ctest --test-dir build-safe --output-on-failure
sudo ./build-safe/single_joint_lab monitor
```

See [the adapter guide](adapter/README.md) and
[the single-joint learning guide](ipe/IPE_LEARNING_GUIDE.md) before issuing
motion commands.

## Safety boundary

Secure the joint body, clear the flange workspace, prepare an emergency power
cut, and verify that no cable can be wound by continuous rotation. CSV can rotate
continuously. CST can continuously accelerate an unloaded axis. Software limits
do not replace a physical emergency stop or power protection.

## Repository boundary

Build/install trees, IDE state, terminal logs, device captures, credentials, and
source archives are not committed. Run this before every publication:

```bash
./scripts/check_repository.sh
```

Third-party licensing boundaries are described in [NOTICE.md](NOTICE.md).
