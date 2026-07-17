# Project File Index

## Production robot application

| Path | Responsibility |
| --- | --- |
| `ros2_ws/src/ipe_description/` | Robot model, zero, direction, interfaces, real/mock selection |
| `ros2_ws/src/ipe_controllers/` | Real-time CST impedance ros2_control plugin |
| `ros2_ws/src/ipe_control/` | Standard trajectory handling and application commands |
| `ros2_ws/src/ipe_bringup/` | Unified launch and controller parameters |
| `scripts/build_ros2_project.sh` | Build the complete multi-package workspace |
| `scripts/run_cst_project.sh` | Start the production physical-joint application |
| `docs/OPERATIONS_GUIDE.md` | Controller, script, terminal, and shutdown instructions |
| `docs/PROJECT_WORKFLOW.md` | Architecture, deployment, and extension guidance |

Only `ros2_ws/src/` and workspace documentation are committed. `build/`,
`install/`, and `log/` are reproducible local output.

## Production package internals

| File | Purpose |
| --- | --- |
| `ipe_description/urdf/ipe_single_joint.urdf.xacro` | Model, interfaces, safety parameters, hardware backend |
| `ipe_controllers/src/cst_impedance_controller.cpp` | Controller lifecycle and 100 Hz update loop |
| `ipe_controllers/include/ipe_controllers/impedance_law.hpp` | Independently testable CST control law |
| `ipe_control/ipe_control/reference_manager.py` | Trajectory validation and 50 Hz reference interpolation |
| `ipe_control/ipe_control/send_joint_goal.py` | Relative-degree or absolute-radian command utility |
| `ipe_bringup/config/controllers.yaml` | Rates, controller types, gains, and limits |
| `ipe_bringup/launch/ipe_cst_project.launch.py` | Unified process orchestration |

## EtherCAT and single-joint core

| File | Purpose |
| --- | --- |
| `adapter/src/ipe_three_mode_lab.cpp` | Interactive CSP/CSV/CST commissioning console |
| `ipe/src/single_joint_lab.cpp` | Monitor, reset, hold, bounded CSP motion, and learning session |
| `ipe/includes/ecat_motor_master.c/.h` | PDO mapping, CiA 402 state machine, modes, and commands |
| `ipe/includes/ethercat_common.c/.h` | SOEM initialization, 1 ms PDO loop, WKC/DC, recovery |
| `ipe/includes/cia402_def.h` | CiA 402 objects, status words, and control words |
| `ipe/includes/ipe_joint_units.hpp` | Count/radian conversion utilities |
| `ipe/tests/ipe_joint_units_test.cpp` | Hardware-free conversion tests |

## ros2_control hardware implementations

| File | Purpose |
| --- | --- |
| `ipe/src/ipe_ros2_control_hardware.cpp` | Read-only hardware plugin |
| `ipe/src/ipe_csp_ros2_control_hardware.cpp` | Standalone bounded CSP plugin |
| `ipe/src/ipe_three_mode_ros2_control_hardware.cpp` | Mutually exclusive CSP/CSV/CST plugin |
| `ipe/src/ipe_ros2_control_node.cpp` | Project controller-manager process |
| `ipe/ipe_ros2_control_plugin.xml` | pluginlib registration |
| `ipe/config/` | Controller-manager parameter files |
| `ipe/launch/` | Read-only and commissioning launch files |
| `ipe/scripts/` | Command, capability, and launch helpers |

## Local-only paths

- `artifacts/test-logs/`: terminal and motion logs
- `artifacts/device-captures/`: identity, SDO, and PDO captures
- `archives/`: original source snapshots
- every `build*`, `install*`, and `log*` tree

These paths are excluded by the root `.gitignore`.

## Version-control boundary

Commit source, build metadata, launch files, models, parameters, tests, and
maintained documentation. Do not commit build output, runtime logs, IDE state,
device captures, archives, or credentials. Treat `ipe/SOEM/` as vendored upstream
source and preserve its license when redistributing the repository.
