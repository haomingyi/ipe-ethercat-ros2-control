# IPE ROS 2 Workspace

This is the production robot workspace. Source packages live in `src/`; the root
build script also includes the hardware package from `../ipe`.

```text
src/
  ipe_description/   URDF/Xacro and real/mock ros2_control hardware selection
  ipe_controllers/   Real-time CST impedance controller
  ipe_control/       Trajectory reference manager and command utilities
  ipe_bringup/       Launch files and deployment parameters
```

Use the scripts in the repository root for building and launching. See
`../docs/OPERATIONS_GUIDE.md` for commands and `../docs/PROJECT_WORKFLOW.md` for
implementation details.
