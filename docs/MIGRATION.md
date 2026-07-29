# Migration and Reproducibility

## What moves between computers

Clone the Git repository. Do not manually copy a previously built workspace.

The repository contains:

- application, controller, and hardware-interface source;
- the required SOEM source and upstream license;
- launch, Xacro, controller, and reference configuration;
- standalone commissioning tools;
- build, test, setup, and environment-check scripts;
- hardware-free automated tests.

The repository intentionally does not contain:

- `build`, `install`, or `log` output;
- Linux file capabilities;
- network-interface names for a specific new host;
- runtime logs or device captures;
- credentials or a developer's shell environment.

Those items are generated or configured on the target computer.

## Target-computer procedure

### 1. Install ROS 2

Install ROS 2 Jazzy using the official ROS packages for the target Ubuntu
release. If ROS is installed outside `/opt/ros/jazzy`, set the actual setup-file
path in `.env`.

### 2. Clone and configure

```bash
git clone https://github.com/haomingyi/ipe-ethercat-ros2-control.git
cd ipe-ethercat-ros2-control
cp .env.example .env
ip -brief link
```

Edit `.env` and set `IPE_INTERFACE` to the dedicated wired adapter connected to
the EtherCAT joint. Keep the checked-in calibration values only for the same
joint and mechanical installation.

### 3. Install dependencies

```bash
./scripts/install_dependencies.sh
./scripts/check_system.sh
```

`install_dependencies.sh` installs general Ubuntu build/runtime tools and runs
`rosdep` against all five ROS packages.

### 4. Build from source

```bash
./scripts/build_ros2_project.sh
./scripts/test_project.sh
```

All paths are derived from the checkout location. The repository may be cloned
under any user account and any directory.

### 5. Validate without hardware

```bash
./scripts/run_mock_project.sh
```

In another terminal:

```bash
source scripts/project_env.sh
ipe_source_ros
ipe_source_workspace
ros2 control list_controllers
```

Stop mock mode before continuing.

### 6. Prepare physical access

Linux file capabilities are inode metadata and do not survive Git clone. Apply
them to the executable built on the target computer:

```bash
./scripts/setup_project_capability.sh
./scripts/check_system.sh --hardware
```

Only continue when the correct network adapter exists, its carrier is up, and
no other EtherCAT master is running.

## Moving an existing checkout

CMake and colcon caches contain absolute paths. After moving or renaming a
checkout, remove only generated output and rebuild:

```bash
./scripts/clean_workspace.sh --yes
./scripts/build_ros2_project.sh
./scripts/test_project.sh
```

Do not repair a stale `CMakeCache.txt` manually.

## Reproducibility boundary

The software build is reproducible from the repository and declared ROS
dependencies. Physical behavior additionally depends on:

- joint firmware and PDO map;
- vendor/product identity;
- encoder zero and direction;
- reducer ratio and velocity scale;
- supply voltage/current limits;
- payload, friction, mounting, and temperature.

Record new hardware calibration as reviewed configuration changes. Do not hide
machine- or joint-specific values inside source code.
