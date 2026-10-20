# g1_mj_description

This repository contains the G1 robot model for [mc_mujoco](https://github.com/rohanpsingh/mc_mujoco).

The package will install the files in this directory so that mc_mujoco can pick them up automatically.

All the G1 hardware revisions of [mc_g1](https://github.com/bastien-muraccioli/mc_g1) / [g1_description](https://github.com/bastien-muraccioli/g1_description) are supported (choose the model matching the `mode_machine` of your robot).

## Requirement 

Please make sure [mc_mujoco](https://github.com/rohanpsingh/mc_mujoco) is installed.

## Install

### From source

In order to install from source please run the following commands:

```bash
git clone https://github.com/isri-aist/g1_mj_description
cd g1_mj_description
mkdir build && cd build
cmake ..
make install
```

## To run

Your mc_rtc configuration file (typically ~/.config/mc_rtc/mc_rtc.yaml) should contain the following lines:

```yaml
# General mc_rtc configuration to run a G1 controller
MainRobot: G1_mode_15  # Use the mode_machine of your robot (G1_mode_<N>), see mc_g1. G1 is an alias of G1_29dof
Enabled: Posture  # Edit with your controller
Timestep: 0.001
```

Then run mc_mujoco:

```bash
mc_mujoco
```

## CMake options

- `SRC_MODE` if `ON` the files loaded by mujoco will point to the source rather than the installed files (default `OFF`)

## Supported models

The model file is selected from the mc_rtc robot name (`xml/<robot name>.xml`):

| mc_g1 module | `mode_machine` | MuJoCo model |
|---|:---:|---|
| `G1_mode_4` | 4 | `g1_23dof_rev_1_0` |
| `G1_mode_5` (`_no_hands`, `_Revo2`) | 5 | `g1_29dof_rev_1_0` (`_no_hands`, `_revo2`) |
| `G1_mode_6` (`_no_hands`, `_Revo2`) | 6 | `g1_29dof_lock_waist_rev_1_0` (`_no_hands`, `_revo2`) |
| `G1_mode_10` | 10 | `g1_23dof_mode_10` |
| `G1_mode_11` ... `G1_mode_16`, `G1_mode_18` (`_no_hands`, `_Revo2`) | 11 ... 18 | `g1_29dof_mode_<N>` (`_no_hands`, `_revo2`) |
| `G1_23dof` (deprecated hardware) | 1 | `g1_23dof` |
| `G1` / `G1_29dof`, `G1_no_hands`, `G1_Revo2` (deprecated hardware) | 2 | `g1_29dof`, `g1_29dof_no_hands`, `g1_29dof_revo2` |

Models of the waist-locked revisions (6, 12, 14, 16) have no waist roll/pitch joints (27 actuated joints).

### Generating the models of the hardware revisions

The models of the hardware revisions are generated from the Unitree URDF files of g1_description, using the hand-made models of the same family (`g1_23dof.xml`, `g1_29dof.xml`, `g1_29dof_no_hands.xml`, `g1_29dof_revo2.xml`) as templates: body poses, inertias (fixed links merged), meshes, joint limits, actuator ranges, locked joints and IMU sites come from the URDF, everything else (defaults, collisions, sensors, Revo2 hands) from the template. After changing a template or g1_description, regenerate them with:

```bash
python3 tools/generate_variants.py <path to g1_description> .
```

## IMUs

The model reproduces the two IMUs of the real G1, following the [mc_mujoco](https://github.com/rohanpsingh/mc_mujoco) convention (`<body sensor>_gyro` / `<body sensor>_accelerometer`) for the body sensors of [mc_g1](https://github.com/bastien-muraccioli/mc_g1):

| Site | Location (Unitree URDF) | MuJoCo sensors | mc_g1 body sensor |
|---|---|---|---|
| `imu` | pelvis, `imu_in_pelvis` | `Accelerometer_quat/_gyro/_accelerometer` | `Accelerometer` (main IMU, `rt/lowstate`) |
| `secondary_imu` | torso, `imu_in_torso` | `TorsoAccelerometer_quat/_gyro/_accelerometer` | `TorsoAccelerometer` (`rt/secondary_imu`) |
