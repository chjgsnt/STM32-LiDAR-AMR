# VSCode CMake Build

Date: 2026-05-19

Target: STM32 NUCLEO-F446RE

Toolchain: VSCode + STM32CubeCLT + CMake

## Current Status

`AMR_LiDAR_Robot/` is still an STM32CubeIDE-generated project. Current check:

- Missing `AMR_LiDAR_Robot/CMakeLists.txt`
- Missing `AMR_LiDAR_Robot/CMakePresets.json`
- Missing `AMR_LiDAR_Robot/cmake/`
- `AMR_LiDAR_Robot/AMR_LiDAR_Robot.ioc` still contains `ProjectManager.TargetToolchain=STM32CubeIDE`

These files are CubeMX-generated CMake project infrastructure. Do not hand-write a long-term replacement, because future CubeMX code generation can easily drift from manually maintained CMake settings.

## Required CubeMX Step

Open this file in STM32CubeMX:

```text
AMR_LiDAR_Robot/AMR_LiDAR_Robot.ioc
```

Then set:

```text
Project Manager -> Toolchain / IDE -> CMake
```

Run:

```text
Generate Code
```

After generation, confirm that `AMR_LiDAR_Robot/` contains:

- `CMakeLists.txt`
- `CMakePresets.json`
- `cmake/`

## Step2 Bring-Up Source Layout

The Step2 bring-up files that must participate in the build are already placed in CubeMX/CMake default source and include locations:

- `AMR_LiDAR_Robot/Core/Src/i2c_scan.c`
- `AMR_LiDAR_Robot/Core/Inc/i2c_scan.h`
- `AMR_LiDAR_Robot/Core/Inc/bringup_log.h`

Do not put required `.c` files only under `Firmware/App/Src`. If application code must be compiled from `Firmware/App/Src` later, also update the generated `CMakeLists.txt` to add the matching source files and include paths.

## PowerShell Build Commands

After regenerating the project from CubeMX with `Toolchain / IDE` set to `CMake`, run:

```powershell
cd "C:\Users\29525\Desktop\CodingDay\Microprocessor Systems Design\AMR_LiDAR_Robot"
cmake --list-presets
cmake --preset Debug
cmake --build --preset Debug
```

If `cmake --list-presets` does not show `Debug`, the CMake project has not been generated correctly. Reopen CubeMX, confirm `Toolchain / IDE` is `CMake`, and run `Generate Code` again.
