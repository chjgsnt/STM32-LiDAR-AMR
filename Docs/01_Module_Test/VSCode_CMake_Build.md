# VSCode CMake Build

Date: 2026-05-19

Target: STM32 NUCLEO-F446RE

Toolchain: VSCode + STM32CubeCLT + CMake

## Current Status

`AMR_LiDAR_Robot/` now contains the CubeMX-generated CMake project infrastructure:

- `AMR_LiDAR_Robot/CMakeLists.txt`
- `AMR_LiDAR_Robot/CMakePresets.json`
- `AMR_LiDAR_Robot/cmake/`

Verified result:

- `cmake --build --preset Debug` completes successfully.
- The board can be programmed through ST-LINK.
- UART serial logging works on `COM11`.
- FreeRTOS starts and prints the bring-up log.

## Step2 Bring-Up Source Layout

The Step2 bring-up files that must participate in the build are placed in CubeMX/CMake default source and include locations or explicitly listed in the top-level CMake user source section:

- `AMR_LiDAR_Robot/Core/Src/i2c_scan.c`
- `AMR_LiDAR_Robot/Core/Inc/i2c_scan.h`
- `AMR_LiDAR_Robot/Core/Inc/bringup_log.h`
- `AMR_LiDAR_Robot/Core/Src/ssd1306.c`
- `AMR_LiDAR_Robot/Core/Inc/ssd1306.h`

Do not put required `.c` files only under `Firmware/App/Src`. If application code must be compiled from `Firmware/App/Src` later, also update the generated `CMakeLists.txt` to add the matching source files and include paths.

## PowerShell Build Commands

Run:

```powershell
cd "C:\Users\29525\Desktop\CodingDay\Microprocessor Systems Design\AMR_LiDAR_Robot"
cmake --list-presets
cmake --preset Debug
cmake --build --preset Debug
```

If `cmake --list-presets` does not show `Debug`, regenerate the project from CubeMX with `Toolchain / IDE` set to `CMake`.
