# Fully Embedded 2D-LiDAR Autonomous Mobile Robot

## Hardware Platform

- MCU board: STM32 NUCLEO-F446RE
- Sensor target: RPLidar C1 2D LiDAR
- IMU target: MPU6500
- Display target: SSD1306 OLED
- Actuation target: DC motors with encoders

## Development Toolchain

- STM32CubeMX
- VSCode with STM32CubeIDE extension
- STM32CubeCLT

## Project Progress / Bring-Up Status

- Step 0 created the baseline project file structure for later firmware, documentation, tools, and tests.
- Step 1 created the first STM32CubeMX configuration for core bring-up peripherals and middleware.
- Step 2 verified the generated configuration on hardware with USART2 logging, FreeRTOS startup, I2C1 scanning, and basic I2C module bring-up.
- MPU6500 IMU bring-up is verified on I2C1 at address `0x68`: `WHO_AM_I = 0x70`, raw accelerometer/gyroscope data, `g` / `dps` conversion, gyro bias calibration, pitch/roll, complementary-filter fused pitch/roll, and App-layer readout are working.
- Step 3 verified the AT8236 motor driver path, Motor A/B PWM forward/reverse output, and TIM2/TIM4 encoder counting.
- Step 4 Chassis Open-Loop Test is complete. The chassis open-loop path has been verified with `MPU6500 IMU ready=1`, AT8236 motor driver output, Motor A/B PWM forward/reverse, TIM2/TIM4 encoder counting, and chassis `Forward`, `Backward`, `TurnLeft`, and `TurnRight` actions.
- Step 5 Encoder Speed Balance Test is complete. With `base_duty=500`, `CHASSIS_LEFT_TRIM=-10`, and `CHASSIS_RIGHT_TRIM=10`, P-only balance reduces left/right wheel speed difference. During the second half of forward motion, `err` can usually be reduced to about `+/-10 ticks/sample`. Backward motion can run, but `err` fluctuates slightly more and will be optimized in the later PID stage. `MPU6500 IMU ready=1` remained valid during the test.
- Step 6 Wheel Speed PI Closed-Loop Test is complete. With `target_ticks_per_sample=800`, `feedforward_duty=500`, and duty limited to `300..600`, both Forward PI and Backward PI ran to completion. Startup duty briefly reached `600`, then stabilized around `470..500`; later encoder deltas were mostly `810..830 ticks/sample`, close to the `800` target. `MPU6500 IMU ready=1` remained valid, and I2C baseline stayed normal with `SCL=1`, `SDA=1`.
- Step 7 Heading Hold Test has a saved baseline. It uses MPU6500 gyro-Z yaw integration above the Step6 wheel speed PI loop, with `yaw_error = current_yaw - target_yaw` and explicit left/right target correction. Current conservative parameters are `HEADING_K_FORWARD=2`, `HEADING_K_BACKWARD=2`, and correction limits of `50 ticks/sample` in both directions. Forward heading hold is clearly improved with final yaw around `7.1 deg`; backward can run but still has about `+7 deg` final error relative to its target. `MPU6500 IMU ready=1` remained valid and I2C stayed normal.
- Step 8 Test Mode and Logging Cleanup is in progress. Test selection is centralized through `APP_ACTIVE_TEST` in `Core/Inc/app_config.h`, only one bring-up test is enabled by default, UART logs now use the `APP_LOG` / `LOG_INFO` path with a FreeRTOS log mutex, and I2C scan verbosity is controlled by `APP_I2C_SCAN_VERBOSE`.
- Step 9 RPLIDAR C1 UART4 bring-up preparation is recorded. RPLIDAR C1 uses TTL UART on `UART4` at `460800` baud, `8N1`, with `PC10` as `UART4_TX` and `PC11` as `UART4_RX`. Hardware wiring is complete; the next step is raw UART reception code and rx byte-count verification.
- Final chassis direction configuration: `CHASSIS_LEFT_SIGN = -1`, `CHASSIS_RIGHT_SIGN = 1`.
- Current wheel speed PI test target: `800 ticks/sample`; feedforward duty: `500`; duty limit: `300..600`.
- Current known issue: backward heading hold still needs tuning; LiDAR raw UART reception and navigation integration are not completed yet.

## Planned Firmware Modules

- LiDAR data reception and RPLidar C1 packet parsing
- MPU6500 IMU interface
- SSD1306 OLED display interface
- Motor control
- Encoder feedback
- PID control
- Occupancy grid mapping
- Navigation and path planning
- UART debug logging
- FreeRTOS application tasks
- Application configuration

## Evidence Recording

Use the `Docs/` folders to keep project evidence organized:

- `Docs/01_Module_Test/`: module bring-up notes, including MPU6500 IMU validation and RPLIDAR C1 UART4 bring-up
- `Docs/Progress_Report.md`: project bring-up progress report
- `Docs/02_Debug_Logs/`: serial logs and runtime traces
- `Docs/03_OLED_Photos/`: OLED display photos during tests
- `Docs/04_Logic_Analyzer/`: UART, I2C, PWM, and encoder captures
- `Docs/05_LiDAR_Raw_Data/`: raw LiDAR frames and parsing notes
- `Docs/06_PID_Tuning/`: PID tuning tables and test observations
- `Docs/07_Map_Output/`: occupancy grid snapshots and exported map data
- `Docs/08_Benchmark/`: timing, CPU load, memory, and latency measurements
- `Docs/09_Git_Records/`: milestone notes and change records

## Directory Overview

- `Firmware/App/`: application-level entry points and FreeRTOS task placeholders
- `Firmware/Drivers_User/`: user-maintained hardware driver modules
- `Firmware/Middleware_User/`: user-maintained control, mapping, navigation, and utility modules
- `Firmware/Config/`: application configuration headers
- `Docs/`: design notes, test evidence, debug records, and experiment outputs
- `Tools/`: host-side scripts and serial helper tools
- `Tests/`: unit and hardware test placeholders

## Next Step

Add raw UART4 reception code for RPLIDAR C1 and verify rx byte count at `460800` baud, while keeping backward heading correction / odometry-yaw fusion as the next control tuning item.
